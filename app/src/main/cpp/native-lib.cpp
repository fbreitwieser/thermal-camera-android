#include <jni.h>
#include <string>
#include <atomic>
#include <mutex>
#include <cmath>
#include <cstring>
#include <thread>
#include <chrono>
#include <android/log.h>
#include <libusb.h>
#include "libuvc/libuvc.h"

#define LOG_TAG "ThermalCamera"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// Output dimensions shared by all supported cameras
static constexpr int FRAME_WIDTH = 256;
static constexpr int IMAGE_HEIGHT = 192;   // Top of frame: visual image
static constexpr int THERMAL_HEIGHT = 192; // Below: 16-bit thermal data

struct DeviceGeometry {
    int frameHeight;
    int thermalStartRow;
};

static constexpr DeviceGeometry INFIRAY_GEOMETRY = {384, 192};
static constexpr DeviceGeometry THERMAL_MASTER_P2_GEOMETRY = {386, 194};
static DeviceGeometry g_geometry = INFIRAY_GEOMETRY;

static DeviceGeometry selectDeviceGeometry(jint vendorId, jint productId) {
    if (vendorId == 0x3474 && productId == 0x4281) {
        return THERMAL_MASTER_P2_GEOMETRY;
    }
    return INFIRAY_GEOMETRY;
}

// Global state
static uvc_context_t *g_ctx = nullptr;
static uvc_device_handle_t *g_devh = nullptr;
static std::atomic<bool> g_streaming{false};
static std::mutex g_mutex;

// Java callback references
static JavaVM *g_jvm = nullptr;
static jobject g_callback_obj = nullptr;
static jmethodID g_callback_method = nullptr;

static uvc_error_t getStreamCtrlForAnyFrameRate(
        uvc_device_handle_t *devh,
        uvc_stream_ctrl_t *ctrl,
        int width,
        int height) {
    static constexpr uint8_t YUYV_GUID[16] = {
            'Y', 'U', 'Y', '2', 0x00, 0x00, 0x10, 0x00,
            0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
    };

    const uvc_format_desc_t *format = uvc_get_format_descs(devh);
    for (; format != nullptr; format = format->next) {
        if (std::memcmp(format->guidFormat, YUYV_GUID, sizeof(YUYV_GUID)) != 0) {
            continue;
        }
        for (const uvc_frame_desc_t *frame = format->frame_descs;
             frame != nullptr; frame = frame->next) {
            if (frame->wWidth != width || frame->wHeight != height) {
                continue;
            }
            if (frame->intervals != nullptr) {
                for (const uint32_t *interval = frame->intervals; *interval != 0; ++interval) {
                    int fps = static_cast<int>(10000000 / *interval);
                    if (fps > 0) {
                        uvc_error_t res = uvc_get_stream_ctrl_format_size(
                                devh, ctrl, UVC_FRAME_FORMAT_YUYV, width, height, fps);
                        if (res == UVC_SUCCESS) {
                            LOGI("Falling back to %d fps (advertised)", fps);
                            return res;
                        }
                    }
                }
            } else {
                if (frame->dwMinFrameInterval == 0 ||
                    frame->dwMaxFrameInterval < frame->dwMinFrameInterval ||
                    frame->dwFrameIntervalStep == 0) {
                    continue;
                }
                uint32_t minFps = static_cast<uint32_t>(
                        (10000000ULL + frame->dwMaxFrameInterval - 1) /
                        frame->dwMaxFrameInterval);
                uint32_t maxFps = 10000000 / frame->dwMinFrameInterval;
                for (uint32_t fps = minFps; fps <= maxFps; ++fps) {
                    uint32_t interval = 10000000 / fps;
                    uint32_t offset = interval - frame->dwMinFrameInterval;
                    if (interval >= frame->dwMinFrameInterval &&
                        interval <= frame->dwMaxFrameInterval &&
                        !(offset && offset % frame->dwFrameIntervalStep)) {
                        LOGI("Falling back to %u fps (continuous range)", fps);
                        return uvc_get_stream_ctrl_format_size(
                                devh, ctrl, UVC_FRAME_FORMAT_YUYV,
                                width, height, static_cast<int>(fps));
                    }
                }
            }
        }
    }
    return UVC_ERROR_INVALID_MODE;
}

// Convert raw thermal value to Celsius
static inline float rawToCelsius(uint16_t raw) {
    return (raw / 64.0f) - 273.15f;
}

// Frame counter for debug logging
static int g_frame_count = 0;

// Hysteresis state for min/max marker stability
static int prevMinRow = 0, prevMinCol = 0;
static int prevMaxRow = 0, prevMaxCol = 0;
static bool hasInitialized = false;

// Temperature rounding mode: 0 = none, 1 = 0.2°C, 2 = 0.5°C
static int g_rounding_mode = 0;

static inline float roundTemperature(float temp) {
    switch (g_rounding_mode) {
        case 1: // Round to 0.2
            return std::round(temp * 5.0f) / 5.0f;
        case 2: // Round to 0.5
            return std::round(temp * 2.0f) / 2.0f;
        default:
            return temp;
    }
}

// Frame callback from libuvc
static void frameCallback(uvc_frame_t *frame, void *user_ptr) {
    g_frame_count++;
    if (g_frame_count % 25 == 1) {  // Log every ~1 second at 25fps
        LOGI("Frame received #%d: %zu bytes, %dx%d", g_frame_count,
             frame ? frame->data_bytes : 0,
             frame ? (int)frame->width : 0,
             frame ? (int)frame->height : 0);
    }

    if (!frame || !frame->data || frame->data_bytes == 0) {
        LOGE("Invalid frame received");
        return;
    }

    size_t expected_size = FRAME_WIDTH * g_geometry.frameHeight * 2;
    if (frame->data_bytes < expected_size) {
        LOGE("Frame too small: %zu < %zu", frame->data_bytes, expected_size);
        return;
    }

    uint8_t *data = static_cast<uint8_t*>(frame->data);
    size_t stride = FRAME_WIDTH * 2; // YUYV: 2 bytes per pixel

    // Allocate grayscale buffer for top half (visual image)
    // We extract Y channel from YUYV
    uint8_t grayscale[FRAME_WIDTH * IMAGE_HEIGHT];

    // Extract grayscale from top 192 rows (visual image)
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < FRAME_WIDTH; x++) {
            // YUYV format: Y0 U Y1 V - we just need Y values
            grayscale[y * FRAME_WIDTH + x] = data[y * stride + x * 2];
        }
    }

    // Process the 192 thermal rows, which start at a device-specific offset
    // Each pixel is 16-bit LE stored in YUYV as: lo byte in Y, hi byte in U/V
    float minTemp = 1000.0f, maxTemp = -1000.0f, sumTemp = 0.0f;
    int minRow = 0, minCol = 0, maxRow = 0, maxCol = 0;

    uint8_t *thermal_data = data + (g_geometry.thermalStartRow * stride);

    for (int y = 0; y < THERMAL_HEIGHT; y++) {
        for (int x = 0; x < FRAME_WIDTH; x++) {
            // Extract 16-bit LE value from YUYV
            // Low byte is in Y position, high byte follows
            uint8_t lo = thermal_data[y * stride + x * 2];
            uint8_t hi = thermal_data[y * stride + x * 2 + 1];
            uint16_t raw = lo | (hi << 8);
            float temp = rawToCelsius(raw);

            sumTemp += temp;

            if (temp < minTemp) {
                minTemp = temp;
                minRow = y;
                minCol = x;
            }
            if (temp > maxTemp) {
                maxTemp = temp;
                maxRow = y;
                maxCol = x;
            }
        }
    }

    // Hysteresis: keep previous location if temperature difference is small
    // Threshold matches rounding mode: 0.1 (none), 0.2, or 0.5
    float hysteresisThreshold;
    switch (g_rounding_mode) {
        case 1: hysteresisThreshold = 0.2f; break;
        case 2: hysteresisThreshold = 0.5f; break;
        default: hysteresisThreshold = 0.1f; break;
    }

    if (hasInitialized) {
        // Get temp at previous min location
        uint8_t lo = thermal_data[prevMinRow * stride + prevMinCol * 2];
        uint8_t hi = thermal_data[prevMinRow * stride + prevMinCol * 2 + 1];
        float prevMinLocTemp = rawToCelsius(lo | (hi << 8));

        if (prevMinLocTemp - minTemp < hysteresisThreshold) {
            minRow = prevMinRow;
            minCol = prevMinCol;
            minTemp = prevMinLocTemp;
        }

        // Same for max location
        lo = thermal_data[prevMaxRow * stride + prevMaxCol * 2];
        hi = thermal_data[prevMaxRow * stride + prevMaxCol * 2 + 1];
        float prevMaxLocTemp = rawToCelsius(lo | (hi << 8));

        if (maxTemp - prevMaxLocTemp < hysteresisThreshold) {
            maxRow = prevMaxRow;
            maxCol = prevMaxCol;
            maxTemp = prevMaxLocTemp;
        }
    }

    // Update previous locations
    prevMinRow = minRow; prevMinCol = minCol;
    prevMaxRow = maxRow; prevMaxCol = maxCol;
    hasInitialized = true;

    float avgTemp = sumTemp / (THERMAL_HEIGHT * FRAME_WIDTH);

    // Center temperature (middle of thermal region)
    int centerY = THERMAL_HEIGHT / 2;
    int centerX = FRAME_WIDTH / 2;
    uint8_t clo = thermal_data[centerY * stride + centerX * 2];
    uint8_t chi = thermal_data[centerY * stride + centerX * 2 + 1];
    uint16_t centerRaw = clo | (chi << 8);
    float centerTemp = rawToCelsius(centerRaw);

    // Apply rounding to displayed temperatures
    centerTemp = roundTemperature(centerTemp);
    minTemp = roundTemperature(minTemp);
    maxTemp = roundTemperature(maxTemp);
    avgTemp = roundTemperature(avgTemp);

    // Build linearly-normalized thermal image from raw 16-bit values.
    // Guarantees: pixel 0 = minTemp, pixel 255 = maxTemp, linear in between.
    // Used by the locked scale colormap path to avoid AGC-induced color jumps.
    uint8_t thermalLinear[FRAME_WIDTH * THERMAL_HEIGHT];
    {
        float range = maxTemp - minTemp;
        if (range < 0.001f) range = 0.001f;
        for (int y = 0; y < THERMAL_HEIGHT; y++) {
            for (int x = 0; x < FRAME_WIDTH; x++) {
                uint8_t lo = thermal_data[y * stride + x * 2];
                uint8_t hi = thermal_data[y * stride + x * 2 + 1];
                uint16_t raw = lo | (hi << 8);
                float norm = (rawToCelsius(raw) - minTemp) / range;
                norm = fmaxf(0.0f, fminf(1.0f, norm));
                thermalLinear[y * FRAME_WIDTH + x] = (uint8_t)(norm * 255.0f + 0.5f);
            }
        }
    }

    // Call Java callback
    if (g_jvm && g_callback_obj && g_callback_method) {
        JNIEnv *env;
        bool attached = false;
        int getEnvResult = g_jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);

        if (getEnvResult == JNI_EDETACHED) {
            if (g_jvm->AttachCurrentThread(&env, nullptr) == JNI_OK) {
                attached = true;
            } else {
                LOGE("Failed to attach thread to JVM");
                return;
            }
        } else if (getEnvResult != JNI_OK) {
            LOGE("Failed to get JNI environment");
            return;
        }

        // Create byte arrays for camera-AGC grayscale and linear thermal image
        jbyteArray imageArray = env->NewByteArray(FRAME_WIDTH * IMAGE_HEIGHT);
        env->SetByteArrayRegion(imageArray, 0, FRAME_WIDTH * IMAGE_HEIGHT,
                                reinterpret_cast<jbyte*>(grayscale));

        jbyteArray thermalLinearArray = env->NewByteArray(FRAME_WIDTH * THERMAL_HEIGHT);
        env->SetByteArrayRegion(thermalLinearArray, 0, FRAME_WIDTH * THERMAL_HEIGHT,
                                reinterpret_cast<jbyte*>(thermalLinear));

        // Call: onFrame(byte[] imageData, byte[] thermalImageData,
        //               float centerTemp, float minTemp, float maxTemp, float avgTemp,
        //               int minRow, int minCol, int maxRow, int maxCol)
        env->CallVoidMethod(g_callback_obj, g_callback_method,
                           imageArray, thermalLinearArray,
                           centerTemp, minTemp, maxTemp, avgTemp,
                           minRow, minCol, maxRow, maxCol);

        env->DeleteLocalRef(imageArray);
        env->DeleteLocalRef(thermalLinearArray);

        if (attached) {
            g_jvm->DetachCurrentThread();
        }
    }
}

extern "C" JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM *vm, void *reserved) {
    g_jvm = vm;
    return JNI_VERSION_1_6;
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_breyt_thermalcamera_MainActivity_getLibUvcVersion(
        JNIEnv *env,
        jobject /* this */) {
    const char *version = LIBUVC_VERSION_STR;
    LOGI("libuvc version: %s", version);
    return env->NewStringUTF(version);
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_breyt_thermalcamera_MainActivity_initUvc(
        JNIEnv *env,
        jobject /* this */) {
    uvc_context_t *ctx = nullptr;
    uvc_error_t res = uvc_init(&ctx, nullptr);
    if (res < 0) {
        LOGE("uvc_init failed: %s", uvc_strerror(res));
        return JNI_FALSE;
    }
    LOGI("uvc_init succeeded");
    uvc_exit(ctx);
    return JNI_TRUE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_breyt_thermalcamera_ThermalCamera_nativeOpen(
        JNIEnv *env,
        jobject /* this */,
        jint fd,
        jint vendorId,
        jint productId) {

    std::lock_guard<std::mutex> lock(g_mutex);

    // Clean up any stale state from previous session
    if (g_devh != nullptr || g_ctx != nullptr) {
        LOGW("Cleaning up stale camera state before opening");
        if (g_streaming) {
            uvc_stop_streaming(g_devh);
            g_streaming = false;
        }
        if (g_callback_obj) {
            env->DeleteGlobalRef(g_callback_obj);
            g_callback_obj = nullptr;
        }
        g_callback_method = nullptr;
        if (g_devh) {
            uvc_close(g_devh);
            g_devh = nullptr;
        }
        if (g_ctx) {
            uvc_exit(g_ctx);
            g_ctx = nullptr;
        }
        // Reset hysteresis state
        hasInitialized = false;
        prevMinRow = prevMinCol = prevMaxRow = prevMaxCol = 0;
        g_frame_count = 0;
    }

    LOGI("Opening camera with fd=%d, VID=%04x, PID=%04x", fd, vendorId, productId);
    g_geometry = selectDeviceGeometry(vendorId, productId);
    LOGI("Using frame 256x%d, thermal data at row %d",
         g_geometry.frameHeight, g_geometry.thermalStartRow);

    // On Android, we must set LIBUSB_OPTION_NO_DEVICE_DISCOVERY before init
    // because Android restricts direct access to /dev/bus/usb.
    // Setting on NULL context affects all future contexts (global default).
    int usb_res = libusb_set_option(nullptr, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);
    if (usb_res != LIBUSB_SUCCESS && usb_res != LIBUSB_ERROR_NOT_SUPPORTED) {
        LOGE("libusb_set_option(NO_DEVICE_DISCOVERY) failed: %s", libusb_strerror((libusb_error)usb_res));
        // Continue anyway - older libusb versions don't support this
    } else {
        LOGI("libusb_set_option(NO_DEVICE_DISCOVERY) success");
    }

    // Initialize UVC context - pass NULL so libuvc creates its own libusb context
    // and starts the event handler thread (required for receiving frames)
    uvc_error_t res = uvc_init(&g_ctx, nullptr);
    if (res < 0) {
        LOGE("uvc_init failed: %s", uvc_strerror(res));
        return JNI_FALSE;
    }
    LOGI("uvc_init succeeded");

    // Wrap the Android USB file descriptor directly with libuvc
    // uvc_wrap takes: file descriptor, context, output device handle
    // Retry with delay if device is busy (common when reconnecting quickly)
    // Note: If a kernel driver (V4L2) has claimed the device, retries won't help
    const int maxRetries = 3;
    const int retryDelayMs = 150;

    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        res = uvc_wrap(fd, g_ctx, &g_devh);
        if (res == UVC_SUCCESS) {
            break;
        }
        if (res == UVC_ERROR_BUSY && attempt < maxRetries) {
            LOGW("uvc_wrap returned BUSY, retrying in %dms (attempt %d/%d)",
                 retryDelayMs, attempt, maxRetries);
            std::this_thread::sleep_for(std::chrono::milliseconds(retryDelayMs));
        } else {
            LOGE("uvc_wrap failed: %s (attempt %d/%d)", uvc_strerror(res), attempt, maxRetries);
            uvc_exit(g_ctx);
            g_ctx = nullptr;
            return JNI_FALSE;
        }
    }

    LOGI("Camera opened successfully");
    return JNI_TRUE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_breyt_thermalcamera_ThermalCamera_nativeStartStream(
        JNIEnv *env,
        jobject thiz,
        jobject callback) {

    std::lock_guard<std::mutex> lock(g_mutex);

    if (g_devh == nullptr) {
        LOGE("Camera not open");
        return JNI_FALSE;
    }

    if (g_streaming) {
        LOGE("Already streaming");
        return JNI_FALSE;
    }

    // Store callback reference
    g_callback_obj = env->NewGlobalRef(callback);
    jclass callbackClass = env->GetObjectClass(callback);
    // Signature: (byte[] imageData, byte[] thermalImageData,
    //             float centerTemp, float minTemp, float maxTemp, float avgTemp,
    //             int minRow, int minCol, int maxRow, int maxCol)V
    g_callback_method = env->GetMethodID(callbackClass, "onFrame",
            "([B[BFFFFIIII)V");

    if (g_callback_method == nullptr) {
        LOGE("Failed to find onFrame method");
        env->DeleteGlobalRef(g_callback_obj);
        g_callback_obj = nullptr;
        return JNI_FALSE;
    }

    // Reset frame counter
    g_frame_count = 0;

    // Configure stream: YUYV @ 25fps at the device's frame height
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res = uvc_get_stream_ctrl_format_size(
            g_devh, &ctrl,
            UVC_FRAME_FORMAT_YUYV,
            FRAME_WIDTH, g_geometry.frameHeight, 25);

    if (res < 0) {
        LOGE("uvc_get_stream_ctrl_format_size failed: %s", uvc_strerror(res));
        res = getStreamCtrlForAnyFrameRate(
                g_devh, &ctrl, FRAME_WIDTH, g_geometry.frameHeight);
        if (res < 0) {
            LOGE("No supported frame rate found: %s", uvc_strerror(res));
            env->DeleteGlobalRef(g_callback_obj);
            g_callback_obj = nullptr;
            return JNI_FALSE;
        }
    }

    LOGI("Stream ctrl: format=%d, frame=%d, interval=%u",
         ctrl.bFormatIndex, ctrl.bFrameIndex, ctrl.dwFrameInterval);

    // Start streaming
    res = uvc_start_streaming(g_devh, &ctrl, frameCallback, nullptr, 0);
    if (res < 0) {
        LOGE("uvc_start_streaming failed: %s", uvc_strerror(res));
        env->DeleteGlobalRef(g_callback_obj);
        g_callback_obj = nullptr;
        return JNI_FALSE;
    }

    g_streaming = true;
    LOGI("Streaming started");
    return JNI_TRUE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_breyt_thermalcamera_ThermalCamera_nativeStopStream(
        JNIEnv *env,
        jobject /* this */) {

    std::lock_guard<std::mutex> lock(g_mutex);

    if (!g_streaming || g_devh == nullptr) {
        return;
    }

    uvc_stop_streaming(g_devh);
    g_streaming = false;

    if (g_callback_obj) {
        env->DeleteGlobalRef(g_callback_obj);
        g_callback_obj = nullptr;
    }
    g_callback_method = nullptr;

    // Reset hysteresis state
    hasInitialized = false;
    prevMinRow = prevMinCol = prevMaxRow = prevMaxCol = 0;

    LOGI("Streaming stopped");
}

extern "C" JNIEXPORT void JNICALL
Java_com_breyt_thermalcamera_ThermalCamera_nativeClose(
        JNIEnv *env,
        jobject /* this */) {

    std::lock_guard<std::mutex> lock(g_mutex);

    if (g_streaming) {
        uvc_stop_streaming(g_devh);
        g_streaming = false;
    }

    if (g_callback_obj) {
        env->DeleteGlobalRef(g_callback_obj);
        g_callback_obj = nullptr;
    }
    g_callback_method = nullptr;

    if (g_devh) {
        uvc_close(g_devh);
        g_devh = nullptr;
    }

    if (g_ctx) {
        uvc_exit(g_ctx);
        g_ctx = nullptr;
    }

    // Reset hysteresis and frame state
    hasInitialized = false;
    prevMinRow = prevMinCol = prevMaxRow = prevMaxCol = 0;
    g_frame_count = 0;

    LOGI("Camera closed");
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_breyt_thermalcamera_ThermalCamera_nativeIsStreaming(
        JNIEnv *env,
        jobject /* this */) {
    return g_streaming ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jint JNICALL
Java_com_breyt_thermalcamera_ThermalCamera_getFrameWidth(
        JNIEnv *env,
        jobject /* this */) {
    return FRAME_WIDTH;
}

extern "C" JNIEXPORT jint JNICALL
Java_com_breyt_thermalcamera_ThermalCamera_getImageHeight(
        JNIEnv *env,
        jobject /* this */) {
    return IMAGE_HEIGHT;
}

extern "C" JNIEXPORT void JNICALL
Java_com_breyt_thermalcamera_ThermalCamera_nativeSetRoundingMode(
        JNIEnv *env,
        jobject /* this */,
        jint mode) {
    g_rounding_mode = mode;
    LOGI("Rounding mode set to %d", mode);
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_breyt_thermalcamera_ThermalCamera_nativeGetFormatInfo(
        JNIEnv *env,
        jobject /* this */) {

    if (g_devh == nullptr) {
        return env->NewStringUTF("Camera not open");
    }

    std::string info;

    // Get device descriptor for camera name
    uvc_device_t *dev = uvc_get_device(g_devh);
    if (dev) {
        uvc_device_descriptor_t *desc;
        if (uvc_get_device_descriptor(dev, &desc) == UVC_SUCCESS) {
            if (desc->product) {
                info += "Device: ";
                info += desc->product;
                info += "\n";
            }
            if (desc->manufacturer) {
                info += "Manufacturer: ";
                info += desc->manufacturer;
                info += "\n";
            }
            char vidpid[32];
            snprintf(vidpid, sizeof(vidpid), "VID: %04x, PID: %04x\n", desc->idVendor, desc->idProduct);
            info += vidpid;
            uvc_free_device_descriptor(desc);
        }
    }

    info += "\nSupported formats:\n";

    // Iterate through format descriptors
    const uvc_format_desc_t *format_desc = uvc_get_format_descs(g_devh);
    while (format_desc != nullptr) {
        // Format type
        const char *format_name = "Unknown";
        switch (format_desc->bDescriptorSubtype) {
            case UVC_VS_FORMAT_UNCOMPRESSED:
                // Check GUID for specific format
                if (format_desc->guidFormat[0] == 'Y' && format_desc->guidFormat[1] == 'U' &&
                    format_desc->guidFormat[2] == 'Y' && format_desc->guidFormat[3] == '2') {
                    format_name = "YUY2 (YUYV)";
                } else if (format_desc->guidFormat[0] == 'N' && format_desc->guidFormat[1] == 'V' &&
                           format_desc->guidFormat[2] == '1' && format_desc->guidFormat[3] == '2') {
                    format_name = "NV12";
                } else {
                    format_name = "Uncompressed";
                }
                break;
            case UVC_VS_FORMAT_MJPEG:
                format_name = "MJPEG";
                break;
            case UVC_VS_FORMAT_FRAME_BASED:
                format_name = "Frame-based";
                break;
            default:
                break;
        }

        info += "  ";
        info += format_name;
        info += ":\n";

        // Iterate through frame descriptors for this format
        const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
        while (frame_desc != nullptr) {
            char resolution[64];
            // Calculate fps from interval (100ns units)
            float fps = frame_desc->dwDefaultFrameInterval > 0
                        ? 10000000.0f / frame_desc->dwDefaultFrameInterval
                        : 0;
            snprintf(resolution, sizeof(resolution), "    %dx%d @ %.0f fps\n",
                     frame_desc->wWidth, frame_desc->wHeight, fps);
            info += resolution;
            frame_desc = frame_desc->next;
        }

        format_desc = format_desc->next;
    }

    return env->NewStringUTF(info.c_str());
}
