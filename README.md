# Thermal Camera for Android

Android app for USB thermal cameras (Infiray P2 Pro, Thermal Master P2, Topdon TC001, and compatible devices).

## Features

- Real-time thermal imaging via USB (UVC protocol)
- Temperature overlays: center, min/max markers, tap-to-measure
- Temperature scale with optional lock: freeze the color range to a fixed min/max so colors stay stable as the scene changes; adjust the range manually via the menu
- Multiple colormaps: Turbo, Inferno, Ironbow, Grayscale
- Adjustable contrast
- Rotation sync with device orientation (long-press rotate to toggle lock)
- Mirror mode for selfie use
- Pinch-to-zoom and pan, double-tap to reset
- Temperature rounding (none, 0.2°C, 0.5°C)
- Screenshot capture to gallery

## Supported Devices

Tested: **Infiray P2 Pro** and **Thermal Master P2**. Other Infiray/Topdon USB-C cameras likely compatible.

Open an issue with vendor ID, product ID, and resolution for unsupported devices.

## Requirements

- Android 7.0+ (API 24)
- USB OTG support

## Building

Requires Android Studio, SDK 35, NDK 27.3.13750724, CMake.

```bash
./gradlew assembleDebug
./gradlew installDebug
```

## Controls

| Control | Action |
|---------|--------|
| **Colormap** button | Cycle colormaps |
| **Mirror** button | Toggle horizontal flip |
| **Rotate** button | Rotate 90° clockwise |
| **Rotate** long-press | Toggle rotation lock (sync with device) |
| **⋮** menu | Contrast, rounding, scale lock, camera info |
| Pinch | Zoom (1x–5x) |
| Drag | Pan when zoomed |
| Double-tap | Reset zoom |
| Single-tap | Show temperature at point |

## Dependencies

- [libusb](https://github.com/libusb/libusb)
- [libuvc](https://github.com/libuvc/libuvc)
