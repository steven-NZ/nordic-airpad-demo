# Build Instructions

## Prerequisites

1. **VS Code** with **nRF Connect for VS Code** extension installed
   - Install from VS Code Extensions marketplace
2. **Hardware:** nRF52840 DK + USB cable

---

## Setup

1. **Open Project**
   - Launch VS Code
   - Open the `nordic-airpad-demo` folder

2. **Add Build Configuration**
   - Open nRF Connect extension panel (sidebar)
   - Click "Add Build Configuration"
   - Select board: `nrf52840dk/nrf52840`
   - Configuration will auto-generate

---

## Build & Flash

Use the **nRF Connect: Actions** panel in VS Code:

1. **Build**
   - Click "Build" in the Actions panel
   - Wait for compilation to complete
   - **Tip:** Hover over "Build" to find "Pristine Build" (clean rebuild)

2. **Flash**
   - Connect nRF52840 DK via USB
   - Click "Flash" in the Actions panel
   - Firmware will be programmed to the device

---

## View Console Logs

1. Navigate to **Connected Devices** in nRF Connect panel
2. Click on your nRF52840 DK device
3. Select **VCOM0**
4. Click **Open Terminal**

The console will display log output from the running firmware.
