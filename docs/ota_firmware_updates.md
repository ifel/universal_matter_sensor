# Matter Over-The-Air (OTA) Firmware Updates

This document explains how to configure, build, and deploy Over-The-Air (OTA) firmware updates for the ESP32 Matter sensor using Home Assistant or Espressif's cloud/portal tools.

---

## 1. Matter OTA Configuration in `sdkconfig`

The project's `partitions.csv` defines dual OTA slots (`ota_0` and `ota_1`, 1.875 MB each) and `otadata`, and [`sdkconfig.defaults`](../sdkconfig.defaults) enables Matter OTA by default:

```ini
CONFIG_ENABLE_OTA_REQUESTOR=y
CONFIG_CHIP_OTA_IMAGE_BUILD=y
```

- `CONFIG_ENABLE_OTA_REQUESTOR=y`: Initializes the Matter OTA Software Update Requestor cluster and endpoints inside `esp_matter::start()`.
- `CONFIG_CHIP_OTA_IMAGE_BUILD=y`: Automatically wraps the compiled `.bin` binary with a Matter OTA header to produce `matter_sensor-ota.bin` during `idf.py build`. *(Note: if the `.ota` image already exists in the build directory, remove it before rebuilding or generate it manually below).*

### Separate Product IDs for Thread (`0x8001`) vs Wi-Fi (`0x8000`)

To prevent Home Assistant from accidentally pushing a Wi-Fi OTA image to a Thread device (or vice versa), [`CMakePresets.json`](../CMakePresets.json) assigns a distinct `CONFIG_DEVICE_PRODUCT_ID` to each transport:

| CMake Preset | Transport | Config File | Vendor ID (`VID`) | Product ID (`PID`) | OTA Image Path |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`prod`** | **Matter over Thread** | [`sdkconfig.defaults.prod`](../sdkconfig.defaults.prod) | `0xFFF1` | **`0x8001`** | `build/prod/matter_sensor-ota.bin` |
| **`prod_wifi`** | **Matter over Wi-Fi** | [`sdkconfig.defaults.prod_wifi`](../sdkconfig.defaults.prod_wifi) | `0xFFF1` | **`0x8000`** | `build/prod_wifi/matter_sensor-ota.bin` |
| **`default`** | **Matter over Wi-Fi** *(dev)* | [`sdkconfig.defaults`](../sdkconfig.defaults) | `0xFFF1` | **`0x8000`** | `build/default/matter_sensor-ota.bin` |

---

## 2. Build a New OTA Firmware Image

Matter requires the new image's `PROJECT_VER_NUMBER` to be **strictly higher** than the currently running version, and both the Vendor ID (`0xFFF1`) and Product ID (`0x8001` for Thread or `0x8000` for Wi-Fi) to match the running firmware.

1. Increment the version in [`CMakeLists.txt`](../CMakeLists.txt):

   ```cmake
   set(PROJECT_VER "5.4")
   set(PROJECT_VER_NUMBER 9)
   ```

   *(If `CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER` is overridden in `sdkconfig`, increment it there as well).*
2. Build the desired preset (`prod` for Thread or `prod_wifi` for Wi-Fi):

   ```bash
   idf.py --preset prod build        # outputs build/prod/matter_sensor-ota.bin (PID 0x8001)
   # or:
   idf.py --preset prod_wifi build   # outputs build/prod_wifi/matter_sensor-ota.bin (PID 0x8000)
   ```

3. Inspect the generated OTA image header to verify the Vendor ID, Product ID, and software version number:

   ```bash
   $ESP_MATTER_PATH/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py show build/prod/matter_sensor-ota.bin
   ```

4. *(Optional)* You can also generate the OTA image manually from `matter_sensor.bin` using CHIP's `ota_image_tool.py` (use `-p 0x8001` for Thread or `-p 0x8000` for Wi-Fi):

   ```bash
   $ESP_MATTER_PATH/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py create \
     -v 0xFFF1 -p 0x8001 -vn 9 -vs "5.4" -da sha256 \
     build/prod/matter_sensor.bin build/prod/matter_sensor-ota.bin
   ```

---

## 3. Deploy the OTA Update via Home Assistant (Local Matter OTA Provider)

Home Assistant's Matter Server includes a built-in Matter **OTA Provider** that serves local `.ota` files directly to your devices over Wi-Fi:

1. **Enable the local OTA directory in Matter Server**:
   - In Home Assistant, check **Settings → Add-ons → Matter Server → Configuration** (or pass `--ota-provider-dir /data/updates` if running the Matter Server Docker container directly, rPI based HA pass `--ota-provider-dir /config/updates` automatically).
2. **Copy `build/*.ota` to Home Assistant**:
   - Place `*.ota` into the Matter Server's OTA directory on the Home Assistant host, for rPI installation:

     ```text
     /app_configs/core_matter_server/updates/
     ```

     *(or `/data/updates` inside the container).*
3. **Trigger the OTA Update**:
   - Restart the Matter Server add-on so it indexes the new `.ota` file. If it's reindexed, the file will be removed from the ota-provider-dir.
   - Trigger the update from the device's **Firmware** update entity in Home Assistant (or wait for the periodic OTA check / run `matter ota query` in the ESP32 serial console).
   - The ESP32 downloads the image over Wi-Fi via Matter BDX into the inactive OTA partition, verifies the checksum, and reboots into the new version.

---

## 4. Integrating Portals for Firmware Maintenance & OTA

Depending on how you want to manage firmware releases, there are three portal options you can integrate into your workflow:

### Option A: CSA Distributed Compliance Ledger (DCL) Test-Net Portal (Native Home Assistant Integration)

Home Assistant's Matter Server natively queries the **CSA DCL** (`on.dcl.csa-iot.org` and `on.test-net.dcl.csa-iot.org`) for firmware updates:

- You can host your `.ota` binary on **GitHub Releases** (HTTPS URL) and register the software version metadata (Vendor ID, Product ID, SoftwareVersion, `otaUrl`, `otaFileSize`, and SHA-256 digest) on the DCL test-net portal or via a local DCL/JSON manifest.
- When enabled in Home Assistant Matter Server (`--enable-test-net-dcl`), Home Assistant automatically detects new versions and shows an **"Update Available"** notification in the Home Assistant UI without manually copying `.ota` files.

### Option B: ESP Launchpad + GitHub Releases (Web Portal for Maintaining & Flashing Releases)

If you want a lightweight web portal to maintain versioned firmware releases and generate QR codes without running cloud infrastructure:

- Publish your merged binaries (`idf.py merge-bin`) and `.ota` files to **GitHub Releases**.
- Add an **ESP Launchpad** `config.toml` manifest to your repository.
- You get a web portal at `https://espressif.github.io/esp-launchpad/?flashConfigURL=...` where you can pick any firmware version, flash devices over USB in the browser, and display the Matter commissioning QR code automatically.

### Option C: ESP RainMaker Portal (Espressif Cloud Fleet & OTA Management)

[**ESP RainMaker**](https://rainmaker.espressif.com/) (and **ESP ZeroCode**) is Espressif's cloud portal for managing device fleets, uploading firmware images, and pushing OTA updates from a web dashboard:

- **How it works**: You upload `.bin` firmware releases to the RainMaker web dashboard and click **Push OTA** for a device or group.
- **Tradeoff with Home Assistant**: RainMaker OTA uses Espressif's `esp_rainmaker` cloud agent (MQTT/HTTPS) rather than the local Matter OTA cluster. Adding the RainMaker agent alongside `esp_matter` increases flash/RAM usage and requires an ESP RainMaker cloud account. For a purely local Home Assistant Matter setup, **Home Assistant's built-in Matter OTA Provider (Section 3)** requires zero cloud dependencies and works natively with your current code.
