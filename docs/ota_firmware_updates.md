# Matter Over-The-Air (OTA) Firmware Updates

This document explains how to configure, build, and deploy Over-The-Air (OTA) firmware updates for the ESP32 Matter sensor using Home Assistant or Espressif's cloud/portal tools.

---

## 1. Enable Matter OTA in `sdkconfig` (Required on Base Firmware)

The project's `partitions.csv` already defines dual OTA slots (`ota_0` and `ota_1`, 1.875 MB each) and `otadata`. However, the firmware currently running on the ESP32 **must** have the Matter OTA Requestor enabled to accept updates over Matter.

1. Enable these options via `idf.py menuconfig` or by adding them to `sdkconfig.defaults`:

   ```ini
   CONFIG_ENABLE_OTA_REQUESTOR=y
   CONFIG_CHIP_OTA_IMAGE_BUILD=y
   ```

   - `CONFIG_ENABLE_OTA_REQUESTOR=y`: Initializes the Matter OTA Software Update Requestor cluster and endpoints inside `esp_matter::start()`.
   - `CONFIG_CHIP_OTA_IMAGE_BUILD=y`: Automatically wraps the compiled `.bin` binary with a Matter OTA header to produce a `.ota` file during `idf.py build`. Note, if OTA image with this version exists, it will not be re-created, either remove it, or build manually
   (see below).
2. Build and flash this initial firmware (`v1`, `PROJECT_VER_NUMBER 1` in `CMakeLists.txt`) over USB once.

---

## 2. Build a New OTA Firmware Image (`v2`)

Matter requires the new image's software version number to be **strictly higher** than the currently running version, and the Vendor ID (`0xFFF1`) and Product ID (`0x8000`) to match.

1. Increment the version in `CMakeLists.txt`:

   ```cmake
   set(PROJECT_VER "2.0")
   set(PROJECT_VER_NUMBER 2)
   ```

   *(If `CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER` is set in `sdkconfig` under `Component config → CHIP Device Layer → Device Identification Options`, increment it there as well).*
2. Build the project:

   ```bash
   idf.py build
   ```

The build outputs **`build/matter_sensor-ota.bin`**.
3. To see information about the ota image run:

```bash
$ESP_MATTER_PATH/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py show build/matter_sensor-ota.bin
```

You will see the header information, including the Vendor ID, Product ID, and the software version number, confirming that the image is ready for distribution.
4. [Optional] You can also generate it manually from `build/matter_sensor.bin` using CHIP's tool, but if OTA is   configured, `idf.py build` should generate it.:

   ```bash
   $ESP_MATTER_PATH/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py create \
     -v 0xFFF1 -p 0x8000 -vn 2 -vs "2.0" -da sha256 \
     build/matter_sensor.bin build/matter_sensor-ota.bin
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
