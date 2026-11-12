# Commissioning to Home Assistant (Matter over Thread & Wi-Fi)

This document describes how to commission the ESP32 Matter sensor into **Home Assistant** over **Thread** (`--preset prod`) or **Wi-Fi** (`--preset prod_wifi` / `--preset default`), along with alternative options using **ESP Launchpad** and **`esp-matter-mfg-tool`** to simplify flashing and QR code generation.

---

## 1. Flash and Retrieve Onboarding Codes

1. Flash the firmware for your chosen preset and open the serial monitor (erase flash first if the board was previously commissioned):

   ```bash
   idf.py --preset prod erase-flash flash monitor        # Matter over Thread (PID 0x8001)
   # or:
   idf.py --preset prod_wifi erase-flash flash monitor   # Matter over Wi-Fi (PID 0x8000)
   ```

2. On boot, the firmware prints the onboarding codes in the serial log (`matter_manager: QR code: …`, `QR code image: …`, `Manual pairing code: …`). With the default test credentials (Passcode `20202021`, Discriminator `3840`, Bluetooth discovery) they are:
   - **Manual Pairing Code** (all presets): `34970112332`
   - **QR Code** `prod` (Thread, PID `0x8001`): `MT:-24J042C00KA0648G00` ([image](https://project-chip.github.io/connectedhomeip/qrcode.html?data=MT%3A-24J042C00KA0648G00))
   - **QR Code** `prod_wifi` / `default` (Wi-Fi, PID `0x8000`): `MT:Y.K9042C00KA0648G00` ([image](https://project-chip.github.io/connectedhomeip/qrcode.html?data=MT%3AY.K9042C00KA0648G00))
   - The codes are derived from the passcode, discriminator, Vendor/Product ID and discovery mode, so they change only if one of those does (e.g. a factory partition from `esp-matter-mfg-tool`, below). To compute them without a device: `$ESP_MATTER_PATH/connectedhomeip/connectedhomeip/src/setup_payload/python/SetupPayload.py generate -d 3840 -p 20202021 -vid 65521 -pid 32769 -dm 2 -cf 0` (decimal IDs; `-dm 2` = BLE).
   - *Tip*: With `CONFIG_ENABLE_CHIP_SHELL=y` enabled, you can type `matter onboardingcodes ble` in the serial monitor at any time to reprint them.

---

## 2. Home Assistant Prerequisites & Trusting Development / Self-Signed Certificates

- The **Matter** integration must be installed in Home Assistant (**Settings → Devices & Services → Add Integration → Matter**).
- Your network must have **2.4 GHz Wi-Fi** and **IPv6** enabled.

### Trusting Self-Signed / Test Attestation Certificates (Required for DIY Devices)

DIY / homemade Matter devices built with ESP-Matter use **test / self-signed Device Attestation Certificates (DACs)** (Test Vendor ID `0xFFF1`) rather than commercial CSA production certificates. Before commissioning, ensure both **Home Assistant** and your **phone (iOS / Android)** are configured to accept test/uncertified Matter devices:

#### A. In Home Assistant (Matter Server Add-on / Container)

1. **Home Assistant OS / Supervised (Matter Server Add-on)**:
   - In Home Assistant, go to **Settings → Add-ons → Matter Server → Configuration**.
   - Ensure **Enable test-net DCL (`enable_test_net_dcl`)** is enabled (enabled by default in recent Matter Server versions so test/development PAA certificates are trusted).
   - If you generate custom self-signed PAA root certificates (outside the standard CHIP test PAA), place your root `.pem`/`.der` certificate into `/share/matter_server/credentials/` (or specify `--paa-root-cert-dir` in the container startup args) and restart the **Matter Server** add-on.

---

## 3. Commissioning Methods

### Option A: Via Home Assistant Companion App (BLE → Wi-Fi)

Even for **Matter over Wi-Fi**, Bluetooth (BLE) is used briefly during initial pairing to transfer your Wi-Fi credentials to the ESP32:

1. Open the **Home Assistant Companion App** on your phone (with Bluetooth enabled and connected to your 2.4 GHz Wi-Fi network).
2. Go to **Settings → Devices & Services → Matter → Add Device**.
3. Scan the QR code from the URL printed in your serial monitor (or choose **Enter code manually** and enter `34970112332`).
4. If prompted about an uncertified device, tap **Add Anyway / Continue**.
5. **Wi-Fi Credentials Transfer**:
   - **iOS (iPhone / iPad)**: Does **not** prompt for a password—iOS automatically shares the SSID and password of the Wi-Fi network your phone is currently connected to (from iOS Keychain). If you have separate 2.4 GHz and 5 GHz SSIDs, ensure your iPhone is connected to the **2.4 GHz SSID** before starting.
   - **Android**: Displays the 2.4 GHz networks scanned by the ESP32 and either auto-fills your saved Wi-Fi password or prompts you to select the 2.4 GHz network and enter its password.
6. Once the phone transfers Wi-Fi credentials to the ESP32 and commissioning completes, the ESP32 automatically turns off BLE (`BLE deinitialized`) and communicates with Home Assistant exclusively over **Wi-Fi**.

### Option B: Pure Wi-Fi Commissioning (No Phone / No Bluetooth)

If you want to commission directly from the Home Assistant web UI without using Bluetooth or a phone:

1. Connect the ESP32 to your 2.4 GHz Wi-Fi via the `idf.py monitor` console:

   ```text
   matter esp wifi connect <YOUR_2.4GHZ_WIFI_SSID> <YOUR_WIFI_PASSWORD>
   ```

2. Wait until the serial log prints `Wi-Fi connectivity change` and `Interface IP address changed`.
3. In the Home Assistant web UI, go to **Settings → Devices & Services → Matter → Configure → Commission a device** (or select **"Yes, it's already in use / on network"** in the mobile app).
4. Enter the manual pairing code: **`34970112332`**.
5. Home Assistant will discover the ESP32 on your Wi-Fi network via mDNS and pair with it directly.

---

## 4. Alternative Options to Simplify Flashing & QR Code Generation

By default, the firmware uses test commissioning credentials (`CONFIG_EXAMPLE_COMMISSIONABLE_DATA_PROVIDER=y`), where every device shares the same passcode (`20202021`) and discriminator (`3840`), and you open the serial monitor URL to view the QR code.

You can use either of the following two options to simplify this workflow:

### Alternative 1: ESP Launchpad (Browser Flashing + Automatic In-Browser QR Code)

[**ESP Launchpad**](https://espressif.github.io/esp-launchpad/) is Espressif's WebSerial portal that lets you maintain firmware images in GitHub Releases, flash any ESP32 directly from Chrome/Edge without ESP-IDF installed, and **automatically display the Matter QR code in the browser** right after flashing.

#### ESP Launchpad Step-by-Step

1. **Build a single merged flash binary**:

   ```bash
   idf.py merge-bin -o matter_sensor_merged.bin
   ```

   This produces `build/matter_sensor_merged.bin` (which combines the bootloader, partition table, and app image starting at offset `0x0`).
2. **Upload `matter_sensor_merged.bin` to a GitHub Release** in your repository (e.g., tag `v1.0.0`).
3. **Create a `launchpad.toml` file** in your repository (for example at `docs/launchpad.toml`):

   ```toml
   esp_toml_version = 1.0
   firmware_images_url = "https://github.com/<your-username>/matter_sensor/releases/download/v1.0.0/"

   supported_apps = ["matter_sensor"]

   [matter_sensor]
   chipsets = ["ESP32-C6", "ESP32", "ESP32-H2"]
   image.esp32c6 = "matter_sensor_merged.bin"
   image.esp32 = "matter_sensor_merged.bin"
   image.esp32h2 = "matter_sensor_merged.bin"
   readme.text = "https://raw.githubusercontent.com/<your-username>/matter_sensor/main/README.md"
   ```

4. **Open ESP Launchpad with your config URL**:

   ```text
   https://espressif.github.io/esp-launchpad/?flashConfigURL=https://raw.githubusercontent.com/<your-username>/matter_sensor/main/docs/launchpad.toml
   ```

5. **Flash & Scan QR Code**:
   - Click **Connect**, select your ESP32 USB serial port, and click **Flash**.
   - Switch to the **Console** tab in ESP Launchpad and click **Reset Device**—ESP Launchpad detects the Matter `MT:...` onboarding payload in the boot log and renders the scannable QR code directly on screen.

---

### Alternative 2: `esp-matter-mfg-tool` (Unique Per-Device QR Code PNGs & Factory Partitions)

If you have multiple sensors on the same network, generating unique factory partitions with `esp-matter-mfg-tool` gives each sensor its own unique Passcode, Discriminator, and a ready-to-print **QR code PNG image** (`<uuid>-qrcode.png`).

#### esp-matter-mfg-tool Step-by-Step

1. **Install `esp-matter-mfg-tool`**:

   ```bash
   pip install esp-matter-mfg-tool
   ```

2. **Generate factory partition(s) and QR code image(s)** (change `-n 1` to generate multiple devices at once):

   ```bash
   esp-matter-mfg-tool -n 1 -v 0xFFF1 -p 0x8000 --pai \
     -k $ESP_MATTER_PATH/connectedhomeip/connectedhomeip/credentials/test/attestation/Chip-Test-PAI-FFF1-8000-Key.pem \
     -c $ESP_MATTER_PATH/connectedhomeip/connectedhomeip/credentials/test/attestation/Chip-Test-PAI-FFF1-8000-Cert.pem \
     -cd $ESP_MATTER_PATH/connectedhomeip/connectedhomeip/credentials/test/certification-declaration/Chip-Test-CD-FFF1-8000.der
   ```

   - **`-v 0xFFF1` (Vendor ID / VID)**: `0xFFF1` is the standard Connectivity Standards Alliance (CSA) **Test Vendor ID** configured in `sdkconfig` (`CONFIG_DEVICE_VENDOR_ID=0xFFF1`).
   - **`-p 0x8000` or `-p 0x8001` (Product ID / PID)**:
     - Use **`-p 0x8000`** (with `Chip-Test-PAI-FFF1-8000-*` and `Chip-Test-CD-FFF1-8000.der`) for **Wi-Fi** builds (`prod_wifi` / `default`, `CONFIG_DEVICE_PRODUCT_ID=0x8000`).
     - Use **`-p 0x8001`** (with `Chip-Test-PAI-FFF1-8001-*` and `Chip-Test-CD-FFF1-8001.der`) for **Thread** builds (`prod`, `CONFIG_DEVICE_PRODUCT_ID=0x8001`).
   - *(Note: `-v` and `-p` are encoded into the Matter QR code and Device Attestation Certificate, so they must match the test PAI certificate and any future `.ota` firmware images).*

   This creates an output folder under `out/fff1_8000/<uuid>/` containing:
   - `<uuid>-partition.bin` — The NVS factory partition image.
   - `<uuid>-qrcode.png` — The scannable Matter QR code image (can be printed or saved).
   - `<uuid>-onb_codes.csv` — Contains the manual pairing code, passcode, discriminator, and QR payload.
3. **Configure the firmware to read onboarding data from the `fctry` partition**:
   Enable the following in `idf.py menuconfig` (or `sdkconfig.defaults`):

   ```ini
   CONFIG_ENABLE_ESP32_FACTORY_DATA_PROVIDER=y
   CONFIG_SEC_CERT_COMMISSIONABLE_DATA_PROVIDER=y
   CONFIG_EXAMPLE_COMMISSIONABLE_DATA_PROVIDER=n
   ```

4. **Flash the factory partition to the `fctry` offset (`0x3E0000` in `partitions.csv`)**:

   ```bash
   esptool.py -p <PORT> write_flash 0x3E0000 out/fff1_8000/<uuid>/<uuid>-partition.bin
   ```

5. **Commission with the generated QR code**:
   Scan `<uuid>-qrcode.png` directly with the Home Assistant Companion App—no serial monitor needed.
