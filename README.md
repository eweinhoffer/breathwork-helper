# Breathwork Helper (ESP32-S3 Reverse TFT Feather)

This project turns the Adafruit ESP32-S3 Reverse TFT Feather into a guided breathwork display with locked HTTP OTA updates over local Wi-Fi.

## Project layout
- Sketch folder: `breathwork_helper/`
- Sketch file: `breathwork_helper/breathwork_helper.ino`
- Partitions: `breathwork_helper/partitions.csv`
- Secrets template: `breathwork_helper/secrets.example.h`
- Local secrets: `breathwork_helper/secrets.h` (not committed)

## Buttons
- `D0`: start/pause breathing cycle.
- `D1` short press: cycle breathing mode.
- `D1` hold ~2.2s: arm OTA window.
- `D2`: reset cycle to inhale.

## OTA model
- OTA is locked by default.
- Hold `D1` for ~2.2s to arm OTA for 10 minutes.
- Endpoint: `http://<board_ip>/update`
- Auth from `secrets.h` (`OTA_HTTP_USER`, `OTA_HTTP_PASSWORD`).

## Setup
1. Copy `secrets.example.h` to `secrets.h`.
2. Set Wi-Fi and OTA credentials.
3. Build + flash once over USB.
4. Use OTA for normal updates.

## CLI build
```bash
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
SKETCH="breathwork_helper"
FQBN="esp32:esp32:adafruit_feather_esp32s3_reversetft"

"$CLI" compile --clean --fqbn "$FQBN" --build-path /tmp/breathwork-build --output-dir /tmp/breathwork-out "$SKETCH"
```

## USB flash
```bash
PORT="/dev/cu.usbmodem101"
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
SKETCH="breathwork_helper"
FQBN="esp32:esp32:adafruit_feather_esp32s3_reversetft"

"$CLI" upload -p "$PORT" --fqbn "$FQBN" --input-dir /tmp/breathwork-out "$SKETCH"
```

## OTA flash
1. Hold `D1` for ~2.2s to arm OTA.
2. Run:
```bash
BIN="/tmp/breathwork-out/breathwork_helper.ino.bin"
IP="<board_ip_from_screen>"
USER="ota"
PASS="<ota_password>"

curl --fail -u "$USER:$PASS" "http://$IP/"
curl -v --fail -u "$USER:$PASS" \
  -F "firmware=@$BIN;type=application/octet-stream" \
  "http://$IP/update"
```

## Troubleshooting
- `403 OTA LOCKED`: hold `D1` again.
- `401 Unauthorized`: verify OTA credentials in `secrets.h`.
- `500 OTA FAIL`: reflash once by USB so partition layout is definitely applied.
- If Codex cannot reach LAN IPs, run final OTA `curl` locally on your Mac terminal.
