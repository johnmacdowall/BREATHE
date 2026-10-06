# BREATHE

Air monitor firmware for John's Seeed XIAO ESP32-C3 and 240 x 240 ST7789 display.
Current firmware: Instrument OS **4.4.3**, with the reviewed screen changes from October 6, 2026.
The gas main screen is titled **GAS AIR QUALITY**, without the Bosch/VOC subtitle
or calibration status. Calibration details remain on `gas-info6`.
Faint cyan gas ribbons drift around the main number on a 14-second cycle. They are
decorative, leave the number and scale clear, and become dim and stationary while
the reading is unavailable. The original long-press chamber animation is retained.
Environment temperature and its history now use the SCD41.
The CO2 eye is smaller with space above its caption; edge registration marks have
been removed from all screens. The subtle eye animation is retained.

## Open in Visual Studio Code

1. Install Git for Windows if needed: <https://git-scm.com/downloads/win>.
2. In VS Code, press **Ctrl+Shift+P**, run **Git: Clone**, and paste
   `https://github.com/johnmacdowall/BREATHE.git`.
3. Sign in to GitHub as `johnmacdowall` if prompted. Choose a parent folder and open
   the cloned BREATHE folder. Use a new folder; keep the old working project as a backup.
4. Install the recommended **PlatformIO IDE** extension if it is not already installed.
   Allow PlatformIO to install the dependencies pinned in `platformio.ini`.
5. In the `include` folder, copy `secrets.example.h` to `secrets.h`. Enter your Wi-Fi
   name and password in **secrets.h only**. Git ignores that file. Without it, the
   sensors and screen work, but Wi-Fi stays off. Keep the example file unchanged.
6. Build using PlatformIO's checkmark. Connect the XIAO by USB and use PlatformIO's
   Upload arrow. Use the serial monitor at **115200 baud** when needed.

The source file is `src/main.cpp`. This is a complete PlatformIO project; open its
root folder containing `platformio.ini` when building.

## Sharing changes

- Before editing or building a new revision, use **Source Control > ... > Pull**.
- After changing code locally, save it, stage the intended files, enter a short
  commit message, **Commit**, then **Push**. This makes your edits available here.
- Commit or stash unfinished local changes before pulling incoming changes.
- Ask for firmware changes in the chat and refer to `johnmacdowall/BREATHE`.
  Updates can be submitted to this repository, then pulled into your local project.
- Saving a file alone does not upload it to GitHub. Build outputs and Wi-Fi secrets
  stay local; `.gitignore` excludes them.

## Controls and screens

| Gesture | Action |
| --- | --- |
| Short press | Next main screen, or next information page within the current group |
| Long press (700 ms) | Open that sensor's information pages; return to its main screen |
| Triple-click | Enter Service; triple-click again or long-press to return to the previous screen |

Clicks are grouped using a 350 ms interval so a triple-click does not first advance
through the main screens. Two quick clicks advance two pages after the interval.
Service pages are excluded from the normal main and sensor-information cycles.

| Group | Main screen | Information pages, in order |
| --- | --- | --- |
| `co2` | Eye with SCD41 ppm | 1: readings; 2: history |
| `pms` | Total particles above 0.3 um per 0.1 L | 1: combined mass/counts/size bands; 2: PM2.5 history |
| `gas` | Gas Air Quality (Bosch IAQ) | 1: original chamber/heater animation; 2: resistance graph; 3: raw/filter/baseline/ratio/deviation; 4: heater status; 5: statistics; 6: IAQ diagnostics; 7: IAQ history |
| `env` | SCD41 temperature, BME680 RH and pressure in inHg | 1: raw BME readings; 2: BME dew point; 3: BME absolute humidity; 4: pressure units; 5: SCD41 temperature history; 6: RH history; 7: pressure history |
| `sys` | Triple-click access only | 1: sensor coverage; 2: timing; 3: health; 4: configuration; 5: light/backlight |

IDs use `co2-main`, `co2-info1`, etc. Main-screen order is Particle Count, CO2, Gas Air Quality,
Environment. There are **27 screens**. Information pages have been renumbered after
the removals and consolidation.

Environment's temperature number, Fahrenheit scale bar, and temperature history
use SCD41 readings with their own freshness check. Missing or stale SCD41 data
shows `--`; there is no automatic fallback to Bosch temperature. Environment RH
and pressure still use the BME680. Raw BME temperature and its paired-temperature/RH
moisture calculations remain available on the explicitly labeled BME information
pages. SCD41 temperature is displayed only in the Environment group.
Sensor calibration offsets have not been changed.

The Environment pressure bar retains the current-pressure marker. A fading cyan
trail and an arrow indicate rising or falling pressure; a dash means steady.
A question mark means insufficient recent history. The fit uses up to five minutes
of readings and needs at least two continuous minutes. Pressure is station pressure,
without sea-level correction.

The Gas Air Quality screen uses Bosch's gas-based **IAQ** index. BSEC2 calculations are enabled;
the current number is shown only after the existing run-in and calibration gates.
The original gas chamber/heater animation remains unchanged. Calibration state is
saved in NVS; screen histories are RAM-only.

## Hardware and build

| Connection | XIAO pin / GPIO |
| --- | --- |
| BME680, SCD41, VEML7700 shared I2C SDA | D4 / GPIO6 |
| Shared I2C SCL | D5 / GPIO7 |
| ST7789 SCK | D8 / GPIO8 |
| ST7789 MOSI | D10 / GPIO10 |
| ST7789 RES | D2 / GPIO4 |
| ST7789 DC | D3 / GPIO5 |
| ST7789 backlight PWM | D0 / GPIO2 |
| Button to ground (internal pull-up) | D1 / GPIO3 |
| PMS UART receive | D7 / GPIO20 |
| PMS UART transmit (no commands currently sent) | D6 / GPIO21 |

The PMS connection names in `src/main.cpp` preserve the existing working PIM477
wiring. Sensor acquisition, light control, and display pin assignments are retained
from the current firmware. BSEC owns the BME680 heater and measurement timing.

The target is `seeed_xiao_esp32c3`, Arduino framework, Espressif32 platform 7.1.3.
Library versions and the Bosch BSEC2 commit are pinned in `platformio.ini`.
PlatformIO downloads these dependencies; third-party libraries and compiled firmware
are not included in this repository.

From a PlatformIO terminal:

```sh
pio run
pio run --target upload
pio device monitor --baud 115200
```

Firmware has been compiled for the target. Uploading and testing this revision on
the physical BREATHE device remain local steps.

Official references: [VS Code GitHub workflow](https://code.visualstudio.com/docs/sourcecontrol/github)
and [PlatformIO for VS Code](https://docs.platformio.org/en/latest/integration/ide/vscode.html).
