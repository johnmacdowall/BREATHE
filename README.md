# BREATHE

Air monitor firmware for John's Seeed XIAO ESP32-C3 and 240 x 240 ST7789 display.
Current firmware: Instrument OS **4.4.8**, with the reviewed screen changes from October 6, 2026.
Temperature and relative humidity display as whole numbers, including their history
labels; sensor values and calculations keep full precision. Environment's pressure
label is **BARO**. Text colors are 30% brighter again after the previous 20% increase
(1.20 x 1.30 = 1.56 times the original channels), capped at full intensity.
The gas main screen is titled **VoC AIR QUALITY**, without the Bosch/VOC subtitle
or calibration status. Calibration details remain on `gas-info6`.
Cyan, amber and violet gas ribbons are another 30% brighter than v4.4.5 and drift around the
main number on a 14-second cycle. The colors suggest a mixed VOC response; they do
not represent individually measured gases. The ribbons are decorative, leave the
number and scale clear, and become dim and stationary while
the reading is unavailable. The original long-press chamber animation is retained.
Environment temperature and its history use the **Si7021**. Environment RH and
its history show the arithmetic mean of **Si7021 RH and BME680 RH**. The new
`env-info8` page shows the source readings and average; Service coverage includes Si7021.
The CO2 eye is moved up four pixels to center it between its title and lower caption.
CO2, Particle AQI, particle count and VoC numbers share a 33-pixel-high font; CO2 uses a
compact fallback for five-digit readings. The subtle eye animation is retained.
Environment uses one consistent sans-serif font family, with a clear title,
aligned labels and reading columns. **F**, **%** and **inHg** use the same size,
font and baseline as their numbers. The three-row layout, all readings, and the
bottom bars with their pressure trend are preserved. The embedded Environment
fonts are covered by [FONT-LICENSE.txt](FONT-LICENSE.txt).
Edge registration marks remain removed.

**PARTICLE AQI** is now the main radar screen. It shows a **LIVE ESTIMATE** from
PMS5003 mass readings, a category label, and a six-color EPA scale from 0 to 500.
The radar remains visible around the number; only the text itself is masked.
The previous **TOTAL >0.3 um** count radar is now `pms-info3`.
`pms-info1`, titled **PMS // COUNTS**, retains the large two-column, three-row
layout for the six cumulative size thresholds. `pms-info2` retains PM2.5 history.

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
| `pms` | Particle AQI live estimate with radar and 0-500 scale | 1: six large cumulative size counts; 2: PM2.5 history; 3: total particles above 0.3 um per 0.1 L with radar |
| `gas` | VoC Air Quality (Bosch IAQ) | 1: original chamber/heater animation; 2: resistance graph; 3: raw/filter/baseline/ratio/deviation; 4: heater status; 5: statistics; 6: IAQ diagnostics; 7: IAQ history |
| `env` | Si7021 temperature, averaged Si7021/BME680 RH, BME680 pressure in inHg | 1: raw BME readings; 2: BME dew point; 3: BME absolute humidity; 4: pressure units; 5: Si7021 temperature history; 6: averaged RH history; 7: pressure history; 8: temperature/RH source comparison |
| `sys` | Triple-click access only | 1: sensor coverage; 2: timing; 3: health; 4: configuration; 5: light/backlight |

IDs use `co2-main`, `co2-info1`, etc. Main-screen order is Particle AQI, CO2, VoC Air Quality,
Environment. There are **29 screens**. Information pages have been renumbered after
the removals and consolidation.

Environment's temperature number, Fahrenheit scale bar, and temperature history
use only the Si7021. RH, its scale bar and history use `(Si7021 RH + BME680 RH) / 2`,
calculated before display rounding. Both humidity readings must be fresh (under
six seconds old). A missing, failed or stale Si7021 shows `--` for temperature and
averaged RH; missing/stale BME data shows `--` for averaged RH and pressure.
There is no automatic fallback to another temperature probe or a single RH source.

The Si7021 is read about every two seconds using nonblocking no-hold conversions,
CRC validation for both measurements, and automatic retries after communication
failure. Its heater stays off; sensor values and history retain full precision.
`env-info8` shows Si temperature, Si RH, BME RH, averaged RH, Si sample age and errors.
The local `/status` JSON exposes the same sources and sensor health separately.
Raw BME temperature and its paired-temperature/RH moisture calculations remain on
the explicitly labeled BME information pages. SCD41 continues to supply CO2.
Sensor calibration offsets have not been changed.

The Environment pressure bar retains the current-pressure marker. A fading cyan
trail and an arrow indicate rising or falling pressure; a dash means steady.
A question mark means insufficient recent history. The fit uses up to five minutes
of readings and needs at least two continuous minutes. Pressure is station pressure,
without sea-level correction.

The PMS5003 count above 0.3 micrometers is already the cumulative total above
its smallest reported size threshold, per 0.1 liter of air. Each of the other
thresholds (0.5, 1.0, 2.5, 5.0 and 10 micrometers) is a subset of that total;
adding all six would count some particles repeatedly. The six info-page readings
retain the sensor's cumulative meaning rather than representing disjoint bins.

The Particle AQI live estimate uses the PMS5003's atmospheric PM2.5 and PM10
**mass concentrations** in micrograms per cubic meter, not summed particle counts
or the sensor's CF=1 fields. Each mass reading is converted with the EPA May 2026
breakpoint table, rounded to the nearest integer, and the higher subindex is
displayed. PM2.5 uses tenths and PM10 whole micrograms per cubic meter; the sensor
itself reports whole mass values. Above 500, the highest band is extrapolated:
the actual number remains visible, the marker stops at 500, and **ABOVE SCALE**
appears. Missing, stale or warming-up mass data shows `--`.

This calculation uses the current sample without daily or NowCast averaging, so
the main screen explicitly says **LIVE ESTIMATE**. It is not an official daily
AQI or NowCast. The status JSON exposes both subindices, the dominant mass
channel, category, validity and averaging method under `pms.aqi_live_estimate`.
The VoC index remains separate Bosch IAQ and does not incorporate PMS readings.
PM2.5 mass remains visible on `pms-info2`; all three mass channels and all six
cumulative counts remain in the status JSON. Raw total counts are on `pms-info3`.
References: [Plantower PMS5003 protocol](https://cdn-shop.adafruit.com/product-files/3686/plantower-pms5003-manual_v2-3.pdf)
and [EPA AQI technical guidance](https://document.airnow.gov/technical-assistance-document-for-the-reporting-of-daily-air-quailty.pdf).

The VoC Air Quality screen uses Bosch's gas-based **IAQ** index. BSEC2 calculations are enabled;
the current number is shown only after the existing run-in and calibration gates.
The original gas chamber/heater animation remains unchanged. Calibration state is
saved in NVS; screen histories are RAM-only.

## Hardware and build

| Connection | XIAO pin / GPIO |
| --- | --- |
| BME680, SCD41, Si7021, VEML7700 shared I2C SDA | D4 / GPIO6 |
| Shared I2C SCL | D5 / GPIO7 |
| ST7789 SCK | D8 / GPIO8 |
| ST7789 MOSI | D10 / GPIO10 |
| ST7789 RES | D2 / GPIO4 |
| ST7789 DC | D3 / GPIO5 |
| ST7789 backlight PWM | D0 / GPIO2 |
| Button to ground (internal pull-up) | D1 / GPIO3 |
| PMS UART receive | D7 / GPIO20 |
| PMS UART transmit (no commands currently sent) | D6 / GPIO21 |

Connect Si7021 **VIN to the 3.3V bus**, **GND to GND**, **SDA to D4/GPIO6**, and
**SCL to D5/GPIO7**. Leave the breakout's **3Vo output unconnected**. Its I2C address is **0x40**. Place it away from the Bosch
heater, display and processor heat when evaluating room temperature.

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
