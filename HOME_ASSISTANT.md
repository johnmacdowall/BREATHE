# BREATHE and Home Assistant

Firmware 4.5.0 adds MQTT discovery to the existing Arduino/PlatformIO build.
Firmware 4.5.2 retains the same 92 entities and connection settings while adding
the redesigned Environment gauges and local 24-hour pressure histogram.
The display, sensor acquisition and button controls continue to work independently.
Home Assistant records incoming readings and can use them in dashboards and
automations. Existing on-device history buffers are not backfilled into HA.

## Setup

1. Pull the complete BREATHE repository in VS Code. This update includes
   `include/home_assistant.h` as well as `src/main.cpp`.
2. Open your existing **local** `include/secrets.h`. Keep the working Wi-Fi
   settings and append the following block, filling in your broker's details:

   ```cpp
   #define BREATHE_MQTT_HOST "YOUR_BROKER_LAN_ADDRESS"
   #define BREATHE_MQTT_PORT 1883
   #define BREATHE_MQTT_USERNAME "YOUR_MQTT_USERNAME"
   #define BREATHE_MQTT_PASSWORD "YOUR_MQTT_PASSWORD"
   #define BREATHE_MQTT_TOPIC_PREFIX "air"
   ```

   Use a LAN IP or resolvable DNS name, without a URL scheme or path. MQTT uses
   standard local TCP on port 1883. An empty host leaves MQTT disabled, so older
   Wi-Fi-only `secrets.h` files continue to compile. The example file is a
   template: editing it alone does not configure the firmware. `secrets.h` stays
   out of Git and is not changed by Pull.
3. Confirm Home Assistant's MQTT integration connects to the same broker and
   MQTT discovery is enabled. The device account needs permission to publish
   `air/#`, publish its discovery entries under `homeassistant/sensor/#` and
   `homeassistant/binary_sensor/#`, and subscribe to `homeassistant/status`.
4. Build and upload using PlatformIO. Open the serial monitor at 115200 baud if
   needed. Expected messages include `MQTT: starting ...` and `MQTT: connected.`
5. In Home Assistant, open **Settings > Devices & services > MQTT** and find
   **BREATHE**. Allow about 30 seconds for discovery and the first readings.

There is no YAML sensor list to paste into Home Assistant. The firmware registers
92 entities: 20 main readings and 72 diagnostics, with stable identifiers based
on the ESP32's Wi-Fi MAC address. Additional BREATHE boards get distinct IDs.
You can rename the device or entities from Home Assistant.

## Data and source meanings

| Group | Exposed entities |
| --- | --- |
| Environment | Si7021 temperature, averaged Si7021/BME680 RH, station pressure in inHg, pressure direction |
| CO2 | SCD41 CO2 ppm |
| Particles | Live-estimate AQI, category, dominant channel, PM1/PM2.5/PM10 mass, all six cumulative size counts |
| Gas | Bosch IAQ, Bosch VOC equivalent |
| Light | Ambient illuminance |
| Raw probes | Separate Si7021, BME680 and SCD41 temperature/RH; BME pressure, dew point and absolute humidity |
| Detailed diagnostics | PM2.5/PM10 subindices, gas resistance/filter/baseline/ratio/statistics, calibration, heater state, sample ages, validity flags, sample/error counts, UART byte counts |
| Instrument diagnostics | Backlight target, display timing, current screen, firmware, uptime, Wi-Fi signal/dropouts, heap usage |

Particle AQI remains a **live estimate**, without daily or NowCast averaging.
Bosch IAQ and VOC equivalent are separate Bosch outputs; the VOC equivalent is
not a measurement of individual gases. Particle size counts are cumulative per
0.1 liter and must not be summed together. Their measurement state class reflects
concentration, not a lifetime accumulating count.

Temperature and humidity retain their measurement precision in telemetry while
suggesting whole-number display precision in HA. Primary pressure is published
in inHg; Home Assistant may apply a user's preferred unit, which can be changed
in that entity's settings. Raw probe entities include the sensor name so they
cannot be mistaken for the Environment reading. BME moisture calculations use
the BME's paired raw temperature and humidity, as on its existing info screens.

The pressure trend is exactly the fitted trend used by the display's small arrow.
It needs at
least two continuous minutes of valid history and uses up to five minutes.
Insufficient or stale history makes the trend unavailable, even while the current
pressure reading remains available.
The screen's 24-hour histogram is separate RAM history built from half-hour mean
pressures. It clears on reboot and does not change HA's own recorded history.

## Topics and recovery

For an example device MAC of `00:01:02:03:04:05`:

| Purpose | Topic |
| --- | --- |
| Full JSON snapshot | `air/breathe_000102030405/state` |
| Online/offline state | `air/breathe_000102030405/availability` |
| Example discovery entry | `homeassistant/sensor/breathe_000102030405/temperature/config` |
| HA restart notification | `homeassistant/status` |

`air` is the telemetry prefix. It does **not** replace Home Assistant's discovery
prefix. If your HA installation uses custom discovery or birth topics, define
`BREATHE_HA_DISCOVERY_PREFIX` and `BREATHE_HA_STATUS_TOPIC` in `secrets.h` too.
Topic prefixes are limited to 23 characters to fit the fixed topic buffers.

The state topic contains the same complete JSON snapshot as
`http://<BREATHE-IP>/status`, including additional engineering metadata not given
separate HA entities. Discovery and availability messages are retained; readings
are non-retained and published every ten seconds. Numeric missing data stays
`null`; it never becomes a fabricated zero. Invalid counts are unavailable even
when PM mass remains valid. Calibration-gated Bosch values remain unavailable
until their existing readiness conditions are satisfied.

Each entity requires both a live device and a non-null reading. A valid zero or
false remains available. If telemetry stops, entities expire after 35 seconds.
The broker also receives a retained `offline` Last Will on an unexpected device
disconnect. A reconnect or HA's `online` birth message triggers rediscovery and
a fresh snapshot. The firmware queues at most one outbound message at a time,
so an unavailable broker cannot accumulate a growing measurement backlog.

Network I/O runs in the ESP-IDF MQTT background task. The main loop snapshots
sensor state and queues messages; MQTT callbacks only update atomic flags and
subscribe to HA's birth topic. MQTT does not control the instrument or its sensors.

## Troubleshooting

- **MQTT disabled:** set the host in `include/secrets.h`, then rebuild/upload.
- **No connection:** check the device's Wi-Fi, broker address, account, port and
  network access. If LAN DNS does not resolve, use the broker's reserved LAN IP.
- **Connected but no BREATHE device:** check discovery is enabled and the broker
  account has the topic permissions listed above.
- **Some readings unavailable:** check sensor freshness and Bosch calibration
  diagnostics. Pressure direction also needs enough history.
- **Inspect the feed:** listen to `air/#` in HA's MQTT integration, or open the
  existing local `/status` page. The `mqtt` object reports configuration state,
  connection count, errors and queued report/discovery counts without credentials.

Validated with an ESP32-C3 PlatformIO build and host checks of discovery JSON,
templates, source values, missing readings, reconnects, birth messages and bounded
queues. Final Wi-Fi/Mosquitto connectivity is checked on the physical device.

References: [Home Assistant MQTT](https://www.home-assistant.io/integrations/mqtt/),
[MQTT sensors](https://www.home-assistant.io/integrations/sensor.mqtt/),
[MQTT binary sensors](https://www.home-assistant.io/integrations/binary_sensor.mqtt/),
and [ESP-IDF MQTT client](https://docs.espressif.com/projects/esp-idf/en/v4.4.8/esp32c3/api-reference/protocols/mqtt.html).
