#pragma once
// Copy this file to secrets.h in the same folder, then enter your Wi-Fi settings.
// secrets.h is excluded from Git. Do not put real credentials in this example.
constexpr char WIFI_SSID[] = "YOUR_WIFI_NAME";
constexpr char WIFI_PASSWORD[] = "PUT_YOUR_WIFI_PASSWORD_HERE";

// Home Assistant / Mosquitto. Append this block to an existing secrets.h;
// keep its working Wi-Fi settings. Empty HOST leaves MQTT disabled.
// Use the broker's LAN IP or DNS name, without http:// or mqtt://.
#define BREATHE_MQTT_HOST ""
#define BREATHE_MQTT_PORT 1883
#define BREATHE_MQTT_USERNAME ""
#define BREATHE_MQTT_PASSWORD ""
#define BREATHE_MQTT_TOPIC_PREFIX "air"
// Change only if Home Assistant uses custom discovery/birth topics.
#define BREATHE_HA_DISCOVERY_PREFIX "homeassistant"
#define BREATHE_HA_STATUS_TOPIC "homeassistant/status"
