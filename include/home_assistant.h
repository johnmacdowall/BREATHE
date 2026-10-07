#pragma once
// Home Assistant discovery + read-only telemetry. ESP-IDF MQTT handles network
// I/O on its own task. Only the Arduino loop reads sensor/UI state.
#include <mqtt_client.h>
#include <cJSON.h>
#include <atomic>

// Add these #defines to the existing, ignored secrets.h to enable MQTT.
// Older Wi-Fi-only secrets.h files continue to compile with MQTT disabled.
#ifndef BREATHE_MQTT_HOST
#define BREATHE_MQTT_HOST ""
#endif
#ifndef BREATHE_MQTT_PORT
#define BREATHE_MQTT_PORT 1883
#endif
#ifndef BREATHE_MQTT_USERNAME
#define BREATHE_MQTT_USERNAME ""
#endif
#ifndef BREATHE_MQTT_PASSWORD
#define BREATHE_MQTT_PASSWORD ""
#endif
#ifndef BREATHE_MQTT_TOPIC_PREFIX
#define BREATHE_MQTT_TOPIC_PREFIX "air"
#endif
#ifndef BREATHE_HA_DISCOVERY_PREFIX
#define BREATHE_HA_DISCOVERY_PREFIX "homeassistant"
#endif
#ifndef BREATHE_HA_STATUS_TOPIC
#define BREATHE_HA_STATUS_TOPIC "homeassistant/status"
#endif

static_assert(sizeof(BREATHE_MQTT_TOPIC_PREFIX)<=24,"MQTT topic prefix must be at most 23 characters");
constexpr uint32_t MQTT_REPORT_MS=10000, MQTT_EXPIRE_S=35;
struct HAEntity {
 const char* id;const char* name;const char* path;
 const char* unit;const char* deviceClass;const char* stateClass;
 int8_t precision;bool diagnostic;bool binary;
 constexpr HAEntity(const char* i,const char* n,const char* p,const char* u,
  const char* dc,const char* sc,int8_t digits,bool diag,bool bin=false):
  id(i),name(n),path(p),unit(u),deviceClass(dc),stateClass(sc),precision(digits),diagnostic(diag),binary(bin){}
};
// Paths refer to statusJSON(). Keep source names explicit; raw probe readings
// never replace Environment's Si7021 temperature or averaged humidity.
const HAEntity HA_ENTITIES[]={
 {"temperature","Temperature","environment.temperature_F","\xC2\xB0" "F","temperature","measurement",0,false},
 {"humidity","Humidity","environment.humidity_average_pct","%","humidity","measurement",0,false},
 {"pressure","Barometric pressure","environment.pressure_inHg","inHg","atmospheric_pressure","measurement",2,false},
 {"pressure_trend","Pressure trend","environment.pressure_trend",nullptr,nullptr,nullptr,-1,false},
 {"co2","CO2","co2.ppm","ppm","carbon_dioxide","measurement",0,false},
 {"particle_aqi","Particle AQI live estimate","pms.aqi_live_estimate.index",nullptr,"aqi","measurement",0,false},
 {"particle_category","Particle AQI category","pms.aqi_live_estimate.category",nullptr,nullptr,nullptr,-1,false},
 {"particle_dominant","Particle AQI dominant channel","pms.aqi_live_estimate.dominant",nullptr,nullptr,nullptr,-1,false},
 {"pm1","PM1 mass","pms.pm1_atmospheric_ug_m3","\xCE\xBCg/m\xC2\xB3","pm1","measurement",0,false},
 {"pm25","PM2.5 mass","pms.pm25_atmospheric_ug_m3","\xCE\xBCg/m\xC2\xB3","pm25","measurement",0,false},
 {"pm10","PM10 mass","pms.pm10_atmospheric_ug_m3","\xCE\xBCg/m\xC2\xB3","pm10","measurement",0,false},
 {"particles_03","Particles above 0.3 um","pms.cumulative_counts_per_0_1L[0]","particles/0.1 L",nullptr,"measurement",0,false},
 {"particles_05","Particles above 0.5 um","pms.cumulative_counts_per_0_1L[1]","particles/0.1 L",nullptr,"measurement",0,false},
 {"particles_10","Particles above 1.0 um","pms.cumulative_counts_per_0_1L[2]","particles/0.1 L",nullptr,"measurement",0,false},
 {"particles_25","Particles above 2.5 um","pms.cumulative_counts_per_0_1L[3]","particles/0.1 L",nullptr,"measurement",0,false},
 {"particles_50","Particles above 5.0 um","pms.cumulative_counts_per_0_1L[4]","particles/0.1 L",nullptr,"measurement",0,false},
 {"particles_100","Particles above 10 um","pms.cumulative_counts_per_0_1L[5]","particles/0.1 L",nullptr,"measurement",0,false},
 {"bosch_iaq","VoC air quality (Bosch IAQ)","gas_model.iaq",nullptr,nullptr,"measurement",0,false},
 {"voc_equivalent","Bosch VOC equivalent","gas_model.voc_equivalent_ppm","ppm",nullptr,"measurement",3,false},
 {"illuminance","Illuminance","light.lux_estimate","lx","illuminance","measurement",1,false},
 {"si_temperature","Si7021 temperature","si7021.temperature_C","\xC2\xB0" "C","temperature","measurement",1,true},
 {"si_humidity","Si7021 humidity","si7021.humidity_pct","%","humidity","measurement",1,true},
 {"bme_temperature","BME680 temperature","bme.temperature_C","\xC2\xB0" "C","temperature","measurement",1,true},
 {"bme_humidity","BME680 humidity","bme.humidity_pct","%","humidity","measurement",1,true},
 {"bme_pressure","BME680 station pressure","bme.station_pressure_Pa","Pa","atmospheric_pressure","measurement",1,true},
 {"bme_dewpoint","BME680 dew point","bme.dewpoint_C_water_model","\xC2\xB0" "C","temperature","measurement",1,true},
 {"bme_absolute_humidity","BME680 absolute humidity","bme.absolute_humidity_g_m3","g/m\xC2\xB3",nullptr,"measurement",2,true},
 {"scd_temperature","SCD41 temperature","co2.temperature_C","\xC2\xB0" "C","temperature","measurement",1,true},
 {"scd_humidity","SCD41 humidity","co2.humidity_pct","%","humidity","measurement",1,true},
 {"pressure_change","Pressure fitted five-minute change","environment.pressure_change_5min_inHg","inHg",nullptr,"measurement",4,true},
 {"pm25_aqi","PM2.5 AQI live estimate","pms.aqi_live_estimate.pm25_index",nullptr,"aqi","measurement",0,true},
 {"pm10_aqi","PM10 AQI live estimate","pms.aqi_live_estimate.pm10_index",nullptr,"aqi","measurement",0,true},
 {"gas_raw","Gas raw resistance","gas_model.raw_kOhm","k\xCE\xA9",nullptr,"measurement",3,true},
 {"gas_filtered","Gas filtered resistance","gas_model.filtered_kOhm","k\xCE\xA9",nullptr,"measurement",3,true},
 {"gas_baseline","Gas baseline resistance","gas_model.baseline_kOhm","k\xCE\xA9",nullptr,"measurement",3,true},
 {"gas_previous_raw","Gas previous raw resistance","gas_model.previous_raw_kOhm","k\xCE\xA9",nullptr,"measurement",3,true},
 {"gas_previous_filtered","Gas previous filtered resistance","gas_model.previous_filtered_kOhm","k\xCE\xA9",nullptr,"measurement",3,true},
 {"gas_previous_baseline","Gas previous baseline resistance","gas_model.previous_baseline_kOhm","k\xCE\xA9",nullptr,"measurement",3,true},
 {"gas_ratio","Gas filtered to baseline ratio","gas_model.ratio_F_B",nullptr,nullptr,"measurement",4,true},
 {"gas_deviation","Gas baseline deviation","gas_model.deviation_pct","%",nullptr,"measurement",2,true},
 {"gas_step","Gas sample step","gas_model.sample_step_pct","%",nullptr,"measurement",2,true},
 {"gas_interval","Gas sample interval","gas_model.dt_s","s","duration","measurement",2,true},
 {"gas_alpha","Gas filter alpha","gas_model.alpha",nullptr,nullptr,"measurement",6,true},
 {"gas_beta","Gas baseline beta","gas_model.beta",nullptr,nullptr,"measurement",6,true},
 {"gas_reference_samples","Gas reference samples","gas_model.local_reference_samples",nullptr,nullptr,"total_increasing",0,true},
 {"gas_events","Gas events","gas_model.event_count",nullptr,nullptr,"total_increasing",0,true},
 {"gas_event_step","Gas last event step","gas_model.event_step_pct","%",nullptr,"measurement",2,true},
 {"iaq_accuracy","Bosch IAQ accuracy","gas_model.iaq_accuracy",nullptr,nullptr,nullptr,0,true},
 {"iaq_stabilized","Bosch stabilized","gas_model.iaq_stabilized",nullptr,nullptr,nullptr,-1,true,true},
 {"iaq_run_in","Bosch run-in complete","gas_model.iaq_run_in",nullptr,nullptr,nullptr,-1,true,true},
 {"bsec_status","BSEC status code","gas_model.bsec_status",nullptr,nullptr,nullptr,0,true},
 {"heater_target","Bosch heater target","bme.heater_target_C","\xC2\xB0" "C","temperature","measurement",0,true},
 {"heater_duration","Bosch heater duration","bme.heater_duration_ms","ms","duration","measurement",0,true},
 {"heater_stable","Bosch heater stable","bme.heater_stable",nullptr,nullptr,nullptr,-1,true,true},
 {"gas_valid","Bosch gas valid","bme.gas_valid_flag",nullptr,nullptr,nullptr,-1,true,true},
 {"bme_fresh","BME680 fresh","bme.fresh",nullptr,nullptr,nullptr,-1,true,true},
 {"si_fresh","Si7021 fresh","si7021.fresh",nullptr,nullptr,nullptr,-1,true,true},
 {"co2_fresh","SCD41 fresh","co2.fresh",nullptr,nullptr,nullptr,-1,true,true},
 {"pms_fresh","PMS5003 fresh","pms.fresh",nullptr,nullptr,nullptr,-1,true,true},
 {"counts_valid","Particle counts valid","pms.counts_valid",nullptr,nullptr,nullptr,-1,true,true},
 {"light_fresh","VEML7700 fresh","light.fresh",nullptr,nullptr,nullptr,-1,true,true},
 {"light_saturated","Light saturated","light.saturated",nullptr,nullptr,nullptr,-1,true,true},
 {"si_heater","Si7021 heater enabled","si7021.heater_enabled",nullptr,nullptr,nullptr,-1,true,true},
 {"bme_age","BME680 sample age","bme.age_s","s","duration","measurement",1,true},
 {"si_age","Si7021 sample age","si7021.age_s","s","duration","measurement",1,true},
 {"co2_age","SCD41 sample age","co2.age_s","s","duration","measurement",1,true},
 {"pms_age","PMS5003 sample age","pms.age_s","s","duration","measurement",1,true},
 {"bme_good","BME680 good samples","bme.good",nullptr,nullptr,"total_increasing",0,true},
 {"bme_failed","BME680 failed samples","bme.failed",nullptr,nullptr,"total_increasing",0,true},
 {"si_good","Si7021 good samples","si7021.good",nullptr,nullptr,"total_increasing",0,true},
 {"si_failed","Si7021 failed samples","si7021.failed",nullptr,nullptr,"total_increasing",0,true},
 {"co2_good","SCD41 good samples","co2.good",nullptr,nullptr,"total_increasing",0,true},
 {"co2_failed","SCD41 failed samples","co2.failed",nullptr,nullptr,"total_increasing",0,true},
 {"pms_good","PMS5003 good frames","pms.good_frames",nullptr,nullptr,"total_increasing",0,true},
 {"pms_bad","PMS5003 bad frames","pms.bad_frames",nullptr,nullptr,"total_increasing",0,true},
 {"pms_error","PMS5003 error code","pms.sensor_error",nullptr,nullptr,nullptr,0,true},
 {"pms_bytes","PMS5003 received bytes","pms.rx_bytes","B","data_size","total_increasing",0,true},
 {"pms_byte_age","PMS5003 byte age","pms.last_byte_age_s","s","duration","measurement",1,true},
 {"light_raw","Light raw reading","light.raw",nullptr,nullptr,"measurement",0,true},
 {"backlight_target","Backlight target","light.target_pct","%",nullptr,"measurement",1,true},
 {"backlight_pwm","Backlight PWM enabled","light.pwm_enabled",nullptr,nullptr,nullptr,-1,true,true},
 {"fps","Display frame rate","display.fps","fps",nullptr,"measurement",1,true},
 {"draw_mean","Display mean draw time","display.mean_draw_ms","ms","duration","measurement",2,true},
 {"draw_max","Display maximum draw time","display.max_draw_ms","ms","duration","measurement",2,true},
 {"frame_overruns","Display frame overruns","display.frame_overruns",nullptr,nullptr,"total_increasing",0,true},
 {"screen","Current screen","screen_id",nullptr,nullptr,nullptr,-1,true},
 {"firmware","Firmware","firmware",nullptr,nullptr,nullptr,-1,true},
 {"uptime","Uptime","uptime_s","s","duration","measurement",0,true},
 {"wifi_rssi","Wi-Fi signal","wifi_rssi_dbm","dBm","signal_strength","measurement",0,true},
 {"wifi_dropouts","Wi-Fi dropouts","wifi_dropouts",nullptr,nullptr,"total_increasing",0,true},
 {"heap_free","Free heap","heap_free","B","data_size","measurement",0,true},
 {"heap_min","Minimum free heap","heap_min","B","data_size","measurement",0,true}
};
constexpr size_t HA_ENTITY_COUNT=sizeof(HA_ENTITIES)/sizeof(HA_ENTITIES[0]);
esp_mqtt_client_handle_t mqttClient=nullptr;
std::atomic<bool> mqttConnected{false},mqttDiscoveryRequested{false};
std::atomic<uint32_t> mqttConnections{0},mqttErrors{0};
char mqttDeviceID[32]={0},mqttStateTopic[64]={0},mqttAvailabilityTopic[64]={0};
uint32_t mqttStateAt=0,mqttDiscoveryAt=0,mqttRetryAt=0,mqttReports=0;
size_t mqttDiscoveryIndex=HA_ENTITY_COUNT;
bool mqttWasConnected=false,mqttNeedOnline=false;
String statusJSON();

String mqttDiscoveryJSON(const HAEntity& entity){
 cJSON* root=cJSON_CreateObject();if(!root)return String();
 String unique=String(mqttDeviceID)+"_"+entity.id;
 cJSON_AddStringToObject(root,"name",entity.name);
 cJSON_AddStringToObject(root,"unique_id",unique.c_str());
 cJSON_AddStringToObject(root,"state_topic",mqttStateTopic);
 String value=String("value_json.")+entity.path;
 String valueTemplate=entity.binary?String("{{ 'ON' if ")+value+" else 'OFF' }}":String("{{ ")+value+" }}";
 cJSON_AddStringToObject(root,"value_template",valueTemplate.c_str());
 if(entity.unit)cJSON_AddStringToObject(root,"unit_of_measurement",entity.unit);
 if(entity.deviceClass)cJSON_AddStringToObject(root,"device_class",entity.deviceClass);
 if(entity.stateClass)cJSON_AddStringToObject(root,"state_class",entity.stateClass);
 if(entity.precision>=0)cJSON_AddNumberToObject(root,"suggested_display_precision",entity.precision);
 if(entity.diagnostic)cJSON_AddStringToObject(root,"entity_category","diagnostic");
 cJSON_AddNumberToObject(root,"expire_after",MQTT_EXPIRE_S);
 cJSON_AddNumberToObject(root,"qos",0);
 // Zero and false are valid readings. Only null/missing fields are unavailable.
 cJSON* availability=cJSON_AddArrayToObject(root,"availability");
 cJSON* deviceAvailability=cJSON_CreateObject();cJSON_AddItemToArray(availability,deviceAvailability);
 cJSON_AddStringToObject(deviceAvailability,"topic",mqttAvailabilityTopic);
 cJSON* readingAvailability=cJSON_CreateObject();cJSON_AddItemToArray(availability,readingAvailability);
 cJSON_AddStringToObject(readingAvailability,"topic",mqttStateTopic);
 String available=String("{{ 'online' if ")+value+" is defined and "+value+" is not none else 'offline' }}";
 cJSON_AddStringToObject(readingAvailability,"value_template",available.c_str());
 cJSON_AddStringToObject(root,"availability_mode","all");
 cJSON* device=cJSON_AddObjectToObject(root,"device");
 cJSON* identifiers=cJSON_AddArrayToObject(device,"identifiers");
 cJSON_AddItemToArray(identifiers,cJSON_CreateString(mqttDeviceID));
 cJSON_AddStringToObject(device,"name","BREATHE");
 cJSON_AddStringToObject(device,"manufacturer","BREATHE");
 cJSON_AddStringToObject(device,"model","XIAO ESP32-C3 Air Monitor");
 cJSON_AddStringToObject(device,"sw_version",FIRMWARE_VERSION);
 String url=String("http://")+WiFi.localIP().toString()+"/";
 cJSON_AddStringToObject(device,"configuration_url",url.c_str());
 char* encoded=cJSON_PrintUnformatted(root);
 String result=encoded?String(encoded):String();
 if(encoded)cJSON_free(encoded);cJSON_Delete(root);return result;
}
void mqttEvent(void*,esp_event_base_t,int32_t eventID,void* data){
 auto event=static_cast<esp_mqtt_event_handle_t>(data);
 switch(eventID){
  case MQTT_EVENT_CONNECTED:
   mqttConnected=true;++mqttConnections;mqttDiscoveryRequested=true;
   // Callback runs on MQTT's task, so subscribe cannot stall the display loop.
   esp_mqtt_client_subscribe(event->client,BREATHE_HA_STATUS_TOPIC,1);break;
  case MQTT_EVENT_DISCONNECTED:mqttConnected=false;break;
  case MQTT_EVENT_ERROR:++mqttErrors;break;
  case MQTT_EVENT_DATA:
   if(event->current_data_offset==0 && event->total_data_len==6 && event->data_len==6 &&
      event->topic_len==(int)strlen(BREATHE_HA_STATUS_TOPIC) &&
      !memcmp(event->topic,BREATHE_HA_STATUS_TOPIC,event->topic_len) && !memcmp(event->data,"online",6))
    mqttDiscoveryRequested=true;
   break;
  default:break;
 }
}
void startMQTT(){
 if(mqttClient || !BREATHE_MQTT_HOST[0] || WiFi.status()!=WL_CONNECTED || !due32(millis(),mqttRetryAt))return;
 mqttRetryAt=millis()+30000;
 // Hardware MAC gives stable IDs across firmware updates and distinct devices.
 uint8_t mac[6];WiFi.macAddress(mac);
 snprintf(mqttDeviceID,sizeof(mqttDeviceID),"breathe_%02x%02x%02x%02x%02x%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
 snprintf(mqttStateTopic,sizeof(mqttStateTopic),"%s/%s/state",BREATHE_MQTT_TOPIC_PREFIX,mqttDeviceID);
 snprintf(mqttAvailabilityTopic,sizeof(mqttAvailabilityTopic),"%s/%s/availability",BREATHE_MQTT_TOPIC_PREFIX,mqttDeviceID);
 esp_mqtt_client_config_t config={};
 config.host=BREATHE_MQTT_HOST;config.port=BREATHE_MQTT_PORT;config.transport=MQTT_TRANSPORT_OVER_TCP;
 config.client_id=mqttDeviceID;
 config.username=BREATHE_MQTT_USERNAME[0]?BREATHE_MQTT_USERNAME:nullptr;
 config.password=BREATHE_MQTT_PASSWORD[0]?BREATHE_MQTT_PASSWORD:nullptr;
 config.lwt_topic=mqttAvailabilityTopic;config.lwt_msg="offline";config.lwt_qos=1;config.lwt_retain=1;
 config.keepalive=20;config.network_timeout_ms=3000;config.reconnect_timeout_ms=10000;
 config.buffer_size=1024;config.out_buffer_size=1024;config.task_stack=6144;
 config.protocol_ver=MQTT_PROTOCOL_V_3_1_1;
 mqttClient=esp_mqtt_client_init(&config);
 if(!mqttClient){Serial.println("MQTT: initialization failed; retrying in 30 seconds.");return;}
 if(esp_mqtt_client_register_event(mqttClient,MQTT_EVENT_ANY,mqttEvent,nullptr)!=ESP_OK || esp_mqtt_client_start(mqttClient)!=ESP_OK){
  esp_mqtt_client_destroy(mqttClient);mqttClient=nullptr;
  Serial.println("MQTT: start failed; retrying in 30 seconds.");return;
 }
 Serial.printf("MQTT: starting %s; %u Home Assistant entities.\n",mqttDeviceID,(unsigned)HA_ENTITY_COUNT);
}
void serviceMQTT(){
 startMQTT();if(!mqttClient)return;
 const bool connected=mqttConnected.load() && WiFi.status()==WL_CONNECTED;
 if(connected!=mqttWasConnected){
  Serial.println(connected?"MQTT: connected.":"MQTT: disconnected; background reconnect active.");
  mqttWasConnected=connected;if(connected)mqttNeedOnline=true;
 }
 if(!connected)return;
 const uint32_t now=millis();
 if(mqttDiscoveryRequested.exchange(false)){
  mqttNeedOnline=true;mqttDiscoveryIndex=0;mqttStateAt=now;
  mqttDiscoveryAt=now+250+(esp_random()%500);
 }
 // Bound the queue to one message, including unacknowledged discovery. Broker
 // outages cannot accumulate samples or drain the display's working memory.
 if(esp_mqtt_client_get_outbox_size(mqttClient)>0)return;
 if(mqttNeedOnline){
  if(esp_mqtt_client_enqueue(mqttClient,mqttAvailabilityTopic,"online",6,1,1,true)>=0)mqttNeedOnline=false;
  return;
 }
 if(due32(now,mqttStateAt)){
  String state=statusJSON();
  // Live data is non-retained, QoS 0; HA expires it if updates stop.
  if(esp_mqtt_client_enqueue(mqttClient,mqttStateTopic,state.c_str(),state.length(),0,0,true)>=0){
   mqttStateAt=now+MQTT_REPORT_MS;++mqttReports;
  }else mqttStateAt=now+1000;
  return;
 }
 if(mqttDiscoveryIndex<HA_ENTITY_COUNT && due32(now,mqttDiscoveryAt)){
  const HAEntity& entity=HA_ENTITIES[mqttDiscoveryIndex];
  String topic=String(BREATHE_HA_DISCOVERY_PREFIX)+"/"+(entity.binary?"binary_sensor/":"sensor/")+mqttDeviceID+"/"+entity.id+"/config";
  String payload=mqttDiscoveryJSON(entity);
  if(payload.length() && esp_mqtt_client_enqueue(mqttClient,topic.c_str(),payload.c_str(),payload.length(),1,1,true)>=0)++mqttDiscoveryIndex;
  mqttDiscoveryAt=now+100;
 }
}
