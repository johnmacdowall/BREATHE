/*
 BREATHE // INSTRUMENT OS 4.4.6 -- 06 October 2026
 Gas main title: VoC AIR QUALITY; calibration details remain on gas-info6.
 Brighter cyan, amber and violet ribbons frame IAQ; motion requires a valid reading.
 Text colors +20% then +30%; main ribbons brightened another 30%; T/RH whole numbers.
 CO2 eye moved up 4 px; shared 33 px main numerals for CO2, particle count and VoC.
 Environment units share the numbers' full font size and baseline.
 Environment temperature/history use Si7021; RH is the mean of Si7021 and BME680.
 Si7021 heater off, CRC checked, asynchronous 2-second sampling on shared I2C 0x40.
 Smaller CO2 eye with caption clearance; registration frame removed on every screen.
 Four main screens; sensor info by long press; service pages by triple-click only.
 Short press: next in current layer. Hold: information / return. Triple: service / return.
 Gas info reduced 13 -> 7; PMS information merged; CO2 source page removed.
 Pressure bar: five-minute fitted direction, fading trail, arrow/steady indicator.
 A question mark means insufficient recent pressure history (minimum two minutes).
 Main cycle: Particle Count -> CO2 -> VoC Air Quality (Bosch IAQ) -> Environment.
 Screen IDs (info pages renumbered after removal / consolidation):
   co2-main: eye; co2-info1: SCD41 readings; co2-info2: history.
   pms-main: total count; pms-info1: mass/counts/size bands; pms-info2: PM2.5 history.
   gas-main: Bosch IAQ; gas-info1: original chamber/heater animation;
   gas-info2: resistance graph; gas-info3: raw/filter/baseline/ratio/deviation;
   gas-info4: heater status; gas-info5: statistics; gas-info6: IAQ diagnostics;
   gas-info7: IAQ history.
   env-main: Si7021 temperature, averaged RH, BME pressure; env-info1: raw BME readings;
   env-info2: dew point; env-info3: absolute humidity; env-info4: pressure units;
   env-info5: Si7021 temperature history; env-info6: averaged RH history;
   env-info7: pressure history; env-info8: Si7021 / BME source comparison.
   sys-info1: sensor coverage; sys-info2: timing; sys-info3: health;
   sys-info4: configuration; sys-info5: light/backlight. Triple-click access only.
 The CO2 eye always uses the SCD41. The lungs screen has been removed.
 Requires the supplied PlatformIO dependencies: Bosch BSEC2 and BME68x.
 BSEC owns the BME680 heater, timing and oversampling; no competing driver.
 Bosch 3.3V / 3-second / 4-day configuration. Calibration state saved in NVS.
 IAQ is Bosch's gas-based indoor index, not EPA AQI or measured VOC ppm.
 Raw gas filtering and the local reference remain separate engineering tools.
 Missing/stale/warming channels show no current number or position marker.
 Particle radar/contact positions and gas/heat animation remain schematics.
 Histories are RAM-only; only the Bosch calibration state survives reboot.
 Existing wiring, light control, PMS and SCD41 retained.
 Wi-Fi credentials are loaded from local include/secrets.h (excluded from Git).
*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <bsec2.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <new>
#include <stdarg.h>
#include <string.h>
#include <initializer_list>

// ----------------------------- LOCAL CONFIG -------------------------------
// Keep Wi-Fi credentials in the ignored local include/secrets.h file.
// With no local credentials, the sensors and display operate without Wi-Fi.
#if __has_include("secrets.h")
#include "secrets.h"
#else
constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";
#endif
constexpr int BUTTON_GPIO = 3;     // XIAO D1 / GPIO3 -> pushbutton -> GND.
constexpr bool ENABLE_PMS = true; // Absent PMS simply stays offline.
constexpr int PMS_RX_GPIO = 20;    // D7 <- PIM477 pin labelled RX (sensor TX).
constexpr int PMS_TX_GPIO = 21;   // D6 -> PIM477 pin labelled TX (sensor RX); no commands sent yet.
constexpr uint32_t PMS_WARMUP_MS = 30000;
uint64_t pmsStartedMs = 0;
constexpr bool ENABLE_VEML7700 = true;
// Onboard transistor switches the backlight. BLK is active-high.
// Set to -1 to disable PWM if restoring unconnected BLK wiring.
constexpr int BACKLIGHT_GPIO = 2; // XIAO D0 -> display BLK; remove BLK-to-3V3 wire.
constexpr bool BACKLIGHT_ACTIVE_HIGH = true;
constexpr uint8_t BACKLIGHT_MIN = 24, BACKLIGHT_MAX = 255;
constexpr float LIGHT_PATH_SCALE = 1.0f; // Tune after installing the clear light pipe.
constexpr bool ENABLE_SCD41 = true;  // SCD41 connected at 0x62 on shared D4/D5 I2C bus.
constexpr bool ENABLE_SI7021 = true; // Si7021 at 0x40 on the same D4/D5 bus, powered at 3.3V.
constexpr uint32_t SI7021_STALE_MS = 6000;
constexpr bool START_AUTO_TOUR = false;
constexpr bool INTAKE_ON_LEFT = true; // Visual convention; flip for final enclosure.
constexpr uint32_t FINAL_SPI_HZ = 40000000;
constexpr uint32_t FRAME_US = 50000, SAMPLE_MS = 2000, TOUR_MS = 15000;
constexpr uint32_t ENV_STALE_MS = 6000, PM_STALE_MS = 8000, CO2_STALE_MS = 15000;
constexpr uint32_t DEBOUNCE_MS = 30, HOLD_MS = 700, CLICK_GAP_MS = 350;
constexpr float FAST_TAU_S = 6.0f, BASE_TAU_S = 180.0f;
constexpr uint32_t BASE_MIN_SAMPLES = 30; // UI learning gate, NOT calibration.
constexpr float EVENT_STEP_PCT = 5.0f;   // Sample-to-sample step, not a risk limit.
constexpr uint32_t EVENT_GAP_MS = 10000;
// These are reported from BSEC's active configuration, never imposed by the UI.
uint16_t heaterTargetC=0, heaterDurationMs=0;
uint8_t osHum=0, osTemp=0, osPres=0;
constexpr float BME_ENCLOSURE_TEMP_OFFSET_C=0.0f; // Tune only after enclosure testing.
constexpr uint64_t BOSCH_SAVE_MS=6ULL*60*60*1000;
constexpr float MAG_A = 17.62f, MAG_B = 243.12f, MAG_C = 6.112f;
constexpr float WATER_VAPOR_R = 461.5f; // J/(kg K); ideal-gas approximation.
static_assert(BUTTON_GPIO < 0 || (BUTTON_GPIO != 4 && BUTTON_GPIO != 5 &&
 BUTTON_GPIO != 6 && BUTTON_GPIO != 7 && BUTTON_GPIO != 8 && BUTTON_GPIO != 10 &&
 (!ENABLE_PMS || BUTTON_GPIO != PMS_RX_GPIO)), "Button conflicts with sensor/display");

Adafruit_ST7789 tft(&SPI, -1, 5, 4);
Bsec2 bosch;
Preferences boschStore;
alignas(8) uint8_t boschMemory[BSEC_INSTANCE_SIZE];
const uint8_t BOSCH_CONFIG[] = {
#include "config/bme680/bme680_iaq_33v_3s_4d/bsec_iaq.txt"
};
HardwareSerial pmsPort(1);
WebServer server(80);
GFXcanvas16* canvas = nullptr;

uint64_t nowMs() { return (uint64_t)esp_timer_get_time() / 1000ULL; }
bool due32(uint32_t n, uint32_t d) { return (int32_t)(n-d) >= 0; }
float clipf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
int clipi(int x, int a, int b) { return x < a ? a : (x > b ? b : x); }
float fahrenheit(float c) { return c*1.8f+32.0f; }

struct ScalarSample { uint64_t ms=0; float value=NAN; };
struct EnvSample { uint64_t ms=0; float t=NAN, rh=NAN, p=NAN; };
struct GasSample { uint64_t ms=0; float raw=NAN, filtered=NAN, base=NAN; };
template<class T, int Capacity> struct Ring {
 T items[Capacity]; int head=0, count=0;
 void push(const T& v) { items[head]=v; head=(head+1)%Capacity; if(count<Capacity) ++count; }
 const T& at(int n) const { return items[(head-count+n+Capacity)%Capacity]; }
 void clear() { head=count=0; }
};
Ring<EnvSample,180> envHistory;
Ring<GasSample,100> gasHistory;
Ring<ScalarSample,180> pmHistory, co2History, temperatureHistory;
// Both humidity sources can update independently (Si7021 ~2s, BME ~3s).
Ring<ScalarSample,300> humidityHistory;

struct Environment {
 bool present=false, configured=false, have=false, pending=false;
 bool gasAccepted=false, flagsKnown=false, heaterStable=false, gasValid=false;
 uint8_t address=0x76, variant=0; bool variantKnown=false;
 uint32_t readyAt=0, nextAt=0, retryAt=0, good=0, failed=0, gasRejected=0;
 uint32_t consecutiveFails=0;
 uint64_t ms=0; float t=NAN, rh=NAN, p=NAN, gas=NAN;
} env;
struct GasModel {
 bool have=false, seeded=false; uint32_t samples=0, events=0;
 uint64_t ms=0, eventMs=0;
 float raw=NAN, rawPrev=NAN, f=NAN, fPrev=NAN, base=NAN, basePrev=NAN;
 float dt=NAN, alpha=NAN, beta=NAN, ratio=NAN, deviation=NAN, step=NAN, eventStep=NAN;
 void accept(float r, uint64_t at) {
  if(!isfinite(r) || r<=0) return;
  bool reset = !have || at<=ms || at-ms>30000;
  seeded=reset;
  if(reset) {
   raw=f=base=r;
   // Initialization is not a filter update: no previous sample or dt exists.
   rawPrev=fPrev=basePrev=dt=alpha=beta=step=NAN;
   ratio=1; deviation=0; samples=1;
   // A gap begins a new local reference. No event is inferred across the gap.
   eventMs=0; eventStep=NAN;
  } else {
   rawPrev=raw; fPrev=f; basePrev=base;
   dt=(at-ms)/1000.0f; alpha=-expm1f(-dt/FAST_TAU_S);
   beta=-expm1f(-dt/BASE_TAU_S); raw=r;
   f=fPrev+alpha*(raw-fPrev);
   base=basePrev+beta*(f-basePrev);
   ratio=f/base; deviation=100.0f*(ratio-1.0f);
   step=100.0f*(raw/rawPrev-1.0f);
   if(samples<UINT32_MAX) ++samples;
   if(samples>=BASE_MIN_SAMPLES && fabsf(step)>=EVENT_STEP_PCT &&
      (!eventMs || at-eventMs>=EVENT_GAP_MS)) { eventMs=at; eventStep=step; ++events; }
  }
  ms=at; have=true;
 }
} gas;
struct Moisture {
 bool valid=false; float es=NAN, e=NAN, gamma=NAN, dew=NAN, ah=NAN;
};
Moisture moisture(float t, float rh) {
 Moisture m;
 // Deliberately bounded room-air, liquid-water model. No ice/frost-point claim.
 if(!isfinite(t) || !isfinite(rh) || t<0 || t>60 || rh<0 || rh>100) return m;
 m.valid=true; m.es=MAG_C*expf(MAG_A*t/(MAG_B+t)); m.e=m.es*rh/100.0f;
 m.ah=(100000.0f/WATER_VAPOR_R)*m.e/(273.15f+t);
 if(rh>0) { m.gamma=logf(rh/100.0f)+MAG_A*t/(MAG_B+t);
  m.dew=MAG_B*m.gamma/(MAG_A-m.gamma); }
 return m;
}
struct PMSDiagnostics {
 uint64_t bytes=0, lastByteMs=0;
 uint8_t recent[32]={}; uint8_t head=0, count=0;
 void record(uint8_t c, uint64_t at) {
  ++bytes; lastByteMs=at; recent[head]=c; head=(head+1)%32;
  if(count<32)++count;
 }
 String hex() const {
  String out; out.reserve(96);
  for(int i=0;i<count;++i) {
   if(i)out+=' ';
   char pair[3]; snprintf(pair,sizeof(pair),"%02X",recent[(head+32-count+i)%32]);
   out+=pair;
  }
  return out;
 }
} pmsDiag;
struct PMSState {
 bool have=false, countsOK=false, latestOK=false;
 uint64_t ms=0, eventMs=0, historyMs=0;
 uint16_t pm1=0, pm25=0, pm10=0, counts[6]={};
 uint8_t version=0, error=0;
 uint32_t good=0, bad=0, countBad=0;
} pm;
struct CO2State {
 bool present=false, have=false, lastOK=false;
 uint64_t ms=0; float ppm=NAN, t=NAN, rh=NAN;
 uint8_t phase=0; uint32_t nextAt=0, good=0, bad=0;
} co2;
struct Si7021State {
 bool present=false, configured=false, have=false, lastOK=false;
 uint8_t phase=0, userRegister=0;
 uint32_t nextAt=100, good=0, bad=0; // Allow the initial power-up interval.
 uint64_t ms=0;
 float t=NAN, rh=NAN, pendingRh=NAN;
} si7021;
bool envFresh() { return env.have && nowMs()-env.ms<ENV_STALE_MS; }
bool gasFresh() { return env.gasAccepted && gas.have && nowMs()-gas.ms<ENV_STALE_MS; }
bool pmFresh() { return nowMs()-pmsStartedMs>=PMS_WARMUP_MS && pm.have && pm.latestOK && nowMs()-pm.ms<PM_STALE_MS; }
bool co2Fresh() { return co2.have && co2.lastOK && nowMs()-co2.ms<CO2_STALE_MS; }
bool si7021Fresh() { return ENABLE_SI7021 && si7021.configured && si7021.have && si7021.lastOK && nowMs()-si7021.ms<SI7021_STALE_MS; }
bool environmentHumidityFresh() { return si7021Fresh() && envFresh(); }
float environmentTemperatureF() { return si7021Fresh()?fahrenheit(si7021.t):NAN; }
float environmentHumidity() { return environmentHumidityFresh()?(si7021.rh+env.rh)*0.5f:NAN; }
void recordEnvironmentHumidity() {
 static uint64_t lastSi=0,lastBme=0;
 if(!environmentHumidityFresh() || (lastSi==si7021.ms && lastBme==env.ms))return;
 ScalarSample s;s.ms=nowMs();s.value=environmentHumidity();humidityHistory.push(s);
 lastSi=si7021.ms;lastBme=env.ms;
}


struct IAQState {
 bool initialized=false, have=false, stabilized=false, runIn=false;
 bool storage=false, restored=false, saved=false;
 uint8_t accuracy=0; uint64_t ms=0, lastSaved=0;
 float index=NAN, staticIndex=NAN, vocEquivalent=NAN;
 int bsecStatus=0, sensorStatus=0;
} iaq;
Ring<ScalarSample,180> iaqHistory;
bool iaqFresh() { return iaq.initialized && iaq.have && gasFresh() && nowMs()-iaq.ms<10000; }
bool iaqReady() { return iaqFresh() && iaq.stabilized && iaq.runIn && iaq.accuracy>0; }

// -------------------------------- I2C -------------------------------------
bool registerBytes(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t n) {
 Wire.beginTransmission(addr); Wire.write(reg);
 if(Wire.endTransmission(false)!=0) return false;
 if(Wire.requestFrom(addr,n)!=n) { while(Wire.available()) Wire.read(); return false; }
 for(uint8_t i=0;i<n;++i) data[i]=(uint8_t)Wire.read();
 return true;
}
// VEML7700: little-endian registers; gain 1/8, integration 100ms.
// Lux is an estimate at the sensor, not calibrated room illuminance through a pipe.
struct LightState {
 bool present=false, have=false, pwmReady=false, saturated=false;
 uint32_t nextAt=0, good=0, bad=0;
 uint64_t ms=0;
 uint16_t raw=0;
 float lux=NAN, filtered=NAN;
 uint8_t target=180, duty=180;
} light;
bool lightFresh() { return light.have && nowMs()-light.ms<3000; }
static_assert(BACKLIGHT_GPIO == -1 || BACKLIGHT_GPIO == 2,
 "Backlight pin must be disabled or the reserved D0/GPIO2");
bool lightWrite(uint8_t reg, uint16_t value) {
 Wire.beginTransmission(0x10); Wire.write(reg);
 Wire.write((uint8_t)value); Wire.write((uint8_t)(value>>8));
 return Wire.endTransmission()==0;
}
void writeBacklight(uint8_t duty) {
 if(!light.pwmReady)return;
 uint8_t value=BACKLIGHT_ACTIVE_HIGH?duty:255-duty;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
 ledcWrite(BACKLIGHT_GPIO,value);
#else
 ledcWrite(0,value);
#endif
}
void beginBacklight() {
 if(BACKLIGHT_GPIO<0)return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
 light.pwmReady=ledcAttach(BACKLIGHT_GPIO,5000,8);
#else
 light.pwmReady=ledcSetup(0,5000,8)>0;
 if(light.pwmReady)ledcAttachPin(BACKLIGHT_GPIO,0);
#endif
 writeBacklight(light.duty);
}
void serviceLight() {
 if(!ENABLE_VEML7700 || !due32(millis(),light.nextAt))return;
 uint32_t n=millis();
 if(!light.present) {
  light.present=lightWrite(0x03,0x0000) && lightWrite(0x00,0x1000);
  light.have=false;
  light.nextAt=n+(light.present?300:10000);
  Serial.printf("VEML7700 %s at 0x10\n",light.present?"ready":"not found");
  return;
 }
 light.nextAt=n+250;
 uint8_t bytes[2];
 if(!registerBytes(0x10,0x04,bytes,2)) {
  ++light.bad; light.present=false; light.have=false; light.nextAt=n+10000;
  light.target=180; light.duty=180; writeBacklight(light.duty); return;
 }
 light.raw=(uint16_t)bytes[0]|((uint16_t)bytes[1]<<8);
 light.saturated=light.raw>=65500;
 // Original Adafruit breakout conversion (0.0576 lx/count at gain1/100ms).
 // Saturation selects maximum brightness; reported lux becomes null.
 light.lux=light.raw*0.4608f*LIGHT_PATH_SCALE;
 float dt=light.have?(nowMs()-light.ms)/1000.0f:0;
 light.filtered=light.have?light.filtered+(-expm1f(-dt/2.0f))*(light.lux-light.filtered):light.lux;
 light.ms=nowMs(); light.have=true; ++light.good;
 float fraction=clipf(log1pf(light.filtered)/log1pf(1000.0f),0,1);
 light.target=light.saturated?BACKLIGHT_MAX:
  (uint8_t)lroundf(BACKLIGHT_MIN+(BACKLIGHT_MAX-BACKLIGHT_MIN)*fraction);
 int delta=(int)light.target-light.duty;
 if(abs(delta)>2)light.duty=(uint8_t)((int)light.duty+clipi(delta,-8,8));
 writeBacklight(light.duty);
}

// BSEC2 1.10.2610 converts bme68xData.pressure from Pa to hPa before callback.
// Normalize it back to Pa once, so every existing environment calculation agrees.
void boschData(const bme68xData data,const bsecOutputs outputs,const Bsec2 instance) {
 const uint64_t at=nowMs();
#ifdef BME68X_USE_FPU
 float t=data.temperature, h=data.humidity;
#else
 float t=data.temperature/100.0f, h=data.humidity/1000.0f;
#endif
 float p=data.pressure*100.0f;
 bool ok=isfinite(t)&&isfinite(h)&&isfinite(p)&&t>=-40&&t<=85&&h>=0&&h<=100&&p>=30000&&p<=110000;
 env.flagsKnown=true;
 env.heaterStable=(data.status&BME68X_HEAT_STAB_MSK)!=0;
 env.gasValid=(data.status&BME68X_GASM_VALID_MSK)!=0;
 (void)instance;
 // The callback instance is const; read the owning sensor's current settings.
 const auto& active=bosch.sensor.getHeaterConfiguration();
 heaterTargetC=active.heatr_temp;heaterDurationMs=active.heatr_dur;
 bosch.sensor.getTPH(osHum,osTemp,osPres);
 float r=data.gas_resistance/1000.0f;
 env.gasAccepted=ok&&env.heaterStable&&env.gasValid&&isfinite(r)&&r>0;
 if(ok) {
  env.t=t;env.rh=h;env.p=p;env.ms=at;env.have=true;env.consecutiveFails=0;++env.good;
  EnvSample sample;sample.ms=at;sample.t=t;sample.rh=h;sample.p=p;envHistory.push(sample);
 } else {++env.failed;env.have=false;}
 env.gas=env.gasAccepted?r:NAN;
 if(env.gasAccepted) {
  gas.accept(r,at);GasSample gs;gs.ms=at;gs.raw=r;gs.filtered=gas.f;gs.base=gas.base;gasHistory.push(gs);
 } else ++env.gasRejected;
 bool gotIndex=false;
 iaq.staticIndex=iaq.vocEquivalent=NAN;
 for(uint8_t i=0;i<outputs.nOutputs;++i) {
  const bsecData& out=outputs.output[i];
  switch(out.sensor_id) {
   case BSEC_OUTPUT_IAQ:iaq.index=out.signal;iaq.accuracy=out.accuracy;gotIndex=true;break;
   case BSEC_OUTPUT_STATIC_IAQ:iaq.staticIndex=out.signal;break;
   case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:iaq.vocEquivalent=out.signal;break;
   case BSEC_OUTPUT_STABILIZATION_STATUS:iaq.stabilized=out.signal>=1;break;
   case BSEC_OUTPUT_RUN_IN_STATUS:iaq.runIn=out.signal>=1;break;
   default:break;
  }
 }
 iaq.have=gotIndex&&isfinite(iaq.index)&&iaq.index>=0;
 if(iaq.have)iaq.ms=at;
 if(iaqReady()) {ScalarSample q;q.ms=at;q.value=iaq.index;iaqHistory.push(q);}
}
void beginBME() {
 env.pending=false;env.gasAccepted=false;env.flagsKnown=false;env.have=false;
 iaq.have=false;iaq.stabilized=false;iaq.runIn=false;iaq.accuracy=0;
 bosch.allocateMemory(boschMemory);
 env.address=0x76;env.present=bosch.begin(env.address,Wire);
 if(!env.present){env.address=0x77;env.present=bosch.begin(env.address,Wire);}
 env.configured=env.present&&bosch.setConfig(BOSCH_CONFIG);
 if(env.configured) {
  bosch.setTemperatureOffset(BME_ENCLOSURE_TEMP_OFFSET_C);
  if(!iaq.storage)iaq.storage=boschStore.begin("breathe-bsec",false);
  // Namespace/key tied to the pinned library, configuration and sensor address.
  const char* key=env.address==0x76?"v2610-33v4d-76":"v2610-33v4d-77";
  if(iaq.storage&&boschStore.getBytesLength(key)==BSEC_MAX_STATE_BLOB_SIZE) {
   uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
   if(boschStore.getBytes(key,state,sizeof(state))==sizeof(state)) {
    iaq.restored=bosch.setState(state);
    if(!iaq.restored)boschStore.remove(key);
   }
  }
  bsecSensor wanted[]={BSEC_OUTPUT_IAQ,BSEC_OUTPUT_STATIC_IAQ,
   BSEC_OUTPUT_RAW_TEMPERATURE,BSEC_OUTPUT_RAW_HUMIDITY,BSEC_OUTPUT_RAW_PRESSURE,
   BSEC_OUTPUT_RAW_GAS,BSEC_OUTPUT_STABILIZATION_STATUS,BSEC_OUTPUT_RUN_IN_STATUS,
   BSEC_OUTPUT_BREATH_VOC_EQUIVALENT};
  env.configured=bosch.updateSubscription(wanted,sizeof(wanted)/sizeof(wanted[0]),BSEC_SAMPLE_RATE_LP);
  if(env.configured)bosch.attachCallback(boschData);
 }
 iaq.initialized=env.configured;iaq.bsecStatus=bosch.status;iaq.sensorStatus=bosch.sensor.status;
 env.consecutiveFails=0;env.nextAt=millis();env.retryAt=millis()+10000;
 Serial.printf("BSEC initialized=%u addr=0x%02X status=%d sensor=%d state=%u\n",
  iaq.initialized,env.address,iaq.bsecStatus,iaq.sensorStatus,iaq.restored);
}
void serviceBME() {
 if(!iaq.initialized){if(due32(millis(),env.retryAt))beginBME();return;}
 bool ok=bosch.run();iaq.bsecStatus=bosch.status;iaq.sensorStatus=bosch.sensor.status;
 if(!ok && (bosch.status<0||bosch.sensor.status<0)) {
  ++env.failed;env.gasAccepted=false;env.flagsKnown=false;
  iaq.initialized=false;env.configured=false;env.retryAt=millis()+10000;return;
 }
 if(iaqReady()&&iaq.accuracy==3&&iaq.storage&&(!iaq.saved||nowMs()-iaq.lastSaved>=BOSCH_SAVE_MS)) {
  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
  // Rate limit failed writes too; live sensing never waits for calibration saves.
  iaq.lastSaved=nowMs();iaq.saved=true;
  if(bosch.getState(state)) {
   const char* key=env.address==0x76?"v2610-33v4d-76":"v2610-33v4d-77";
   boschStore.putBytes(key,state,sizeof(state));
  }
 }
}

// ------------------------ PMS5003: bounded streaming parser ----------------
uint16_t be16(const uint8_t* p) { return ((uint16_t)p[0]<<8)|p[1]; }
bool acceptPMS(const uint8_t* b, uint64_t at) {
 if(b[0]!=0x42 || b[1]!=0x4D || be16(b+2)!=28) return false;
 uint16_t sum=0; for(int i=0;i<30;++i) sum+=b[i];
 if(sum!=be16(b+30)) return false;
 pm.version=b[28]; pm.error=b[29];
 if(pm.error) { pm.latestOK=false; return false; }
 uint16_t newPM=be16(b+12);
 if(pm.have && at-pm.ms<PM_STALE_MS && newPM>pm.pm25+5 &&
    newPM>pm.pm25*1.5f && (!pm.eventMs || at-pm.eventMs>5000)) pm.eventMs=at;
 // ATMOSPHERIC fields, never the CF=1 factory-environment fields.
 pm.pm1=be16(b+10); pm.pm25=newPM; pm.pm10=be16(b+14);
 for(int i=0;i<6;++i) pm.counts[i]=be16(b+16+2*i);
 pm.countsOK=true;
 for(int i=1;i<6;++i) if(pm.counts[i]>pm.counts[i-1]) pm.countsOK=false;
 if(!pm.countsOK) ++pm.countBad;
 pm.ms=at; pm.have=pm.latestOK=true; ++pm.good;
 if(at-pmsStartedMs<PMS_WARMUP_MS)return true;
 if(!pm.historyMs || at-pm.historyMs>=1000) {
  ScalarSample s; s.ms=at; s.value=pm.pm25; pmHistory.push(s); pm.historyMs=at;
 }
 return true;
}
struct PMSParser {
 uint8_t b[32]={}; int used=0; uint32_t lastByte=0;
 void feed(uint8_t c, uint32_t at32, uint64_t at64) {
  if(used && at32-lastByte>200) used=0;
  lastByte=at32;
  if(used==0 && c!=0x42) return;
  if(used==1 && c!=0x4D) { used=(c==0x42)?1:0; return; }
  b[used++]=c;
  if(used==4 && be16(b+2)!=28) { ++pm.bad; resync(); }
  if(used==32) { if(!acceptPMS(b,at64)) { ++pm.bad; resync(); } else used=0; }
 }
 void resync() {
  for(int i=1;i<used;++i) if(b[i]==0x42 && (i==used-1 || b[i+1]==0x4D)) {
   memmove(b,b+i,used-i); used-=i; return;
  }
  used=0;
 }
} pmsParser;
void servicePMS() {
 if(!ENABLE_PMS) return;
 // At 9600 baud a 32-byte frame takes ~33ms; drain before and after drawing.
 for(int budget=0;budget<256 && pmsPort.available();++budget) {
  uint8_t c=(uint8_t)pmsPort.read(); uint64_t at=nowMs();
  pmsDiag.record(c,at);
  pmsParser.feed(c,millis(),at);
 }
}

// Optional SCD41 periodic mode. No forced calibration, EEPROM or ASC changes.
uint8_t scdCRC(const uint8_t* p) {
 uint8_t c=0xFF;
 for(int i=0;i<2;++i) { c^=p[i]; for(int j=0;j<8;++j) c=(c&0x80)?(c<<1)^0x31:c<<1; }
 return c;
}
bool scdCommand(uint16_t cmd) {
 Wire.beginTransmission(0x62); Wire.write(cmd>>8); Wire.write(cmd&255);
 return Wire.endTransmission()==0;
}
bool scdWords(uint8_t* p,uint8_t n) {
 if(Wire.requestFrom((uint8_t)0x62,n)!=n) { while(Wire.available())Wire.read(); return false; }
 for(uint8_t i=0;i<n;++i) p[i]=(uint8_t)Wire.read();
 for(uint8_t i=0;i<n;i+=3) if(scdCRC(p+i)!=p[i+2])return false;
 return true;
}
void scdFailure() { ++co2.bad; co2.lastOK=false; co2.phase=0; co2.nextAt=millis()+10000; }
void serviceCO2() {
 if(!ENABLE_SCD41 || !due32(millis(),co2.nextAt))return;
 uint32_t n=millis(); uint8_t data[9];
 switch(co2.phase) {
 case 0:
  co2.present=scdCommand(0x3F86); // stop a possibly already-running periodic cycle
  if(!co2.present) { scdFailure(); return; }
  co2.phase=1; co2.nextAt=n+500; break;
 case 1:
  if(!scdCommand(0x21B1)) { scdFailure(); return; }
  co2.phase=2; co2.nextAt=n+5000; break;
 case 2:
  if(!scdCommand(0xE4B8)) { scdFailure(); return; }
  co2.phase=3; co2.nextAt=n+2; break;
 case 3:
  if(!scdWords(data,3)) { scdFailure(); return; }
  if((be16(data)&0x07FF)==0) { co2.phase=2; co2.nextAt=n+1000; break; }
  if(!scdCommand(0xEC05)) { scdFailure(); return; }
  co2.phase=4; co2.nextAt=n+2; break;
 case 4:
  if(!scdWords(data,9) || be16(data)==0) { scdFailure(); return; }
  co2.ppm=be16(data); co2.t=-45+175.0f*be16(data+3)/65535.0f;
  co2.rh=100.0f*be16(data+6)/65535.0f; co2.ms=nowMs();
  co2.have=co2.lastOK=true; ++co2.good;
  { ScalarSample s; s.ms=co2.ms; s.value=co2.ppm; co2History.push(s); }
  co2.phase=2; co2.nextAt=n+1000; break;
 }
}

// Si7021-A20: no-hold conversions leave the shared bus and animation running.
// RH and temperature are checked separately with CRC-8 (poly 0x31, init 0).
// The 0xE0 cached-temperature command has no CRC, so use 0xF3 for temperature.
bool si7021Command(uint8_t command) {
 Wire.beginTransmission(0x40);Wire.write(command);
 return Wire.endTransmission()==0;
}
bool si7021ReadWord(uint16_t& word) {
 uint8_t data[3];
 if(Wire.requestFrom((uint8_t)0x40,(uint8_t)3)!=3) {
  while(Wire.available())Wire.read();return false;
 }
 for(int i=0;i<3;++i)data[i]=(uint8_t)Wire.read();
 uint8_t crc=0;
 for(int i=0;i<2;++i) {crc^=data[i];for(int bit=0;bit<8;++bit)crc=(crc&0x80)?(crc<<1)^0x31:crc<<1;}
 if(crc!=data[2])return false;
 word=((uint16_t)data[0]<<8)|data[1];return true;
}
void si7021Failure() {
 ++si7021.bad;si7021.lastOK=si7021.configured=false;
 si7021.pendingRh=NAN;si7021.phase=0;si7021.nextAt=millis()+10000;
}
void serviceSi7021() {
 if(!ENABLE_SI7021 || !due32(millis(),si7021.nextAt))return;
 uint8_t reg=0;uint16_t raw=0;
 switch(si7021.phase) {
 case 0:
  si7021.present=si7021Command(0xFE); // Reset after power-up or a failed cycle.
  if(!si7021.present){si7021Failure();return;}
  si7021.phase=1;si7021.nextAt=millis()+20;break;
 case 1:
  if(!registerBytes(0x40,0xE7,&reg,1)){si7021Failure();return;}
  // Preserve reserved bits; 12-bit RH / 14-bit T, with the heater OFF.
  reg&=(uint8_t)~0x85;
  Wire.beginTransmission(0x40);Wire.write(0xE6);Wire.write(reg);
  if(Wire.endTransmission()!=0){si7021Failure();return;}
  si7021.phase=2;si7021.nextAt=millis();break;
 case 2:
  // Verify configuration on every cycle, including after an unexpected reset.
  if(!registerBytes(0x40,0xE7,&reg,1) || (reg&0xC5)!=0) {
   si7021Failure();return; // Wrong resolution, heater enabled, or supply too low.
  }
  si7021.userRegister=reg;si7021.configured=true;
  if(!si7021Command(0xF5)){si7021Failure();return;}
  si7021.phase=3;si7021.nextAt=millis()+30;break;
 case 3:
  if(!si7021ReadWord(raw) || (raw&3)!=2){si7021Failure();return;}
  si7021.pendingRh=clipf(125.0f*(raw&0xFFFC)/65536.0f-6.0f,0,100);
  if(!si7021Command(0xF3)){si7021Failure();return;}
  si7021.phase=4;si7021.nextAt=millis()+15;break;
 case 4: {
  if(!si7021ReadWord(raw) || (raw&3)!=0){si7021Failure();return;}
  const float t=175.72f*(raw&0xFFFC)/65536.0f-46.85f;
  if(!isfinite(t) || t< -40 || t>125 || !isfinite(si7021.pendingRh)) {si7021Failure();return;}
  // Publish a complete validated pair; never combine a partial failed cycle.
  si7021.t=t;si7021.rh=si7021.pendingRh;si7021.ms=nowMs();
  si7021.have=si7021.lastOK=true;++si7021.good;
  ScalarSample s;s.ms=si7021.ms;s.value=t;temperatureHistory.push(s);
  si7021.phase=2;si7021.nextAt=millis()+SAMPLE_MS;break;
 }
 }
}

// ---------------------- Navigation and button recognizer ------------------
enum Page { AIR_SAMPLE,EYE,CHEMISTRY,ENVIRONMENT,PAGE_COUNT };
const char* PAGE_NAMES[PAGE_COUNT]={"PARTICLE COUNT","CO2","VoC AIR QUALITY","ENVIRONMENT"};
const char* PAGE_IDS[PAGE_COUNT]={"pms","co2","gas","env"};
Page page=AIR_SAMPLE;
bool detail=false,serviceMode=false,autoTour=START_AUTO_TOUR,bootActive=true;
bool serviceReturnDetail=false;
uint8_t serviceReturnSubpage=0;
uint8_t subpage=0,trendMetric=0,infoOrdinal=0,infoTotal=0;
uint64_t pageSince=0,transitionAt=0,bootAt=0;
uint32_t spiHz=FINAL_SPI_HZ;
constexpr uint8_t SYSTEM_DETAIL_COUNT=5;
uint8_t localDetailCount(){switch(page){case EYE:return 2;case AIR_SAMPLE:return 2;case CHEMISTRY:return 7;case ENVIRONMENT:return 8;default:return 0;}}
uint8_t detailCount(){return serviceMode?SYSTEM_DETAIL_COUNT:localDetailCount();}
void screenID(char* out,size_t length){
 if(serviceMode)snprintf(out,length,"sys-info%u",subpage+1);
 else if(!detail)snprintf(out,length,"%s-main",PAGE_IDS[page]);
 else snprintf(out,length,"%s-info%u",PAGE_IDS[page],subpage+1);
}
void navigate(int direction){
 autoTour=false;bootActive=false;
 if(detail)subpage=(subpage+detailCount()+direction)%detailCount();
 else {page=(Page)((page+PAGE_COUNT+direction)%PAGE_COUNT);subpage=0;}
 pageSince=transitionAt=nowMs();
}
void leaveService(){
 serviceMode=false;detail=serviceReturnDetail;subpage=serviceReturnSubpage;
 transitionAt=pageSince=nowMs();
}
void toggleService(){
 autoTour=false;bootActive=false;
 if(serviceMode){leaveService();return;}
 serviceReturnDetail=detail;serviceReturnSubpage=subpage;
 serviceMode=true;detail=true;subpage=0;transitionAt=nowMs();
}
void toggleDetails(){
 autoTour=false;bootActive=false;
 if(serviceMode){leaveService();return;}
 detail=!detail;subpage=0;transitionAt=nowMs();
}
void home(){page=EYE;serviceMode=false;detail=false;subpage=0;autoTour=false;bootActive=false;transitionAt=pageSince=nowMs();}
enum Gesture {NONE,TAP,DOUBLE_TAP,HOLD,TRIPLE_TAP};
struct Button {
 bool raw=false,stable=false,held=false;uint8_t taps=0;
 uint32_t changed=0,pressed=0,released=0;
 Gesture tick(bool r,uint32_t n){
  Gesture out=NONE;
  if(r!=raw){
   // Flush an expired sequence before a new press, even if loop polling was delayed.
   if(r&&!stable&&taps&&n-released>CLICK_GAP_MS){out=taps==1?TAP:DOUBLE_TAP;taps=0;}
   raw=r;changed=n;
  }
  if(n-changed>=DEBOUNCE_MS&&stable!=raw){
   stable=raw;
   if(stable){pressed=n;held=false;}
   else if(!held){released=n;if(++taps==3){taps=0;return TRIPLE_TAP;}}
  }
  if(stable&&!held&&n-pressed>=HOLD_MS){held=true;taps=0;return HOLD;}
  if(taps&&!raw&&!stable&&n-released>CLICK_GAP_MS){out=taps==1?TAP:DOUBLE_TAP;taps=0;}
  return out;
 }
} button;
void applyGesture(Gesture g){
 if(g==TAP)navigate(1);
 else if(g==DOUBLE_TAP){navigate(1);navigate(1);}
 else if(g==HOLD)toggleDetails();
 else if(g==TRIPLE_TAP)toggleService();
}
void serviceButton(){if(BUTTON_GPIO>=0)applyGesture(button.tick(digitalRead(BUTTON_GPIO)==LOW,millis()));}

struct Performance {
 uint64_t frames=0, drawUs=0, startMs=0, fpsAt=0;
 uint32_t windowFrames=0, maxDrawUs=0, overruns=0, maxGapUs=0;
 float fps=0;
} perf;

const uint8_t SmallBits[] PROGMEM={8,8,8,8,8,8,0,0,8,8,20,20,20,20,0,0,0,0,0,0,18,9,5,143,226,65,35,248,80,72,36,0,
 8,8,62,73,72,104,62,11,9,73,62,8,8,96,72,36,12,64,193,129,24,18,9,3,0,28,16,8,6,3,2,73,20,138,98,30,128,8,8,8,8,0,
 0,0,0,0,0,12,8,8,16,16,16,16,16,16,8,8,4,48,16,16,8,8,8,8,8,8,16,16,32,8,73,62,28,107,8,0,0,0,0,8,8,8,127,8,8,8,0,
 24,24,16,32,60,0,0,0,24,24,2,4,4,4,8,8,16,16,32,32,32,64,28,34,65,65,73,65,65,65,34,28,24,40,8,8,8,8,8,8,8,62,62,
 67,1,1,2,6,12,16,32,127,62,65,1,3,28,3,1,1,67,62,6,10,26,18,34,66,127,2,2,2,126,64,64,124,66,1,1,1,66,60,30,49,96,
 64,94,99,65,65,35,30,127,3,2,4,4,8,8,16,16,32,62,65,65,65,62,99,65,65,99,62,60,98,65,65,99,61,1,3,70,60,24,24,0,0,
 0,24,24,24,24,0,0,0,24,24,16,32,1,14,56,64,56,14,1,0,127,0,0,127,0,0,0,64,56,14,1,14,56,64,0,56,68,4,12,24,16,16,0,
 16,16,30,25,136,72,228,146,73,36,146,71,16,12,1,192,8,10,5,2,129,65,16,248,68,65,32,128,126,65,65,65,126,67,65,65,
 67,126,30,33,64,64,64,64,64,64,33,30,124,66,65,65,65,65,65,65,66,124,127,64,64,64,127,64,64,64,64,127,127,64,64,64,
 127,64,64,64,64,64,30,33,64,64,64,67,65,65,33,30,65,65,65,65,127,65,65,65,65,65,62,8,8,8,8,8,8,8,8,62,30,2,2,2,2,2,
 2,2,70,60,66,34,18,10,7,2,65,48,136,66,32,128,64,64,64,64,64,64,64,64,64,127,99,99,85,85,85,73,65,65,65,65,97,97,
 81,81,73,73,69,69,67,67,28,34,65,65,65,65,65,65,34,28,126,67,65,65,67,126,64,64,64,64,28,34,65,65,65,65,65,65,34,
 30,6,2,126,33,144,72,36,51,225,8,130,65,32,64,30,97,64,64,48,14,1,1,67,62,127,4,2,1,0,128,64,32,16,8,4,0,65,65,65,
 65,65,65,65,65,99,62,65,32,136,132,66,32,160,80,40,20,4,0,129,64,160,83,37,162,209,104,72,36,18,0,65,17,5,2,128,
 128,160,80,68,34,32,128,65,17,8,130,129,192,64,32,16,8,4,0,127,3,2,4,8,8,16,32,96,127,28,16,16,16,16,16,16,16,16,
 16,16,28,64,32,32,32,16,16,8,8,4,4,4,2,56,8,8,8,8,8,8,8,8,8,8,56,8,20,34,99,0,0,0,0,0,0,0,0,63,192,48,16,8,0,0,0,0,
 0,0,0,0,0,28,34,2,62,66,66,70,58,64,64,64,124,100,66,66,66,66,100,92,28,34,64,64,64,64,34,28,2,2,2,62,38,66,66,66,
 66,38,58,60,38,66,126,64,64,34,28,14,16,16,126,16,16,16,16,16,16,16,58,38,66,66,66,66,38,58,2,34,28,64,64,64,92,98,
 66,66,66,66,66,66,8,8,0,56,8,8,8,8,8,8,127,8,8,0,56,8,8,8,8,8,8,8,8,8,112,64,32,16,8,132,130,129,192,144,72,34,16,
 128,240,16,16,16,16,16,16,16,16,16,14,126,73,73,73,73,73,73,73,92,98,66,66,66,66,66,66,60,102,66,66,66,66,102,60,
 92,100,66,66,66,66,100,124,64,64,64,58,38,66,66,66,66,38,58,2,2,2,60,50,32,32,32,32,32,32,60,66,64,112,14,2,66,60,
 16,16,126,16,16,16,16,16,16,14,66,66,66,66,66,66,70,58,66,66,36,36,36,24,24,24,129,64,150,139,69,162,208,144,72,66,
 36,24,24,24,36,36,66,66,34,36,36,20,24,8,8,8,16,48,126,2,4,8,16,32,64,126,6,8,8,8,8,8,48,8,8,8,8,8,6,8,8,8,8,8,8,8,
 8,8,8,8,8,8,8,48,8,8,8,8,8,6,8,8,8,8,8,48,57,70,0,0,0};
const GFXglyph SmallGlyphs[] PROGMEM={{0,8,0,8,0,0},{0,8,10,8,0,-10},{10,8,10,8,0,-10},{20,9,10,8,0,-10},{32,8,13,8,
 0,-11},{45,9,10,8,0,-10},{57,9,10,8,0,-10},{69,8,10,8,0,-10},{79,8,12,8,0,-11},{91,8,12,8,0,-11},{103,8,10,8,0,
 -10},{113,8,8,8,0,-8},{121,8,4,8,0,-2},{125,8,4,8,0,-4},{129,8,2,8,0,-2},{131,8,12,8,0,-10},{143,8,10,8,0,-10},
 {153,8,10,8,0,-10},{163,8,10,8,0,-10},{173,8,10,8,0,-10},{183,8,10,8,0,-10},{193,8,10,8,0,-10},{203,8,10,8,0,-10},
 {213,8,10,8,0,-10},{223,8,10,8,0,-10},{233,8,10,8,0,-10},{243,8,7,8,0,-7},{250,8,9,8,0,-7},{259,8,8,8,0,-8},{267,8,
 7,8,0,-7},{274,8,8,8,0,-8},{282,8,10,8,0,-10},{292,9,12,8,0,-10},{306,9,10,8,0,-10},{318,8,10,8,0,-10},{328,8,10,8,
 0,-10},{338,8,10,8,0,-10},{348,8,10,8,0,-10},{358,8,10,8,0,-10},{368,8,10,8,0,-10},{378,8,10,8,0,-10},{388,8,10,8,
 0,-10},{398,8,10,8,0,-10},{408,9,10,8,0,-10},{420,8,10,8,0,-10},{430,8,10,8,0,-10},{440,8,10,8,0,-10},{450,8,10,8,
 0,-10},{460,8,10,8,0,-10},{470,8,12,8,0,-10},{482,9,10,8,0,-10},{494,8,10,8,0,-10},{504,9,10,8,0,-10},{516,8,10,8,
 0,-10},{526,9,10,8,0,-10},{538,9,10,8,0,-10},{550,9,10,8,0,-10},{562,9,10,8,0,-10},{574,8,10,8,0,-10},{584,8,12,8,
 0,-11},{596,8,12,8,0,-10},{608,8,12,8,0,-11},{620,8,10,8,0,-10},{630,9,3,8,0,0},{634,8,12,8,0,-12},{646,8,8,8,0,
 -8},{654,8,11,8,0,-11},{665,8,8,8,0,-8},{673,8,11,8,0,-11},{684,8,8,8,0,-8},{692,8,11,8,0,-11},{703,8,11,8,0,-8},
 {714,8,11,8,0,-11},{725,8,11,8,0,-11},{736,8,14,8,0,-11},{750,9,11,8,0,-11},{763,8,11,8,0,-11},{774,8,8,8,0,-8},
 {782,8,8,8,0,-8},{790,8,8,8,0,-8},{798,8,11,8,0,-8},{809,8,11,8,0,-8},{820,8,8,8,0,-8},{828,8,8,8,0,-8},{836,8,10,
 8,0,-10},{846,8,8,8,0,-8},{854,8,8,8,0,-8},{862,9,8,8,0,-8},{871,8,8,8,0,-8},{879,8,11,8,0,-8},{890,8,8,8,0,-8},
 {898,8,13,8,0,-11},{911,8,14,8,0,-11},{925,8,13,8,0,-11},{938,8,5,8,0,-5}};
const GFXfont SmallFont PROGMEM={(uint8_t*)SmallBits,(GFXglyph*)SmallGlyphs,32,126,17};
const uint8_t LabelBits[] PROGMEM={6,0,192,24,3,0,96,12,1,128,48,6,0,0,0,3,0,96,25,131,48,102,12,193,152,0,0,0,0,0,
 0,0,0,0,0,0,4,193,144,50,4,199,254,255,198,64,152,255,223,248,200,17,2,96,204,0,4,0,128,124,31,198,72,200,25,1,224,
 7,128,152,19,34,103,248,126,1,0,32,4,0,120,25,131,48,102,12,192,243,131,131,158,6,96,204,25,131,48,60,15,131,240,
 96,12,0,192,60,13,155,59,99,236,57,198,31,97,238,6,0,192,24,3,0,96,0,0,0,0,0,0,0,0,0,0,0,3,0,64,24,3,0,64,24,3,0,
 96,12,1,128,48,2,0,96,12,0,128,24,24,1,0,48,6,0,64,12,1,128,48,6,0,192,24,2,0,192,24,2,0,192,4,4,144,84,7,0,224,42,
 9,32,32,0,0,0,0,0,0,0,6,0,192,24,3,7,254,255,193,128,48,6,0,192,0,0,12,1,128,48,12,1,0,31,3,224,0,0,0,0,0,0,12,1,
 128,48,0,0,192,48,6,1,128,48,12,1,128,112,12,1,128,96,12,3,0,96,24,0,31,7,240,198,48,102,12,205,153,179,6,96,204,
 24,198,31,193,240,14,7,192,216,3,0,96,12,1,128,48,6,0,192,24,31,227,252,63,15,241,7,0,96,12,3,128,96,56,14,3,128,
 192,63,231,252,63,15,241,7,0,96,28,63,7,224,14,0,192,25,7,63,195,240,3,0,224,28,5,129,176,38,12,195,24,127,207,248,
 12,1,128,48,63,135,240,192,24,3,240,127,8,112,6,0,192,25,7,63,195,240,15,7,240,226,56,6,0,222,31,227,142,96,204,24,
 199,31,193,240,127,207,248,6,0,192,56,6,0,192,48,6,1,192,48,6,1,128,31,7,241,131,48,102,12,127,15,227,142,96,204,
 25,199,31,193,240,31,7,241,198,48,102,12,227,143,240,246,0,192,56,142,31,129,224,12,1,128,48,0,0,0,0,3,0,96,12,0,
 12,1,128,48,0,0,0,0,3,0,96,12,3,0,64,0,0,64,120,60,28,6,0,112,3,192,30,0,64,0,127,207,248,0,0,7,252,255,128,0,0,0,
 0,64,15,0,120,1,192,12,7,7,131,192,64,0,0,31,7,240,134,0,192,56,14,3,128,96,12,1,128,0,6,0,192,15,3,48,195,25,230,
 108,205,153,179,54,102,204,217,143,24,3,0,48,3,192,14,1,192,56,7,1,176,54,6,192,216,63,135,240,198,48,102,12,127,
 15,241,131,48,102,12,255,31,227,14,96,204,25,135,63,199,240,15,131,248,225,56,6,0,192,24,3,0,96,14,0,225,15,224,
 248,126,15,225,142,48,230,12,193,152,51,6,96,204,57,142,63,135,224,127,207,249,128,48,6,0,255,159,243,0,96,12,1,
 128,63,231,252,127,207,249,128,48,6,0,255,31,227,0,96,12,1,128,48,6,0,15,131,248,225,56,6,0,199,152,243,6,96,206,
 24,195,15,224,240,96,204,25,131,48,102,12,255,159,243,6,96,204,25,131,48,102,12,63,199,248,24,3,0,96,12,1,128,48,6,
 0,192,24,31,227,252,31,3,224,12,1,128,48,6,0,192,24,3,0,97,12,63,3,192,96,204,49,140,51,6,192,248,31,3,48,103,12,
 97,134,48,230,12,96,12,1,128,48,6,0,192,24,3,0,96,12,1,128,63,231,252,113,206,57,199,61,230,172,213,155,179,38,96,
 204,25,131,48,102,12,112,206,25,227,52,102,140,217,153,51,54,98,204,89,143,48,230,28,31,7,240,198,48,102,12,193,
 152,51,6,96,204,24,198,31,193,240,127,15,241,135,48,102,12,195,159,227,248,96,12,1,128,48,6,0,31,7,240,198,48,102,
 12,193,152,51,6,96,204,24,198,31,193,240,7,0,96,127,15,241,135,48,102,12,195,159,227,248,97,140,57,131,48,102,6,31,
 7,241,194,48,6,0,124,7,224,14,0,192,25,7,63,195,240,127,239,252,24,3,0,96,12,1,128,48,6,0,192,24,3,0,96,96,204,25,
 131,48,102,12,193,152,51,6,96,204,25,199,31,193,240,96,204,24,198,24,195,24,119,6,192,216,27,1,192,56,7,0,224,192,
 120,15,1,176,102,236,221,154,179,86,106,207,120,198,24,195,24,113,198,48,236,13,128,224,28,3,128,112,31,3,96,238,
 24,199,28,112,230,24,231,12,192,240,30,1,128,48,6,0,192,24,3,0,96,127,207,248,6,1,192,48,12,3,128,96,28,7,0,192,63,
 231,252,15,1,224,48,6,0,192,24,3,0,96,12,1,128,48,6,0,192,24,3,192,120,96,6,0,192,12,1,128,24,3,0,112,6,0,192,12,1,
 128,24,3,0,48,30,3,192,24,3,0,96,12,1,128,48,6,0,192,24,3,0,96,12,7,128,240,6,1,224,102,24,102,6,0,0,0,0,0,0,0,0,0,
 0,0,0,0,3,255,255,240,48,3,0,48,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,31,7,240,135,0,97,252,255,152,51,14,127,199,216,96,
 12,1,128,48,6,240,255,28,115,6,96,204,25,131,56,231,248,222,0,15,7,240,194,48,6,0,192,24,1,132,63,129,224,0,192,24,
 3,0,97,236,127,156,115,6,96,204,25,131,56,227,252,61,128,31,7,240,199,48,103,252,255,152,1,130,63,193,240,7,129,
 240,48,6,7,248,255,3,0,96,12,1,128,48,6,0,192,24,0,30,199,249,199,48,102,12,193,152,51,142,63,195,216,3,16,227,248,
 62,0,96,12,1,128,48,6,112,255,28,115,6,96,204,25,131,48,102,12,193,128,6,0,192,0,0,3,224,124,1,128,48,6,0,192,24,3,
 3,252,127,128,6,0,192,0,0,3,224,124,1,128,48,6,0,192,24,3,0,96,12,1,128,48,124,15,0,96,12,1,128,48,6,28,199,25,195,
 112,124,15,193,152,49,134,56,195,128,124,15,128,48,6,0,192,24,3,0,96,12,1,128,48,6,0,120,15,0,109,207,253,153,179,
 54,102,204,217,155,51,102,108,204,103,15,241,199,48,102,12,193,152,51,6,96,204,24,31,7,241,199,48,102,12,193,152,
 51,142,63,131,224,111,15,241,199,48,102,12,193,152,51,142,127,141,225,128,48,6,0,192,0,30,199,249,199,48,102,12,
 193,152,51,142,63,195,216,3,0,96,12,1,128,27,131,120,113,12,1,128,48,6,0,192,24,3,0,31,143,249,129,62,3,248,7,128,
 50,6,127,199,224,12,1,128,48,63,231,252,24,3,0,96,12,1,128,48,7,224,124,96,204,25,131,48,102,12,193,152,51,142,63,
 195,152,96,198,48,198,24,193,176,54,6,192,112,14,1,192,192,120,13,131,50,102,236,213,142,225,220,49,134,48,113,199,
 112,108,7,0,224,28,7,192,216,49,142,56,96,198,48,198,28,193,176,54,3,192,112,6,1,192,48,6,3,192,112,0,127,207,248,
 14,3,128,224,24,6,1,192,127,207,248,3,192,248,24,3,0,96,12,1,129,224,60,1,192,24,3,0,96,12,1,128,62,3,192,6,0,192,
 24,3,0,96,12,1,128,48,6,0,192,24,3,0,96,12,1,128,48,6,0,192,60,7,192,24,3,0,96,12,1,128,30,3,192,224,24,3,0,96,12,
 1,129,240,60,0,60,79,249,14,0,0,0,0,0,0};
const GFXglyph LabelGlyphs[] PROGMEM={{0,11,0,11,0,0},{0,11,13,11,0,-13},{18,11,13,11,0,-13},{36,11,14,11,0,-14},
 {56,11,17,11,0,-14},{80,11,13,11,0,-13},{98,11,13,11,0,-13},{116,11,13,11,0,-13},{134,11,16,11,0,-14},{156,11,16,
 11,0,-14},{178,11,13,11,0,-13},{196,11,11,11,0,-11},{212,11,5,11,0,-3},{219,11,6,11,0,-6},{228,11,3,11,0,-3},{233,
 11,15,11,0,-13},{254,11,13,11,0,-13},{272,11,13,11,0,-13},{290,11,13,11,0,-13},{308,11,13,11,0,-13},{326,11,13,11,
 0,-13},{344,11,13,11,0,-13},{362,11,13,11,0,-13},{380,11,13,11,0,-13},{398,11,13,11,0,-13},{416,11,13,11,0,-13},
 {434,11,9,11,0,-9},{447,11,11,11,0,-9},{463,11,10,11,0,-10},{477,11,9,11,0,-9},{490,11,10,11,0,-10},{504,11,13,11,
 0,-13},{522,11,15,11,0,-12},{543,11,13,11,0,-13},{561,11,13,11,0,-13},{579,11,13,11,0,-13},{597,11,13,11,0,-13},
 {615,11,13,11,0,-13},{633,11,13,11,0,-13},{651,11,13,11,0,-13},{669,11,13,11,0,-13},{687,11,13,11,0,-13},{705,11,
 13,11,0,-13},{723,11,13,11,0,-13},{741,11,13,11,0,-13},{759,11,13,11,0,-13},{777,11,13,11,0,-13},{795,11,13,11,0,
 -13},{813,11,13,11,0,-13},{831,11,15,11,0,-13},{852,11,13,11,0,-13},{870,11,13,11,0,-13},{888,11,13,11,0,-13},{906,
 11,13,11,0,-13},{924,11,13,11,0,-13},{942,11,13,11,0,-13},{960,11,13,11,0,-13},{978,11,13,11,0,-13},{996,11,13,11,
 0,-13},{1014,11,16,11,0,-14},{1036,11,15,11,0,-13},{1057,11,16,11,0,-14},{1079,11,13,11,0,-13},{1097,11,4,11,0,0},
 {1103,11,14,11,0,-14},{1123,11,10,11,0,-10},{1137,11,14,11,0,-14},{1157,11,10,11,0,-10},{1171,11,14,11,0,-14},
 {1191,11,10,11,0,-10},{1205,11,14,11,0,-14},{1225,11,14,11,0,-10},{1245,11,14,11,0,-14},{1265,11,14,11,0,-14},
 {1285,11,18,11,0,-14},{1310,11,14,11,0,-14},{1330,11,14,11,0,-14},{1350,11,10,11,0,-10},{1364,11,10,11,0,-10},
 {1378,11,10,11,0,-10},{1392,11,14,11,0,-10},{1412,11,14,11,0,-10},{1432,11,10,11,0,-10},{1446,11,10,11,0,-10},
 {1460,11,13,11,0,-13},{1478,11,10,11,0,-10},{1492,11,10,11,0,-10},{1506,11,10,11,0,-10},{1520,11,10,11,0,-10},
 {1534,11,14,11,0,-10},{1554,11,10,11,0,-10},{1568,11,17,11,0,-14},{1592,11,18,11,0,-14},{1617,11,17,11,0,-14},
 {1641,11,7,11,0,-7}};
const GFXfont LabelFont PROGMEM={(uint8_t*)LabelBits,(GFXglyph*)LabelGlyphs,32,126,21};
const uint8_t NumberBits[] PROGMEM={0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,
 128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,1,192,0,0,0,0,0,0,0,0,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,
 131,224,62,15,128,248,62,3,224,248,15,131,224,62,15,128,248,62,3,224,248,15,131,224,62,15,128,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,120,240,3,195,192,15,15,0,60,120,0,241,224,7,
 135,128,30,30,15,255,255,63,255,252,255,255,243,255,255,192,241,224,3,199,128,30,30,0,120,120,1,227,192,255,255,
 243,255,255,207,255,255,63,255,252,15,30,0,120,120,1,225,224,7,143,0,30,60,0,240,240,3,195,192,0,0,48,0,0,192,0,3,
 0,0,12,0,1,254,0,31,254,0,255,248,3,255,224,31,179,128,124,194,1,243,0,7,204,0,31,176,0,63,224,0,127,240,0,255,224,
 0,255,192,0,255,0,3,62,0,12,248,16,51,224,96,207,129,227,126,7,255,240,31,255,192,63,254,0,31,224,0,12,0,0,48,0,0,
 192,0,3,0,0,12,0,15,128,0,127,0,3,254,0,30,60,0,112,112,1,193,192,7,7,0,30,60,0,63,224,24,127,1,224,248,30,0,1,224,
 0,62,0,3,192,0,60,0,3,192,248,60,7,240,128,63,224,1,227,192,7,7,0,28,28,0,112,112,1,227,192,3,254,0,7,240,0,15,128,
 0,255,0,15,254,0,127,248,3,255,224,15,255,128,63,2,0,248,0,3,224,0,7,192,0,15,128,0,127,0,3,254,0,31,252,60,251,
 240,243,199,227,223,15,207,124,31,189,240,63,231,224,255,159,129,252,127,135,240,255,255,195,255,255,7,255,254,15,
 255,248,15,225,240,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,128,0,60,0,1,240,0,15,128,0,62,0,1,240,0,7,
 192,0,30,0,0,248,0,3,224,0,15,128,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,3,224,
 0,15,128,0,62,0,0,120,0,1,240,0,7,192,0,15,128,0,62,0,0,124,0,0,240,0,3,224,7,192,0,15,0,0,62,0,0,124,0,1,240,0,3,
 224,0,15,128,0,30,0,0,124,0,1,240,0,7,192,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,
 62,0,1,240,0,7,192,0,31,0,0,120,0,3,224,0,15,128,0,124,0,1,240,0,15,128,0,60,0,1,240,0,0,112,0,1,192,1,7,4,7,28,
 112,62,115,224,127,255,0,127,240,0,127,0,1,252,0,31,252,1,255,252,15,156,248,28,113,192,65,193,0,7,0,0,28,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,120,0,1,224,0,7,128,0,30,0,0,120,0,1,224,0,7,128,0,30,0,127,
 255,249,255,255,231,255,255,159,255,254,0,120,0,1,224,0,7,128,0,30,0,0,120,0,1,224,0,7,128,0,30,0,0,0,0,0,252,0,3,
 240,0,15,192,0,63,0,0,252,0,3,240,0,15,128,0,126,0,1,240,0,7,128,0,62,0,0,240,0,7,255,0,31,252,0,127,240,1,255,192,
 7,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,252,0,3,240,0,15,192,0,63,0,0,252,0,3,240,0,0,0,240,0,7,128,
 0,30,0,0,240,0,3,192,0,30,0,0,120,0,3,224,0,15,0,0,60,0,1,224,0,7,128,0,60,0,0,240,0,7,128,0,30,0,0,240,0,3,192,0,
 30,0,0,120,0,3,192,0,15,0,0,124,0,1,224,0,7,128,0,60,0,0,240,0,7,128,0,30,0,0,240,0,0,1,252,0,15,248,0,127,240,3,
 255,224,31,143,192,124,31,1,240,124,15,128,248,62,3,224,248,15,131,224,62,15,152,248,62,243,224,251,207,131,230,62,
 15,128,248,62,3,224,248,15,131,224,60,7,193,240,31,7,192,126,63,0,255,248,1,255,192,3,254,0,7,224,0,1,252,0,63,240,
 0,255,192,3,255,0,14,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,
 1,240,0,7,192,0,31,0,0,124,0,1,240,1,255,255,7,255,252,31,255,240,127,255,192,7,252,0,127,252,1,255,248,7,255,240,
 30,15,224,64,31,128,0,62,0,0,248,0,3,224,0,15,128,0,124,0,3,240,0,31,128,0,124,0,3,224,0,31,0,0,248,0,7,192,0,62,0,
 1,240,0,15,128,0,124,0,3,255,254,15,255,248,63,255,224,255,255,128,7,248,0,127,248,1,255,248,7,255,224,31,255,192,
 96,63,0,0,124,0,1,240,0,7,192,0,62,0,31,248,0,127,128,1,252,0,7,252,0,31,248,0,3,240,0,7,224,0,15,128,0,62,0,0,248,
 48,15,224,255,255,131,255,252,15,255,224,63,255,0,63,240,0,0,31,128,0,126,0,3,248,0,31,224,0,127,128,3,254,0,15,
 248,0,123,224,3,207,128,15,62,0,120,248,3,195,224,15,15,128,120,62,3,224,248,15,3,224,63,255,240,255,255,195,255,
 255,15,255,252,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,15,255,192,63,255,0,255,252,3,255,240,15,0,0,60,0,0,
 240,0,3,192,0,15,0,0,63,240,0,255,240,3,255,240,15,255,224,48,63,128,0,63,0,0,124,0,1,240,0,7,192,0,31,0,0,124,16,
 3,224,112,63,129,255,254,7,255,240,31,255,0,31,240,0,0,255,0,15,254,0,127,248,3,255,224,15,193,128,124,2,1,240,0,7,
 128,0,62,126,0,251,252,3,255,248,15,255,240,63,143,192,252,31,131,224,62,15,128,248,62,3,224,248,15,131,224,62,7,
 128,248,31,7,192,126,63,0,255,252,1,255,224,3,255,0,3,240,0,63,255,224,255,255,131,255,254,15,255,248,0,7,224,0,31,
 0,0,252,0,3,224,0,31,128,0,126,0,1,240,0,15,192,0,62,0,1,248,0,7,192,0,63,0,0,248,0,3,224,0,31,128,0,124,0,3,240,0,
 15,128,0,126,0,1,240,0,7,192,0,63,0,0,1,252,0,31,252,0,255,248,3,255,224,31,143,192,124,31,1,240,124,7,193,240,31,
 7,192,62,62,0,127,240,0,255,128,7,255,0,63,254,1,248,252,7,193,240,62,3,224,248,15,131,224,62,15,128,248,63,7,224,
 126,63,1,255,252,3,255,224,7,255,0,7,240,0,1,248,0,31,248,0,255,240,7,255,224,31,143,192,252,31,3,224,60,15,128,
 248,62,3,224,248,15,131,224,62,15,128,248,63,7,224,126,63,129,255,254,3,255,248,7,251,224,15,207,128,0,60,0,1,240,
 8,7,192,48,126,0,255,248,3,255,192,15,254,0,31,192,0,0,252,0,3,240,0,15,192,0,63,0,0,252,0,3,240,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,3,240,0,15,192,0,63,0,0,252,0,3,240,0,15,192,0,0,252,0,3,240,0,15,192,0,63,0,0,252,0,3,240,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,240,0,15,192,0,63,0,0,252,0,3,240,0,15,192,0,62,0,0,248,0,7,224,0,31,0,0,
 124,0,1,224,0,0,0,16,0,3,192,0,63,0,7,252,0,127,240,7,254,0,255,224,15,252,0,63,128,0,248,0,3,248,0,15,252,0,15,
 252,0,7,254,0,7,255,0,7,252,0,3,240,0,3,192,0,1,0,0,0,0,0,0,63,255,240,255,255,195,255,255,15,255,252,0,0,0,0,0,0,
 0,0,0,0,0,63,255,240,255,255,195,255,255,15,255,252,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0,0,240,0,3,240,0,15,248,0,63,
 248,0,31,248,0,31,252,0,15,252,0,7,240,0,7,192,0,127,0,15,252,0,255,192,31,248,3,255,128,15,248,0,63,0,0,240,0,2,0,
 0,0,0,0,0,0,0,0,254,0,31,254,0,255,252,3,255,240,15,7,224,48,15,128,128,62,0,0,248,0,7,224,0,63,0,1,248,0,15,192,0,
 126,0,1,240,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,0,0,0,0,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,0,254,0,15,
 254,0,127,252,3,224,248,31,1,224,248,3,195,192,15,30,15,188,120,127,241,227,255,207,15,31,60,124,124,241,224,243,
 199,131,207,30,15,60,120,60,241,224,243,199,131,207,31,31,62,60,124,120,255,241,225,255,199,195,239,15,0,0,62,0,0,
 124,1,0,252,14,1,255,248,1,255,224,1,254,0,0,254,0,3,248,0,15,224,0,127,192,1,255,0,7,188,0,62,248,0,251,224,3,239,
 128,15,30,0,124,124,1,241,240,7,199,192,62,15,128,248,62,3,255,248,15,255,224,127,255,193,255,255,7,192,124,31,1,
 240,248,3,227,224,15,143,128,62,126,0,253,240,1,240,63,254,0,255,254,3,255,252,15,255,240,62,7,224,248,15,131,224,
 62,15,128,248,62,3,224,248,31,3,255,252,15,255,192,63,255,0,255,255,3,224,126,15,128,248,62,1,240,248,7,195,224,31,
 15,128,124,62,3,240,248,31,195,255,254,15,255,248,63,255,128,255,248,0,0,63,192,3,255,128,31,254,0,255,248,7,255,
 224,63,131,128,252,2,3,224,0,31,128,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,126,0,0,248,0,3,
 240,8,15,224,224,31,255,128,63,254,0,255,248,0,255,224,0,255,0,63,240,0,255,248,3,255,240,15,255,224,63,255,192,
 248,63,131,224,126,15,128,248,62,3,240,248,7,195,224,31,15,128,124,62,1,240,248,7,195,224,31,15,128,124,62,1,240,
 248,15,195,224,62,15,129,248,62,15,224,255,255,3,255,248,15,255,192,63,254,0,255,192,0,31,255,224,127,255,129,255,
 254,7,255,248,31,255,224,124,0,1,240,0,7,192,0,31,0,0,124,0,1,255,252,7,255,240,31,255,192,127,255,1,255,252,7,192,
 0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,127,255,129,255,254,7,255,248,31,255,224,127,255,128,31,255,224,127,255,129,
 255,254,7,255,248,31,255,224,124,0,1,240,0,7,192,0,31,0,0,124,0,1,255,252,7,255,240,31,255,192,127,255,1,255,252,7,
 192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,0,0,127,0,7,255,0,63,254,1,255,248,15,
 255,224,127,3,129,248,6,7,192,0,63,0,0,248,0,3,224,0,15,131,252,62,15,240,248,63,195,224,255,15,131,252,62,0,240,
 252,3,193,240,15,7,224,60,31,224,240,63,255,192,127,255,0,255,252,1,255,192,1,252,0,62,3,224,248,15,131,224,62,15,
 128,248,62,3,224,248,15,131,224,62,15,128,248,62,3,224,248,15,131,255,254,15,255,248,63,255,224,255,255,131,255,
 254,15,128,248,62,3,224,248,15,131,224,62,15,128,248,62,3,224,248,15,131,224,62,15,128,248,62,3,224,248,15,128,31,
 255,192,127,255,1,255,252,7,255,240,31,255,192,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,
 15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,127,255,1,255,252,7,255,240,31,255,192,127,255,0,1,255,192,7,
 255,0,31,252,0,127,240,1,255,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,
 7,192,0,31,2,0,124,12,3,240,60,31,192,255,254,3,255,248,15,255,192,31,254,0,15,224,0,62,1,248,248,15,195,224,126,
 15,131,240,62,15,128,248,126,3,227,240,15,159,128,62,252,0,251,224,3,255,128,15,254,0,63,252,0,255,248,3,255,224,
 15,207,192,63,63,0,248,126,3,224,248,15,131,240,62,7,192,248,31,131,224,62,15,128,252,62,1,240,248,7,224,15,128,0,
 62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,
 128,0,62,0,0,248,0,3,224,0,15,128,0,63,255,192,255,255,3,255,252,15,255,240,63,255,192,63,3,240,252,15,195,240,127,
 15,225,252,63,135,240,254,31,195,252,255,15,115,188,61,206,240,247,251,195,223,239,15,63,60,60,252,240,243,243,195,
 207,207,15,30,60,60,0,240,240,3,195,192,15,15,0,60,60,0,240,240,3,195,192,15,15,0,60,60,0,240,240,3,192,63,3,224,
 252,15,131,248,62,15,224,248,63,131,224,255,15,131,252,62,15,240,248,63,227,224,251,143,131,238,62,15,188,248,62,
 115,224,249,207,131,231,190,15,142,248,62,59,224,248,255,131,225,254,15,135,248,62,31,224,248,63,131,224,254,15,
 131,248,62,7,224,248,31,128,0,252,0,15,252,0,127,248,3,255,240,15,255,192,126,31,129,240,62,7,192,248,62,1,240,248,
 7,195,224,31,15,128,124,62,1,240,248,7,195,224,31,15,128,124,62,1,240,248,7,193,240,62,7,192,248,31,135,224,63,255,
 0,255,252,1,255,224,3,255,0,3,240,0,31,254,0,127,254,1,255,252,7,255,248,31,255,224,124,15,193,240,31,7,192,124,31,
 1,240,124,7,193,240,63,7,255,248,31,255,224,127,255,1,255,248,7,255,128,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,
 1,240,0,7,192,0,31,0,0,124,0,0,0,252,0,15,252,0,127,248,3,255,240,15,255,192,126,31,129,240,62,7,192,248,62,1,240,
 248,7,195,224,31,15,128,124,62,1,240,248,7,195,224,31,15,128,124,62,1,240,248,7,195,224,31,7,192,248,31,3,224,126,
 31,128,255,252,3,255,240,7,255,128,15,252,0,15,248,0,3,240,0,7,224,0,15,0,0,16,0,63,252,0,255,252,3,255,252,15,255,
 240,63,255,224,248,31,131,224,62,15,128,248,62,3,224,248,15,131,224,124,15,255,240,63,255,128,255,248,3,255,240,15,
 255,192,62,63,128,248,126,3,224,252,15,129,240,62,7,192,248,31,131,224,62,15,128,252,62,1,240,248,7,224,1,252,0,31,
 254,0,255,252,7,255,240,63,255,192,252,15,3,224,12,15,128,16,63,0,0,255,0,1,255,128,7,255,192,7,255,128,15,255,0,7,
 252,0,3,248,0,7,224,0,15,130,0,62,12,0,248,62,15,224,255,255,131,255,252,15,255,224,31,255,0,15,240,0,127,255,241,
 255,255,199,255,255,31,255,252,127,255,240,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,
 128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,62,1,240,248,7,195,224,31,
 15,128,124,62,1,240,248,7,195,224,31,15,128,124,62,1,240,248,7,195,224,31,15,128,124,62,1,240,248,7,195,224,31,15,
 128,124,62,1,240,248,7,195,224,31,15,192,252,31,135,224,127,255,129,255,254,3,255,240,7,255,128,7,248,0,124,0,249,
 248,7,227,224,31,15,128,124,62,1,240,248,7,193,240,62,7,192,248,31,3,224,124,15,128,248,124,3,225,240,15,135,192,
 62,31,0,120,120,1,243,224,7,207,128,31,62,0,60,248,0,243,192,3,255,0,15,252,0,63,240,0,127,128,1,254,0,7,248,0,248,
 0,127,224,1,255,128,7,254,0,31,120,0,121,224,1,231,128,7,158,30,30,120,124,121,243,241,231,207,199,159,63,30,60,
 252,248,243,251,195,223,239,15,115,188,61,206,240,247,63,195,220,255,15,241,252,31,135,240,126,31,129,248,126,7,
 225,248,31,3,224,124,15,128,252,1,249,240,7,195,224,63,15,193,248,31,7,192,126,63,0,248,248,1,247,224,7,255,0,15,
 248,0,63,224,0,127,0,0,248,0,7,240,0,31,192,0,255,128,3,254,0,31,124,0,253,248,3,227,224,31,143,192,124,31,3,224,
 62,31,128,252,124,1,243,240,7,224,252,1,249,240,7,199,224,63,15,128,248,63,7,224,124,31,1,248,252,3,227,224,15,223,
 128,31,124,0,127,240,0,255,128,3,254,0,7,240,0,31,192,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,
 0,62,0,0,248,0,3,224,0,63,255,240,255,255,195,255,255,15,255,252,63,255,240,0,31,128,0,252,0,3,240,0,31,128,0,252,
 0,3,240,0,31,128,0,252,0,3,240,0,31,128,0,252,0,3,224,0,31,128,0,252,0,3,224,0,31,128,0,255,255,195,255,255,15,255,
 252,63,255,240,255,255,192,1,255,0,7,252,0,31,240,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,
 0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,
 124,0,1,240,0,7,252,0,31,240,0,127,192,60,0,0,120,0,1,224,0,3,192,0,15,0,0,30,0,0,120,0,1,224,0,3,192,0,15,0,0,30,
 0,0,120,0,0,240,0,3,192,0,7,128,0,30,0,0,60,0,0,240,0,1,224,0,7,128,0,15,0,0,60,0,0,120,0,1,224,0,7,128,0,15,0,0,
 60,0,0,120,0,1,224,0,3,192,3,254,0,15,248,0,63,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,
 15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,
 224,0,15,128,0,62,0,15,248,0,63,224,0,255,128,0,252,0,3,240,0,31,224,0,255,192,7,255,128,31,62,0,248,124,7,192,248,
 62,1,241,240,3,224,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,3,255,255,255,255,255,255,255,255,15,128,0,31,0,0,62,0,0,124,0,0,240,0,1,224,0,3,192,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,
 254,0,63,255,0,255,254,3,255,248,12,3,224,0,7,192,0,31,0,127,252,15,255,240,127,255,193,255,255,15,192,124,62,1,
 240,248,7,195,224,63,15,193,252,31,255,240,127,255,192,255,223,0,252,124,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,
 1,240,0,7,199,192,31,127,128,127,255,1,255,254,7,241,248,31,131,224,126,15,193,240,31,7,192,124,31,1,240,124,7,193,
 240,31,7,192,124,31,131,240,126,15,129,252,126,7,255,248,31,255,192,125,254,1,241,240,0,0,127,0,7,255,0,63,254,1,
 255,248,15,224,224,63,0,129,248,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,126,0,0,248,2,3,248,56,7,255,224,15,
 255,128,31,252,0,31,192,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,248,248,7,251,224,63,255,129,255,254,
 7,227,248,31,7,224,248,31,131,224,62,15,128,248,62,3,224,248,15,131,224,62,15,128,248,63,7,224,252,31,129,248,254,
 7,255,248,15,255,224,31,239,128,62,62,0,0,254,0,15,254,0,127,252,3,255,248,31,131,240,124,7,195,224,15,143,128,62,
 63,255,248,255,255,227,255,255,143,255,254,62,0,0,248,0,1,240,0,7,224,12,15,255,240,31,255,192,63,255,0,63,240,0,
 31,224,1,255,128,7,254,0,63,248,0,248,0,3,224,0,15,128,7,255,248,31,255,224,127,255,129,255,254,0,62,0,0,248,0,3,
 224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,1,243,224,
 31,239,128,255,254,7,255,248,31,143,224,124,31,131,240,126,15,128,248,62,3,224,248,15,131,224,62,15,128,248,62,3,
 224,252,31,129,240,126,7,227,248,31,255,224,63,255,128,127,190,0,252,248,0,3,224,0,15,128,192,252,3,255,240,15,255,
 128,63,252,0,63,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,199,192,31,127,128,127,255,1,255,252,7,
 241,248,31,131,224,124,15,129,240,62,7,192,248,31,3,224,124,15,129,240,62,7,192,248,31,3,224,124,15,129,240,62,7,
 192,248,31,3,224,124,15,129,240,62,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,0,0,0,0,0,0,0,0,63,240,0,255,192,
 3,255,0,15,252,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,127,255,
 193,255,255,7,255,252,31,255,240,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,0,0,0,0,0,0,0,0,31,248,0,127,224,1,
 255,128,7,254,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,
 3,224,0,15,128,0,62,0,0,248,0,3,224,0,31,128,31,252,0,127,240,1,255,128,7,248,0,31,0,0,124,0,1,240,0,7,192,0,31,0,
 0,124,0,1,240,0,7,192,252,31,7,224,124,63,1,241,248,7,207,192,31,126,0,127,240,1,255,128,7,255,0,31,252,0,127,248,
 1,251,240,7,199,192,31,31,128,124,63,1,240,124,7,193,248,31,3,224,124,15,193,240,31,128,63,240,0,255,192,3,255,0,
 15,252,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,
 0,0,124,0,1,240,0,7,192,0,31,128,0,63,248,0,255,224,1,255,128,3,254,0,121,227,225,239,223,199,255,255,31,255,254,
 124,252,249,225,225,231,135,135,158,30,30,120,120,121,225,225,231,135,135,158,30,30,120,120,121,225,225,231,135,
 135,158,30,30,120,120,121,225,225,231,135,135,158,30,30,31,31,0,125,254,1,255,252,7,255,240,31,199,224,126,15,129,
 240,62,7,192,248,31,3,224,124,15,129,240,62,7,192,248,31,3,224,124,15,129,240,62,7,192,248,31,3,224,124,15,129,240,
 62,7,192,248,0,252,0,15,252,0,127,248,3,255,240,31,135,224,124,15,131,240,63,15,128,124,62,1,240,248,7,195,224,31,
 15,128,124,62,1,240,252,15,193,240,62,7,225,248,15,255,192,31,254,0,63,240,0,63,0,31,31,0,125,254,1,255,252,7,255,
 248,31,199,224,126,15,129,248,63,7,192,124,31,1,240,124,7,193,240,31,7,192,124,31,1,240,126,15,193,248,62,7,241,
 248,31,255,224,127,255,1,247,248,7,199,192,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,0,3,227,224,31,239,
 128,255,254,7,255,248,31,143,224,124,31,131,224,126,15,128,248,62,3,224,248,15,131,224,62,15,128,248,62,3,224,252,
 31,131,240,126,7,227,248,31,255,224,63,255,128,127,190,0,248,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,128,0,
 62,0,7,199,224,31,127,192,127,255,1,255,252,7,248,48,31,128,64,126,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,
 31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,1,255,0,31,255,0,255,252,7,255,240,31,128,192,124,0,1,240,0,7,
 240,0,15,252,0,63,254,0,127,252,0,63,240,0,15,224,0,15,128,0,62,6,1,248,31,255,224,127,255,1,255,248,1,255,128,1,
 240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,3,255,254,15,255,248,63,255,224,255,255,128,31,0,0,124,0,1,240,0,7,192,0,
 31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,224,0,31,254,0,63,248,0,255,224,0,255,128,31,3,224,124,15,129,
 240,62,7,192,248,31,3,224,124,15,129,240,62,7,192,248,31,3,224,124,15,129,240,62,7,192,248,31,3,224,124,15,129,240,
 126,7,227,248,15,255,224,63,255,128,127,190,0,248,248,124,1,241,248,15,195,224,62,15,128,248,62,3,224,124,31,1,240,
 124,7,193,240,15,143,128,62,62,0,248,248,1,227,192,7,223,0,31,124,0,61,224,0,247,128,3,254,0,7,248,0,31,192,0,127,
 0,240,0,63,224,1,255,128,7,222,0,30,120,0,121,225,225,231,135,135,158,63,30,60,252,240,243,243,195,207,207,15,63,
 60,61,206,240,247,59,193,252,254,7,243,248,31,135,224,126,31,129,248,126,3,225,240,63,3,240,126,31,128,248,124,3,
 243,240,7,207,128,15,252,0,63,240,0,127,128,0,252,0,3,240,0,15,192,0,127,128,3,255,0,15,252,0,124,248,3,243,240,31,
 135,224,126,31,131,240,63,31,128,126,126,1,248,248,7,195,224,31,15,192,252,31,3,224,124,15,128,248,124,3,225,240,
 15,143,192,31,62,0,124,248,1,247,224,3,255,0,15,252,0,31,224,0,127,128,1,254,0,3,240,0,15,192,0,62,0,0,248,0,3,224,
 0,31,0,15,252,0,63,224,0,255,0,3,248,0,0,31,255,224,127,255,129,255,254,7,255,248,0,7,224,0,63,0,1,248,0,15,192,0,
 126,0,3,240,0,15,192,0,126,0,3,240,0,31,128,0,252,0,7,224,0,31,255,224,127,255,129,255,254,7,255,248,0,15,224,0,
 255,128,3,254,0,31,128,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,3,240,0,31,128,3,254,0,15,
 192,0,63,224,0,31,128,0,63,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,1,248,0,3,254,0,15,
 248,0,15,224,0,120,0,1,224,0,7,128,0,30,0,0,120,0,1,224,0,7,128,0,30,0,0,120,0,1,224,0,7,128,0,30,0,0,120,0,1,224,
 0,7,128,0,30,0,0,120,0,1,224,0,7,128,0,30,0,0,120,0,1,224,0,7,128,0,30,0,0,120,0,1,224,0,7,128,0,30,0,0,120,0,1,
 224,0,7,128,0,30,0,0,120,0,1,224,0,7,128,0,30,0,31,192,0,127,192,1,255,0,0,126,0,0,248,0,3,224,0,15,128,0,62,0,0,
 248,0,3,224,0,15,128,0,62,0,0,248,0,3,240,0,7,224,0,31,240,0,15,192,1,255,0,7,224,0,63,0,0,248,0,3,224,0,15,128,0,
 62,0,0,248,0,3,224,0,15,128,0,62,0,0,248,0,7,224,1,255,0,7,252,0,31,192,0,15,192,16,127,224,195,255,255,15,255,252,
 48,63,224,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const GFXglyph NumberGlyphs[] PROGMEM={{0,22,0,22,0,0},{0,22,26,22,0,-26},{72,22,26,22,0,-26},{144,22,27,22,0,-27},
 {219,22,32,22,0,-27},{307,22,26,22,0,-26},{379,22,26,22,0,-26},{451,22,26,22,0,-26},{523,22,32,22,0,-27},{611,22,
 32,22,0,-27},{699,22,26,22,0,-26},{771,22,21,22,0,-21},{829,22,12,22,0,-6},{862,22,13,22,0,-13},{898,22,6,22,0,-6},
 {915,22,30,22,0,-26},{998,22,26,22,0,-26},{1070,22,26,22,0,-26},{1142,22,26,22,0,-26},{1214,22,26,22,0,-26},{1286,
 22,26,22,0,-26},{1358,22,26,22,0,-26},{1430,22,26,22,0,-26},{1502,22,26,22,0,-26},{1574,22,26,22,0,-26},{1646,22,
 26,22,0,-26},{1718,22,19,22,0,-19},{1771,22,25,22,0,-19},{1840,22,21,22,0,-21},{1898,22,17,22,0,-17},{1945,22,21,
 22,0,-21},{2003,22,26,22,0,-26},{2075,22,30,22,0,-25},{2158,22,26,22,0,-26},{2230,22,26,22,0,-26},{2302,22,26,22,0,
 -26},{2374,22,26,22,0,-26},{2446,22,26,22,0,-26},{2518,22,26,22,0,-26},{2590,22,26,22,0,-26},{2662,22,26,22,0,-26},
 {2734,22,26,22,0,-26},{2806,22,26,22,0,-26},{2878,22,26,22,0,-26},{2950,22,26,22,0,-26},{3022,22,26,22,0,-26},
 {3094,22,26,22,0,-26},{3166,22,26,22,0,-26},{3238,22,26,22,0,-26},{3310,22,31,22,0,-26},{3396,22,26,22,0,-26},
 {3468,22,26,22,0,-26},{3540,22,26,22,0,-26},{3612,22,26,22,0,-26},{3684,22,26,22,0,-26},{3756,22,26,22,0,-26},
 {3828,22,26,22,0,-26},{3900,22,26,22,0,-26},{3972,22,26,22,0,-26},{4044,22,32,22,0,-27},{4132,22,30,22,0,-26},
 {4215,22,32,22,0,-27},{4303,22,26,22,0,-26},{4375,22,8,22,0,0},{4397,22,29,22,0,-29},{4477,22,20,22,0,-20},{4532,
 22,27,22,0,-27},{4607,22,20,22,0,-20},{4662,22,27,22,0,-27},{4737,22,20,22,0,-20},{4792,22,27,22,0,-27},{4867,22,
 27,22,0,-20},{4942,22,27,22,0,-27},{5017,22,29,22,0,-29},{5097,22,36,22,0,-29},{5196,22,27,22,0,-27},{5271,22,27,
 22,0,-27},{5346,22,20,22,0,-20},{5401,22,20,22,0,-20},{5456,22,20,22,0,-20},{5511,22,27,22,0,-20},{5586,22,27,22,0,
 -20},{5661,22,20,22,0,-20},{5716,22,20,22,0,-20},{5771,22,26,22,0,-26},{5843,22,20,22,0,-20},{5898,22,20,22,0,-20},
 {5953,22,20,22,0,-20},{6008,22,20,22,0,-20},{6063,22,27,22,0,-20},{6138,22,20,22,0,-20},{6193,22,33,22,0,-27},
 {6284,22,36,22,0,-27},{6383,22,33,22,0,-27},{6474,22,15,22,0,-15}};
const GFXfont NumberFont PROGMEM={(uint8_t*)NumberBits,(GFXglyph*)NumberGlyphs,32,126,39};
const uint8_t BigBits[] PROGMEM={0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,
 128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,
 0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,
 3,252,0,0,0,3,248,0,0,0,7,224,0,0,0,15,192,0,0,0,31,128,0,0,0,63,0,0,0,0,126,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,
 63,192,0,0,0,127,128,0,0,1,254,1,254,0,3,252,3,252,0,7,248,7,248,0,15,240,15,240,0,31,224,31,224,0,63,192,63,192,0,
 127,128,127,128,0,255,0,255,0,1,254,1,254,0,3,252,3,252,0,7,248,7,248,0,15,240,15,240,0,31,224,31,224,0,63,192,63,
 192,0,127,128,127,128,0,255,0,255,0,1,254,1,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,1,252,15,224,0,3,240,31,192,0,15,224,127,0,0,31,192,254,0,0,63,129,252,0,0,126,3,248,0,1,252,15,224,0,3,248,31,
 192,0,7,240,63,128,0,15,224,127,0,0,63,128,252,0,0,127,3,248,0,255,255,255,255,193,255,255,255,255,131,255,255,255,
 255,7,255,255,255,254,15,255,255,255,252,31,255,255,255,248,63,255,255,255,240,1,252,15,224,0,3,248,31,192,0,7,240,
 63,128,0,31,192,127,0,0,63,129,252,0,0,127,3,248,0,0,254,7,240,0,3,248,15,192,0,7,240,63,128,15,255,255,255,252,31,
 255,255,255,248,63,255,255,255,240,127,255,255,255,224,255,255,255,255,193,255,255,255,255,131,255,255,255,255,0,
 31,192,254,0,0,63,129,252,0,0,127,3,248,0,0,252,7,240,0,3,248,31,192,0,7,240,63,128,0,15,224,127,0,0,31,128,254,0,
 0,127,1,248,0,0,254,7,240,0,1,252,15,224,0,3,240,31,192,0,0,0,0,120,0,0,0,0,240,0,0,0,1,224,0,0,0,3,192,0,0,0,7,
 128,0,0,0,15,0,0,0,0,30,0,0,0,1,255,192,0,0,31,255,248,0,0,255,255,248,0,3,255,255,240,0,15,255,255,224,0,31,255,
 255,192,0,127,255,255,128,0,255,158,63,0,3,254,60,14,0,7,248,120,4,0,15,240,240,0,0,31,225,224,0,0,63,195,192,0,0,
 127,199,128,0,0,255,143,0,0,1,255,222,0,0,1,255,252,0,0,3,255,255,128,0,3,255,255,192,0,3,255,255,224,0,3,255,255,
 224,0,3,255,255,224,0,1,255,255,192,0,0,63,255,192,0,0,63,255,128,0,0,121,255,128,0,0,241,255,0,0,1,225,254,0,0,3,
 195,252,0,0,7,135,248,0,128,15,15,240,1,192,30,31,224,3,224,60,127,192,7,248,121,255,0,15,255,255,254,0,31,255,255,
 248,0,63,255,255,240,0,127,255,255,192,0,127,255,254,0,0,31,255,248,0,0,3,255,192,0,0,0,120,0,0,0,0,240,0,0,0,1,
 224,0,0,0,3,192,0,0,0,7,128,0,0,0,15,0,0,0,0,30,0,0,0,0,60,0,0,0,0,120,0,0,0,252,0,0,0,7,254,0,0,0,63,255,0,0,0,
 255,255,0,0,1,255,254,0,0,7,240,254,0,0,15,192,252,0,0,63,1,252,0,0,126,1,248,0,0,252,3,240,0,1,248,7,224,0,3,240,
 15,192,0,7,240,63,128,0,7,224,126,0,0,15,225,252,0,0,15,255,240,0,48,31,255,224,1,240,31,255,128,31,224,15,252,1,
 255,0,7,224,15,240,0,0,0,255,128,0,0,7,248,0,0,0,127,128,0,0,7,252,0,0,0,63,192,0,0,3,252,0,252,0,63,224,7,254,0,
 254,0,63,255,1,224,0,255,255,1,0,1,255,254,0,0,7,240,254,0,0,15,192,252,0,0,63,1,252,0,0,126,1,248,0,0,252,3,240,0,
 1,248,7,224,0,3,240,15,192,0,7,240,63,128,0,7,224,126,0,0,15,225,252,0,0,15,255,240,0,0,31,255,224,0,0,31,255,128,
 0,0,15,252,0,0,0,7,224,0,0,1,255,224,0,0,31,255,240,0,0,127,255,224,0,1,255,255,192,0,7,255,255,128,0,31,255,255,0,
 0,63,255,254,0,0,255,255,252,0,1,255,192,24,0,3,255,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,240,0,0,0,127,
 224,0,0,0,127,224,0,0,0,255,192,0,0,1,255,192,0,0,1,255,192,0,0,3,255,128,0,0,15,255,128,0,0,63,255,128,0,0,255,
 255,0,0,3,255,255,0,0,15,255,254,0,0,63,255,254,3,248,127,199,254,7,241,255,15,252,15,227,252,15,252,31,199,248,15,
 252,63,159,224,31,248,127,63,192,31,248,254,127,128,31,251,248,255,0,63,247,241,254,0,63,255,227,254,0,63,255,135,
 252,0,63,255,15,252,0,127,252,15,252,0,127,248,31,252,0,255,224,63,254,7,255,192,63,255,255,255,192,127,255,255,
 255,128,127,255,255,255,128,127,255,255,255,128,127,255,255,255,0,127,255,243,255,0,63,255,131,255,0,15,248,0,0,0,
 1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,
 252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,248,0,0,0,15,224,0,0,0,63,192,0,0,0,127,0,0,0,1,254,0,0,0,7,
 248,0,0,0,15,240,0,0,0,63,192,0,0,0,127,128,0,0,1,255,0,0,0,3,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,192,0,0,0,255,
 128,0,0,1,255,0,0,0,3,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,0,0,0,3,254,0,0,0,7,252,0,
 0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,
 0,0,31,240,0,0,0,63,224,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,3,252,0,0,0,7,252,0,0,
 0,15,248,0,0,0,15,240,0,0,0,31,240,0,0,0,63,224,0,0,0,63,192,0,0,0,127,192,0,0,0,127,128,0,0,0,255,0,0,0,0,255,0,0,
 0,1,254,0,0,0,1,254,0,0,0,1,252,0,0,0,3,252,0,0,0,3,248,0,0,0,3,248,0,0,63,128,0,0,0,63,128,0,0,0,127,128,0,0,0,
 127,0,0,0,0,255,0,0,0,0,255,0,0,0,1,254,0,0,0,1,254,0,0,0,3,252,0,0,0,7,252,0,0,0,7,248,0,0,0,15,248,0,0,0,31,240,
 0,0,0,31,224,0,0,0,63,224,0,0,0,127,192,0,0,0,127,128,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,
 0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,
 0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,
 0,0,0,127,128,0,0,1,255,0,0,0,3,254,0,0,0,7,248,0,0,0,31,240,0,0,0,63,192,0,0,0,127,128,0,0,1,255,0,0,0,3,252,0,0,
 0,7,248,0,0,0,31,224,0,0,0,63,128,0,0,0,255,0,0,0,1,252,0,0,0,7,248,0,0,0,15,224,0,0,0,63,128,0,0,0,0,124,0,0,0,0,
 248,0,0,0,1,240,0,0,0,3,224,0,0,0,7,192,0,0,64,15,128,16,0,224,31,0,224,3,240,62,7,224,7,248,124,63,192,31,252,249,
 255,192,15,253,247,254,0,7,255,255,240,0,3,255,255,128,0,1,255,252,0,0,0,255,224,0,0,1,255,192,0,0,15,255,224,0,0,
 127,255,240,0,3,255,255,248,0,31,251,239,252,0,255,231,207,254,0,255,15,135,248,1,248,31,3,240,1,192,62,1,192,2,0,
 124,0,128,0,0,248,0,0,0,1,240,0,0,0,3,224,0,0,0,7,192,0,0,0,15,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,254,0,0,0,1,252,0,0,0,3,248,0,0,0,7,240,0,0,0,15,224,0,0,0,31,192,0,0,0,63,128,0,0,0,127,0,0,0,
 0,254,0,0,0,1,252,0,0,0,3,248,0,0,0,7,240,0,0,0,15,224,0,7,255,255,255,254,15,255,255,255,252,31,255,255,255,248,
 63,255,255,255,240,127,255,255,255,224,255,255,255,255,193,255,255,255,255,128,0,15,224,0,0,0,31,192,0,0,0,63,128,
 0,0,0,127,0,0,0,0,254,0,0,0,1,252,0,0,0,3,248,0,0,0,7,240,0,0,0,15,224,0,0,0,31,192,0,0,0,63,128,0,0,0,127,0,0,0,0,
 254,0,0,0,1,252,0,0,0,0,0,0,0,0,0,0,0,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,0,0,31,248,0,0,0,63,240,0,0,0,127,224,0,
 0,0,255,192,0,0,1,255,128,0,0,3,255,0,0,0,7,252,0,0,0,31,248,0,0,0,63,224,0,0,0,127,192,0,0,0,255,0,0,0,3,254,0,0,
 0,7,248,0,0,0,15,240,0,0,0,31,192,0,0,0,127,128,0,0,0,254,0,0,0,0,127,255,248,0,0,255,255,240,0,1,255,255,224,0,3,
 255,255,192,0,7,255,255,128,0,15,255,255,0,0,31,255,254,0,0,63,255,252,0,0,127,255,248,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 3,255,0,0,0,7,254,0,0,0,15,252,0,0,0,31,248,0,0,0,63,240,0,0,0,127,224,0,0,0,255,192,0,0,1,255,128,0,0,3,255,0,0,0,
 7,254,0,0,0,15,252,0,0,0,0,0,7,240,0,0,0,31,192,0,0,0,63,128,0,0,0,254,0,0,0,1,252,0,0,0,7,240,0,0,0,15,224,0,0,0,
 63,128,0,0,0,127,0,0,0,0,254,0,0,0,3,248,0,0,0,7,240,0,0,0,31,192,0,0,0,63,128,0,0,0,254,0,0,0,1,252,0,0,0,7,240,0,
 0,0,15,224,0,0,0,63,128,0,0,0,127,0,0,0,1,252,0,0,0,3,248,0,0,0,15,224,0,0,0,31,192,0,0,0,127,0,0,0,0,254,0,0,0,3,
 252,0,0,0,7,240,0,0,0,15,224,0,0,0,63,128,0,0,0,127,0,0,0,1,252,0,0,0,3,248,0,0,0,15,224,0,0,0,31,192,0,0,0,127,0,
 0,0,0,254,0,0,0,3,248,0,0,0,7,240,0,0,0,31,192,0,0,0,63,128,0,0,0,254,0,0,0,1,252,0,0,0,7,240,0,0,0,15,224,0,0,0,
 31,192,0,0,0,127,0,0,0,0,254,0,0,0,3,248,0,0,0,7,240,0,0,0,31,192,0,0,0,63,128,0,0,0,254,0,0,0,0,0,1,255,0,0,0,15,
 255,128,0,0,127,255,192,0,1,255,255,192,0,7,255,255,192,0,31,255,255,192,0,63,255,255,128,0,255,255,255,128,1,255,
 131,255,0,7,254,3,255,0,15,248,3,254,0,31,240,7,252,0,127,192,7,252,0,255,128,15,248,1,255,0,31,240,3,254,0,63,224,
 15,248,0,63,224,31,240,0,127,192,63,224,0,255,128,127,192,1,255,0,255,128,3,254,1,255,7,135,252,3,254,31,143,248,7,
 252,127,159,240,15,248,255,63,224,31,241,254,127,192,63,227,252,255,128,127,195,241,255,0,255,131,195,254,1,255,0,
 7,252,3,254,0,15,248,7,252,0,31,240,15,248,0,63,192,15,248,0,255,128,31,240,1,255,0,63,224,3,254,0,127,192,7,252,0,
 255,192,31,240,0,255,128,63,224,1,255,128,255,192,1,255,131,255,0,3,255,255,254,0,3,255,255,248,0,7,255,255,240,0,
 7,255,255,192,0,7,255,255,0,0,7,255,252,0,0,3,255,224,0,0,1,255,0,0,0,7,255,128,0,1,255,255,0,0,15,255,254,0,0,31,
 255,252,0,0,63,255,248,0,0,127,255,240,0,0,255,255,224,0,1,255,255,192,0,3,252,255,128,0,7,1,255,0,0,0,3,254,0,0,0,
 7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,
 15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,
 31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,
 63,224,0,1,255,255,255,240,3,255,255,255,224,7,255,255,255,192,15,255,255,255,128,31,255,255,255,0,63,255,255,254,
 0,127,255,255,252,0,255,255,255,248,0,0,7,254,0,0,0,255,255,128,0,15,255,255,192,0,63,255,255,224,0,127,255,255,
 224,0,255,255,255,224,1,255,255,255,192,3,255,255,255,192,7,252,7,255,128,15,128,3,255,128,28,0,3,255,0,32,0,7,254,
 0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,1,255,0,0,0,3,254,0,0,0,15,252,0,0,0,31,240,0,
 0,0,127,224,0,0,1,255,128,0,0,7,254,0,0,0,31,248,0,0,0,127,224,0,0,1,255,128,0,0,7,255,0,0,0,31,252,0,0,0,63,240,0,
 0,0,255,192,0,0,3,255,0,0,0,15,252,0,0,0,63,240,0,0,0,255,192,0,0,3,255,0,0,0,15,252,0,0,0,63,240,0,0,0,255,192,0,
 0,3,255,0,0,0,15,255,255,255,192,31,255,255,255,128,63,255,255,255,0,127,255,255,254,0,255,255,255,252,1,255,255,
 255,248,3,255,255,255,240,7,255,255,255,224,0,31,254,0,0,3,255,255,128,0,31,255,255,192,0,63,255,255,192,0,127,255,
 255,192,0,255,255,255,192,1,255,255,255,192,3,255,255,255,128,7,224,15,255,0,12,0,7,255,0,0,0,7,254,0,0,0,7,252,0,
 0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,1,255,0,0,0,7,254,0,0,0,63,248,0,0,127,255,224,0,0,255,255,
 128,0,1,255,254,0,0,3,255,224,0,0,7,255,240,0,0,15,255,248,0,0,31,255,248,0,0,63,255,248,0,0,0,127,248,0,0,0,63,
 248,0,0,0,63,240,0,0,0,63,224,0,0,0,127,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,31,
 248,2,0,0,63,240,7,128,0,255,192,15,240,7,255,128,31,255,255,255,0,63,255,255,252,0,127,255,255,248,0,255,255,255,
 224,1,255,255,255,128,3,255,255,252,0,1,255,255,224,0,0,31,254,0,0,0,0,15,252,0,0,0,31,248,0,0,0,127,240,0,0,1,255,
 224,0,0,3,255,192,0,0,15,255,128,0,0,63,255,0,0,0,127,254,0,0,1,255,252,0,0,3,255,248,0,0,15,255,240,0,0,63,191,
 224,0,0,127,127,192,0,1,252,255,128,0,7,241,255,0,0,15,227,254,0,0,63,135,252,0,0,127,15,248,0,1,252,31,240,0,7,
 240,63,224,0,15,224,127,192,0,63,128,255,128,0,255,1,255,0,1,252,3,254,0,7,240,7,252,0,15,224,15,248,0,63,128,31,
 240,0,254,0,63,224,1,252,0,127,192,3,255,255,255,252,7,255,255,255,248,15,255,255,255,240,31,255,255,255,224,63,
 255,255,255,192,127,255,255,255,128,255,255,255,255,1,255,255,255,254,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,
 252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,3,255,255,254,0,7,255,255,252,
 0,15,255,255,248,0,31,255,255,240,0,63,255,255,224,0,127,255,255,192,0,255,255,255,128,1,255,255,255,0,3,248,0,0,0,
 7,240,0,0,0,15,224,0,0,0,31,192,0,0,0,63,128,0,0,0,127,0,0,0,0,254,0,0,0,1,252,0,0,0,3,251,254,0,0,7,255,255,128,0,
 15,255,255,128,0,31,255,255,192,0,63,255,255,192,0,127,255,255,192,0,255,255,255,128,1,255,255,255,128,3,224,31,
 255,0,4,0,7,255,0,0,0,7,254,0,0,0,7,254,0,0,0,15,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,
 255,128,0,0,1,255,0,0,0,7,254,0,0,0,15,248,0,0,0,63,240,2,0,0,255,224,7,192,15,255,128,15,255,255,255,0,31,255,255,
 252,0,63,255,255,240,0,127,255,255,224,0,255,255,255,0,1,255,255,252,0,0,255,255,224,0,0,31,252,0,0,0,0,63,224,0,0,
 3,255,252,0,0,31,255,252,0,0,255,255,248,0,3,255,255,240,0,15,255,255,224,0,63,255,255,192,0,127,248,15,128,1,255,
 192,7,0,3,254,0,2,0,15,252,0,0,0,31,240,0,0,0,63,224,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,248,0,0,0,31,240,
 127,128,0,63,227,255,224,0,127,223,255,224,0,255,255,255,224,1,255,255,255,224,3,255,255,255,224,7,255,255,255,192,
 15,255,195,255,192,31,254,1,255,128,63,248,1,255,0,127,240,3,255,0,255,224,7,254,1,255,128,7,252,3,255,0,15,248,7,
 254,0,31,240,15,252,0,63,224,15,248,0,127,192,31,240,0,255,128,63,224,1,255,0,127,192,3,254,0,255,192,15,252,0,255,
 128,31,240,1,255,0,63,224,3,255,0,255,192,3,255,135,255,0,7,255,255,254,0,7,255,255,248,0,7,255,255,224,0,7,255,
 255,128,0,7,255,254,0,0,3,255,240,0,0,0,255,128,0,15,255,255,255,192,31,255,255,255,128,63,255,255,255,0,127,255,
 255,254,0,255,255,255,252,1,255,255,255,248,3,255,255,255,240,7,255,255,255,224,0,0,1,255,128,0,0,3,255,0,0,0,7,
 252,0,0,0,31,248,0,0,0,63,240,0,0,0,255,192,0,0,1,255,128,0,0,3,254,0,0,0,15,252,0,0,0,31,248,0,0,0,127,224,0,0,0,
 255,192,0,0,1,255,0,0,0,7,254,0,0,0,15,252,0,0,0,63,240,0,0,0,127,224,0,0,0,255,128,0,0,3,255,0,0,0,7,254,0,0,0,31,
 248,0,0,0,63,240,0,0,0,127,192,0,0,1,255,128,0,0,3,255,0,0,0,15,252,0,0,0,31,248,0,0,0,63,224,0,0,0,255,192,0,0,1,
 255,128,0,0,7,254,0,0,0,15,252,0,0,0,31,240,0,0,0,127,224,0,0,0,255,192,0,0,3,255,0,0,0,7,254,0,0,0,15,248,0,0,0,
 63,240,0,0,0,0,3,255,0,0,0,31,255,128,0,0,255,255,192,0,3,255,255,192,0,15,255,255,192,0,63,255,255,192,0,255,255,
 255,192,1,255,129,255,128,3,254,1,255,0,15,248,1,255,0,31,240,3,254,0,63,192,3,252,0,127,128,7,248,0,255,0,15,240,
 1,254,0,31,224,3,252,0,63,192,7,252,0,255,128,7,248,1,254,0,15,248,7,252,0,15,248,31,240,0,15,255,255,192,0,15,255,
 255,0,0,15,255,252,0,0,15,255,240,0,0,127,255,248,0,1,255,255,248,0,7,255,255,248,0,31,248,31,248,0,63,224,31,240,
 0,255,128,31,240,1,254,0,31,224,7,248,0,31,224,15,240,0,63,192,31,224,0,127,128,63,192,0,255,0,127,128,1,254,0,255,
 0,3,252,1,254,0,15,248,3,254,0,31,240,7,254,0,127,192,7,254,1,255,128,15,254,7,255,0,15,255,255,252,0,31,255,255,
 248,0,31,255,255,224,0,31,255,255,128,0,31,255,254,0,0,15,255,240,0,0,3,255,0,0,0,7,252,0,0,0,63,255,0,0,1,255,255,
 128,0,7,255,255,128,0,31,255,255,128,0,127,255,255,128,1,255,255,255,128,3,255,135,255,0,15,252,3,255,0,31,240,3,
 254,0,63,224,7,252,0,255,128,15,252,1,255,0,15,248,3,254,0,31,240,7,252,0,63,224,15,248,0,127,192,31,240,0,255,192,
 63,224,1,255,128,127,192,3,255,0,255,192,7,254,1,255,128,31,252,3,255,0,63,248,3,255,0,255,240,7,255,135,255,224,
 15,255,255,255,192,15,255,255,255,128,31,255,255,255,0,31,255,255,254,0,31,255,247,252,0,31,255,207,248,0,31,255,
 31,240,0,7,248,63,224,0,0,0,127,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,31,240,0,0,0,63,224,0,0,0,255,192,1,0,
 3,255,0,3,128,15,254,0,7,192,127,248,0,15,255,255,240,0,31,255,255,192,0,63,255,255,0,0,127,255,252,0,0,255,255,
 224,0,0,255,255,0,0,0,31,240,0,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,0,0,31,248,0,0,0,63,240,0,0,0,127,224,0,0,0,
 255,192,0,0,1,255,128,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,192,0,0,1,255,128,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,
 0,0,31,248,0,0,0,63,240,0,0,0,127,224,0,0,0,255,192,0,0,1,255,128,0,0,3,255,0,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,
 0,0,31,248,0,0,0,63,240,0,0,0,127,224,0,0,0,255,192,0,0,1,255,128,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,192,0,0,1,
 255,128,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,0,0,31,248,0,0,0,63,240,0,0,0,127,224,0,0,0,255,192,0,0,1,255,0,0,0,3,
 254,0,0,0,15,252,0,0,0,31,240,0,0,0,63,224,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,7,248,0,0,0,15,240,0,0,0,31,
 192,0,0,0,0,0,0,16,0,0,0,1,224,0,0,0,15,192,0,0,0,255,128,0,0,15,255,0,0,0,127,254,0,0,7,255,252,0,0,63,255,248,0,
 3,255,255,192,0,31,255,252,0,1,255,255,192,0,31,255,252,0,0,255,255,224,0,3,255,254,0,0,7,255,224,0,0,15,254,0,0,0,
 31,224,0,0,0,63,248,0,0,0,127,254,0,0,0,255,255,128,0,0,255,255,192,0,0,127,255,240,0,0,31,255,252,0,0,7,255,255,0,
 0,3,255,255,192,0,0,255,255,224,0,0,127,255,192,0,0,31,255,128,0,0,15,255,0,0,0,3,254,0,0,0,0,252,0,0,0,0,120,0,0,
 0,0,16,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,31,255,255,255,240,63,255,255,255,224,127,255,255,255,192,255,255,
 255,255,129,255,255,255,255,3,255,255,255,254,7,255,255,255,252,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,7,255,255,255,252,15,255,255,255,248,31,255,255,255,240,63,255,255,255,224,127,255,255,255,192,
 255,255,255,255,129,255,255,255,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,60,0,0,0,0,126,0,0,0,0,255,128,0,0,1,255,224,0,0,3,255,240,0,0,7,255,252,0,0,15,
 255,254,0,0,7,255,255,128,0,1,255,255,192,0,0,127,255,240,0,0,31,255,252,0,0,15,255,254,0,0,3,255,254,0,0,0,255,
 252,0,0,0,63,248,0,0,0,15,240,0,0,0,255,224,0,0,15,255,192,0,0,255,255,128,0,7,255,254,0,0,127,255,240,0,7,255,255,
 0,0,127,255,240,0,7,255,255,128,0,63,255,248,0,0,127,255,192,0,0,255,252,0,0,1,255,224,0,0,3,254,0,0,0,7,224,0,0,0,
 15,0,0,0,0,16,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,128,0,0,15,255,224,0,0,127,255,240,0,3,255,
 255,240,0,15,255,255,240,0,31,255,255,240,0,63,255,255,224,0,127,129,255,192,0,248,0,255,192,1,192,0,255,128,3,0,1,
 255,0,4,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,63,224,0,0,0,255,192,0,0,3,255,0,0,0,15,254,0,0,0,63,248,0,0,0,255,
 224,0,0,3,255,128,0,0,15,254,0,0,0,63,248,0,0,0,127,224,0,0,1,255,0,0,0,3,254,0,0,0,15,248,0,0,0,31,224,0,0,0,63,
 192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,
 0,0,127,192,0,0,15,255,240,0,0,127,255,240,0,1,255,255,240,0,7,255,255,240,0,63,255,255,240,0,127,224,63,240,1,255,
 0,15,224,7,248,0,15,224,31,224,0,15,192,63,128,0,31,128,254,0,0,31,129,252,0,0,63,7,240,1,252,126,15,192,15,252,
 252,31,128,127,255,248,127,1,255,255,240,252,3,255,255,225,248,15,255,255,195,240,63,225,255,143,224,127,0,255,31,
 128,252,0,254,63,3,248,1,252,126,7,224,1,248,252,15,192,3,241,248,31,128,7,227,240,63,0,15,199,224,126,0,31,143,
 192,252,0,63,31,129,248,0,126,63,3,240,0,252,126,7,240,3,248,252,7,224,7,241,252,15,224,31,225,248,31,240,255,195,
 240,31,255,255,135,240,31,255,255,15,224,63,255,254,15,192,63,254,252,31,192,31,249,248,31,128,15,224,0,63,128,0,0,
 0,63,128,0,0,0,127,128,0,0,0,127,128,0,0,0,255,128,0,16,0,255,192,0,240,0,255,240,15,224,0,255,255,255,224,0,255,
 255,255,192,0,127,255,255,192,0,63,255,254,0,0,31,255,240,0,0,7,255,0,0,0,7,255,128,0,0,15,255,0,0,0,31,254,0,0,0,
 127,252,0,0,0,255,252,0,0,1,255,248,0,0,3,255,240,0,0,15,255,240,0,0,31,255,224,0,0,63,255,192,0,0,127,255,128,0,1,
 254,127,128,0,3,252,255,0,0,7,249,254,0,0,15,243,252,0,0,63,195,252,0,0,127,135,248,0,0,255,15,240,0,1,254,31,224,
 0,7,252,63,224,0,15,240,63,192,0,31,224,127,128,0,127,192,255,128,0,255,129,255,0,1,254,1,254,0,3,252,3,252,0,15,
 248,7,252,0,31,240,15,248,0,63,255,255,240,0,127,255,255,224,1,255,255,255,224,3,255,255,255,192,7,255,255,255,128,
 15,255,255,255,0,63,255,255,255,0,127,192,3,254,0,255,128,7,252,3,254,0,7,248,7,252,0,15,248,15,248,0,31,240,31,
 240,0,63,224,127,192,0,63,224,255,128,0,127,193,255,0,0,255,131,254,0,1,255,15,248,0,1,255,31,240,0,3,254,0,15,255,
 255,128,0,31,255,255,240,0,63,255,255,240,0,127,255,255,248,0,255,255,255,248,1,255,255,255,240,3,255,255,255,240,
 7,252,0,255,224,15,248,0,127,224,31,240,0,255,192,63,224,0,255,128,127,192,1,255,0,255,128,3,254,1,255,0,7,252,3,
 254,0,15,248,7,252,0,63,224,15,248,0,127,192,31,240,3,255,0,63,255,255,254,0,127,255,255,248,0,255,255,255,192,1,
 255,255,252,0,3,255,255,255,128,7,255,255,255,128,15,255,255,255,192,31,240,3,255,192,63,224,0,255,128,127,192,0,
 255,128,255,128,1,255,1,255,0,1,255,3,254,0,3,254,7,252,0,7,252,15,248,0,15,248,31,240,0,31,240,63,224,0,63,224,
 127,192,0,127,192,255,128,1,255,129,255,0,3,255,3,254,0,15,252,7,252,0,127,248,15,255,255,255,240,31,255,255,255,
 192,63,255,255,255,0,127,255,255,252,0,255,255,255,240,1,255,255,255,128,3,255,255,240,0,0,0,0,31,248,0,0,1,255,
 255,0,0,15,255,255,0,0,63,255,254,0,0,255,255,252,0,3,255,255,248,0,15,255,255,240,0,63,255,255,224,0,255,252,15,
 192,1,255,224,3,128,7,255,128,3,0,15,254,0,2,0,31,248,0,0,0,127,240,0,0,0,255,192,0,0,1,255,128,0,0,3,255,0,0,0,15,
 254,0,0,0,31,248,0,0,0,63,240,0,0,0,127,224,0,0,0,255,192,0,0,1,255,128,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,0,0,
 31,248,0,0,0,63,240,0,0,0,127,224,0,0,0,255,192,0,0,1,255,128,0,0,3,255,128,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,0,
 0,31,252,0,0,0,31,248,0,0,0,63,248,0,8,0,127,248,0,48,0,127,248,0,224,0,255,252,15,192,0,255,255,255,128,0,255,255,
 255,0,0,255,255,254,0,0,255,255,252,0,0,255,255,248,0,0,255,255,240,0,0,127,255,192,0,0,31,248,0,15,255,248,0,0,31,
 255,255,0,0,63,255,255,128,0,127,255,255,192,0,255,255,255,192,1,255,255,255,224,3,255,255,255,192,7,255,255,255,
 192,15,248,15,255,192,31,240,7,255,128,63,224,7,255,128,127,192,7,255,0,255,128,7,255,1,255,0,15,254,3,254,0,15,
 252,7,252,0,31,248,15,248,0,63,240,31,240,0,63,240,63,224,0,127,224,127,192,0,255,192,255,128,1,255,129,255,0,3,
 255,3,254,0,7,254,7,252,0,15,252,15,248,0,31,248,31,240,0,63,240,63,224,0,127,224,127,192,0,255,192,255,128,1,255,
 129,255,0,3,255,3,254,0,15,252,7,252,0,31,248,15,248,0,63,240,31,240,0,255,224,63,224,1,255,192,127,192,7,255,0,
 255,128,31,254,1,255,0,127,248,3,254,3,255,240,7,255,255,255,192,15,255,255,255,0,31,255,255,254,0,63,255,255,248,
 0,127,255,255,192,0,255,255,254,0,1,255,255,240,0,3,255,252,0,0,0,7,255,255,255,192,15,255,255,255,128,31,255,255,
 255,0,63,255,255,254,0,127,255,255,252,0,255,255,255,248,1,255,255,255,240,3,255,255,255,224,7,252,0,0,0,15,248,0,
 0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,255,
 255,252,0,63,255,255,248,0,127,255,255,240,0,255,255,255,224,1,255,255,255,192,3,255,255,255,128,7,255,255,255,0,
 15,255,255,254,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,
 0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,255,255,255,224,7,255,255,255,192,15,255,255,
 255,128,31,255,255,255,0,63,255,255,254,0,127,255,255,252,0,255,255,255,248,1,255,255,255,240,0,3,255,255,255,224,
 7,255,255,255,192,15,255,255,255,128,31,255,255,255,0,63,255,255,254,0,127,255,255,252,0,255,255,255,248,1,255,255,
 255,240,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,
 0,0,0,7,252,0,0,0,15,255,255,254,0,31,255,255,252,0,63,255,255,248,0,127,255,255,240,0,255,255,255,224,1,255,255,
 255,192,3,255,255,255,128,7,255,255,255,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,
 0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,
 0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,0,0,0,31,248,0,0,1,255,254,0,0,15,
 255,255,0,0,127,255,255,0,1,255,255,254,0,7,255,255,252,0,31,255,255,248,0,127,255,255,240,0,255,248,15,224,3,255,
 192,3,192,7,255,0,3,128,31,252,0,1,0,63,240,0,0,0,255,224,0,0,1,255,128,0,0,3,255,0,0,0,7,254,0,0,0,15,252,0,0,0,
 63,240,0,0,0,127,224,0,0,0,255,192,0,0,1,255,128,0,0,3,255,0,0,0,7,254,0,255,252,15,252,1,255,248,31,248,3,255,240,
 63,240,7,255,224,127,224,15,255,192,255,192,31,255,129,255,128,63,255,3,255,0,127,254,7,255,0,3,252,7,254,0,7,248,
 15,252,0,15,240,31,252,0,31,224,63,248,0,63,192,63,248,0,127,128,127,240,0,255,0,127,240,1,254,0,255,240,3,252,0,
 255,248,15,248,1,255,255,255,240,1,255,255,255,224,1,255,255,255,192,1,255,255,255,128,1,255,255,254,0,0,255,255,
 240,0,0,127,255,128,0,0,31,248,0,15,248,0,127,192,31,240,0,255,128,63,224,1,255,0,127,192,3,254,0,255,128,7,252,1,
 255,0,15,248,3,254,0,31,240,7,252,0,63,224,15,248,0,127,192,31,240,0,255,128,63,224,1,255,0,127,192,3,254,0,255,
 128,7,252,1,255,0,15,248,3,254,0,31,240,7,252,0,63,224,15,248,0,127,192,31,240,0,255,128,63,255,255,255,0,127,255,
 255,254,0,255,255,255,252,1,255,255,255,248,3,255,255,255,240,7,255,255,255,224,15,255,255,255,192,31,255,255,255,
 128,63,224,1,255,0,127,192,3,254,0,255,128,7,252,1,255,0,15,248,3,254,0,31,240,7,252,0,63,224,15,248,0,127,192,31,
 240,0,255,128,63,224,1,255,0,127,192,3,254,0,255,128,7,252,1,255,0,15,248,3,254,0,31,240,7,252,0,63,224,15,248,0,
 127,192,31,240,0,255,128,63,224,1,255,0,127,192,3,254,0,255,128,7,252,1,255,0,15,248,3,254,0,31,240,0,3,255,255,
 255,128,7,255,255,255,0,15,255,255,254,0,31,255,255,252,0,63,255,255,248,0,127,255,255,240,0,255,255,255,224,1,255,
 255,255,192,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,
 1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,
 254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,
 252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,1,255,255,255,192,3,255,255,255,128,7,255,255,255,0,15,
 255,255,254,0,31,255,255,252,0,63,255,255,248,0,127,255,255,240,0,255,255,255,224,0,0,31,255,254,0,0,63,255,252,0,
 0,127,255,248,0,0,255,255,240,0,1,255,255,224,0,3,255,255,192,0,7,255,255,128,0,15,255,255,0,0,0,3,254,0,0,0,7,252,
 0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,
 0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,
 0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,192,0,31,240,1,192,0,127,
 224,3,192,0,255,192,7,224,3,255,0,15,240,31,254,0,31,255,255,252,0,63,255,255,240,0,127,255,255,224,0,255,255,255,
 128,1,255,255,254,0,1,255,255,248,0,0,127,255,192,0,0,15,252,0,0,15,248,0,31,248,31,240,0,127,240,63,224,1,255,192,
 127,192,3,255,0,255,128,15,252,1,255,0,63,248,3,254,0,255,224,7,252,1,255,128,15,248,7,254,0,31,240,31,248,0,63,
 224,127,240,0,127,193,255,192,0,255,131,255,0,1,255,15,252,0,3,254,63,240,0,7,252,255,224,0,15,249,255,128,0,31,
 247,254,0,0,63,255,248,0,0,127,255,248,0,0,255,255,240,0,1,255,255,240,0,3,255,255,224,0,7,255,255,224,0,15,255,
 255,192,0,31,255,255,192,0,63,255,255,128,0,127,249,255,128,0,255,227,255,128,1,255,131,255,0,3,254,3,255,0,7,252,
 7,254,0,15,248,7,254,0,31,240,15,252,0,63,224,15,252,0,127,192,31,248,0,255,128,31,248,1,255,0,63,240,3,254,0,63,
 240,7,252,0,127,224,15,248,0,127,224,31,240,0,255,192,63,224,0,255,192,127,192,1,255,192,255,128,1,255,129,255,0,3,
 255,131,254,0,3,255,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,
 128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,
 0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,
 0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,
 0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,255,255,248,1,255,255,255,240,3,255,255,255,224,7,255,
 255,255,192,15,255,255,255,128,31,255,255,255,0,63,255,255,254,0,127,255,255,252,0,31,252,0,127,240,63,248,0,255,
 224,127,248,3,255,192,255,240,7,255,129,255,224,15,255,3,255,192,63,254,7,255,192,127,252,15,255,128,255,248,31,
 255,1,255,240,63,255,7,255,224,127,254,15,255,192,255,124,31,255,129,254,248,62,255,3,253,248,253,254,7,251,241,
 251,252,15,247,227,247,248,31,231,199,207,240,63,207,223,159,224,127,159,191,63,192,255,63,126,127,129,254,62,248,
 255,3,252,127,241,254,7,248,255,227,252,15,241,255,199,248,31,225,255,15,240,63,195,254,31,224,127,135,252,63,192,
 255,15,248,127,129,254,15,240,255,3,252,31,193,254,7,248,0,3,252,15,240,0,7,248,31,224,0,15,240,63,192,0,31,224,
 127,128,0,63,192,255,0,0,127,129,254,0,0,255,3,252,0,1,254,7,248,0,3,252,15,240,0,7,248,31,224,0,15,240,63,192,0,
 31,224,127,128,0,63,192,255,0,0,127,129,254,0,0,255,3,252,0,1,254,7,248,0,3,252,0,15,252,0,31,224,31,248,0,63,192,
 63,248,0,127,128,127,240,0,255,0,255,240,1,254,1,255,224,3,252,3,255,192,7,248,7,255,192,15,240,15,255,128,31,224,
 31,255,0,63,192,63,255,0,127,128,127,254,0,255,0,255,254,1,254,1,255,252,3,252,3,253,248,7,248,7,251,248,15,240,15,
 247,240,31,224,31,231,240,63,192,63,207,224,127,128,127,143,192,255,0,255,31,193,254,1,254,63,131,252,3,252,63,135,
 248,7,248,127,15,240,15,240,126,31,224,31,224,254,63,192,63,193,252,127,128,127,129,248,255,0,255,3,249,254,1,254,
 7,243,252,3,252,7,247,248,7,248,15,239,240,15,240,15,223,224,31,224,31,255,192,63,192,63,255,128,127,128,63,255,0,
 255,0,127,254,1,254,0,127,252,3,252,0,255,248,7,248,1,255,240,15,240,1,255,224,31,224,3,255,192,63,192,7,255,128,
 127,128,7,255,0,255,0,15,254,1,254,0,15,252,3,252,0,31,248,0,0,1,255,0,0,0,31,255,192,0,0,127,255,192,0,3,255,255,
 224,0,15,255,255,224,0,31,255,255,192,0,127,255,255,192,1,255,255,255,192,3,255,131,255,128,15,254,3,255,128,31,
 248,3,255,0,63,240,7,254,0,255,192,7,254,1,255,128,15,252,3,255,0,31,248,7,254,0,63,240,15,248,0,63,224,63,240,0,
 127,224,127,224,0,255,192,255,192,1,255,129,255,128,3,255,3,255,0,7,254,7,254,0,15,252,15,252,0,31,248,31,248,0,63,
 240,63,240,0,127,224,127,224,0,255,192,255,192,1,255,129,255,128,3,255,3,255,0,7,254,7,254,0,15,252,15,252,0,31,
 248,15,248,0,63,224,31,248,0,255,192,63,240,1,255,128,127,224,3,255,0,255,192,7,254,0,255,192,31,248,1,255,128,63,
 240,3,255,128,255,224,3,255,131,255,128,7,255,255,255,0,7,255,255,252,0,7,255,255,240,0,15,255,255,192,0,15,255,
 255,0,0,7,255,252,0,0,7,255,240,0,0,1,255,0,0,7,255,255,0,0,15,255,255,224,0,31,255,255,240,0,63,255,255,248,0,127,
 255,255,248,0,255,255,255,248,1,255,255,255,240,3,255,255,255,240,7,252,3,255,224,15,248,1,255,192,31,240,1,255,
 192,63,224,1,255,128,127,192,3,255,0,255,128,7,254,1,255,0,15,252,3,254,0,31,248,7,252,0,63,240,15,248,0,127,224,
 31,240,1,255,192,63,224,7,255,0,127,192,63,254,0,255,255,255,252,1,255,255,255,240,3,255,255,255,224,7,255,255,255,
 128,15,255,255,254,0,31,255,255,240,0,63,255,255,128,0,127,255,240,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,
 0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,
 0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,0,0,1,255,0,0,0,31,255,192,0,0,127,255,192,0,3,
 255,255,224,0,15,255,255,224,0,31,255,255,192,0,127,255,255,192,1,255,255,255,192,3,255,131,255,128,15,254,3,255,
 128,31,248,3,255,0,63,240,7,254,0,255,192,7,254,1,255,128,15,252,3,255,0,31,248,7,254,0,63,240,15,248,0,63,224,63,
 240,0,127,224,127,224,0,255,192,255,192,1,255,129,255,128,3,255,3,255,0,7,254,7,254,0,15,252,15,252,0,31,248,31,
 248,0,63,240,63,240,0,127,224,127,224,0,255,192,255,192,1,255,129,255,128,3,255,3,255,0,7,254,7,254,0,15,252,15,
 252,0,31,248,15,248,0,63,224,31,248,0,255,192,63,240,1,255,128,127,224,3,255,0,255,192,7,254,0,255,192,31,248,1,
 255,128,63,240,3,255,128,255,224,3,255,131,255,128,7,255,255,255,0,7,255,255,252,0,7,255,255,240,0,15,255,255,192,
 0,15,255,255,128,0,7,255,252,0,0,7,255,252,0,0,1,255,252,0,0,0,15,252,0,0,0,15,252,0,0,0,15,252,0,0,0,15,252,0,0,0,
 15,224,0,0,0,31,128,0,0,0,30,0,0,0,0,16,0,15,255,254,0,0,31,255,255,192,0,63,255,255,224,0,127,255,255,224,0,255,
 255,255,224,1,255,255,255,224,3,255,255,255,224,7,255,255,255,192,15,248,7,255,128,31,240,3,255,128,63,224,7,255,0,
 127,192,7,254,0,255,128,15,252,1,255,0,31,248,3,254,0,63,240,7,252,0,127,224,15,248,0,255,192,31,240,3,255,0,63,
 224,15,254,0,127,192,63,252,0,255,255,255,240,1,255,255,255,192,3,255,255,255,0,7,255,255,248,0,15,255,255,128,0,
 31,255,255,192,0,63,255,255,192,0,127,255,255,128,0,255,131,255,128,1,255,3,255,0,3,254,3,255,0,7,252,7,254,0,15,
 248,7,254,0,31,240,15,252,0,63,224,15,252,0,127,192,31,248,0,255,128,31,248,1,255,0,63,240,3,254,0,63,240,7,252,0,
 127,224,15,248,0,127,224,31,240,0,255,192,63,224,0,255,192,127,192,1,255,128,255,128,1,255,129,255,0,3,255,3,254,0,
 3,255,0,0,3,255,0,0,0,63,255,224,0,0,255,255,248,0,7,255,255,248,0,31,255,255,240,0,127,255,255,224,0,255,255,255,
 192,3,255,255,255,128,7,255,128,255,0,15,252,0,62,0,63,240,0,28,0,127,192,0,8,0,255,128,0,0,1,255,0,0,0,3,254,0,0,
 0,7,254,0,0,0,15,254,0,0,0,31,254,0,0,0,31,255,128,0,0,63,255,224,0,0,63,255,240,0,0,127,255,252,0,0,127,255,252,0,
 0,127,255,254,0,0,63,255,254,0,0,31,255,254,0,0,15,255,254,0,0,7,255,252,0,0,1,255,252,0,0,0,127,248,0,0,0,63,240,
 0,0,0,63,240,0,0,0,127,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,128,0,3,254,1,192,0,7,252,3,192,0,31,248,7,240,
 0,127,224,15,252,3,255,192,31,255,255,255,128,63,255,255,254,0,127,255,255,252,0,255,255,255,240,1,255,255,255,192,
 1,255,255,254,0,0,127,255,240,0,0,15,255,0,0,31,255,255,255,240,63,255,255,255,224,127,255,255,255,192,255,255,255,
 255,129,255,255,255,255,3,255,255,255,254,7,255,255,255,252,15,255,255,255,248,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,
 0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,
 0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,
 0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,
 0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,
 31,240,0,63,224,63,224,0,127,192,127,192,0,255,128,255,128,1,255,1,255,0,3,254,3,254,0,7,252,7,252,0,15,248,15,248,
 0,31,240,31,240,0,63,224,63,224,0,127,192,127,192,0,255,128,255,128,1,255,1,255,0,3,254,3,254,0,7,252,7,252,0,15,
 248,15,248,0,31,240,31,240,0,63,224,63,224,0,127,192,127,192,0,255,128,255,128,1,255,1,255,0,3,254,3,254,0,7,252,7,
 252,0,15,248,15,248,0,31,240,31,240,0,63,224,63,224,0,127,192,127,192,0,255,128,255,128,1,255,1,255,0,3,254,3,254,
 0,7,252,7,252,0,15,248,15,248,0,31,240,31,240,0,63,224,63,224,0,127,192,127,192,0,255,128,255,128,1,255,0,255,128,
 7,252,1,255,0,15,248,3,255,0,63,240,7,255,129,255,224,7,255,255,255,128,15,255,255,255,0,15,255,255,252,0,15,255,
 255,240,0,15,255,255,192,0,15,255,255,0,0,7,255,248,0,0,1,255,128,0,63,224,0,15,248,127,224,0,63,240,127,192,0,127,
 192,255,128,0,255,129,255,0,1,255,3,254,0,3,254,3,254,0,15,248,7,252,0,31,240,15,248,0,63,224,31,240,0,127,192,63,
 224,1,255,128,63,224,3,254,0,127,192,7,252,0,255,128,15,248,1,255,0,31,240,1,255,0,127,192,3,254,0,255,128,7,252,1,
 255,0,15,248,3,254,0,15,240,7,248,0,31,240,31,240,0,63,224,63,224,0,127,192,127,192,0,127,128,255,0,0,255,1,254,0,
 1,255,7,252,0,3,254,15,248,0,7,252,31,240,0,7,248,63,192,0,15,240,127,128,0,31,241,255,0,0,63,227,254,0,0,63,199,
 248,0,0,127,143,240,0,0,255,63,224,0,1,255,127,192,0,1,254,255,0,0,3,253,254,0,0,7,251,252,0,0,15,255,248,0,0,15,
 255,224,0,0,31,255,192,0,0,63,255,128,0,0,127,255,0,0,0,127,252,0,0,0,255,248,0,0,1,255,240,0,0,255,0,0,1,255,254,
 0,0,3,255,252,0,0,7,255,252,0,0,31,247,248,0,0,63,207,240,0,0,127,159,224,0,0,255,63,192,0,1,254,127,128,0,3,252,
 255,0,0,7,249,254,0,0,15,243,252,0,0,31,227,248,15,224,63,199,240,63,224,127,15,240,127,192,254,31,224,255,129,252,
 63,193,255,7,248,127,131,255,15,240,255,15,254,31,225,254,31,252,63,193,252,63,248,127,131,248,127,240,255,7,240,
 251,241,252,15,225,247,227,248,31,231,239,199,240,63,207,223,143,224,127,159,191,31,192,255,62,63,63,128,254,124,
 126,127,1,253,248,253,254,3,251,241,251,248,7,247,227,255,240,15,239,131,255,224,31,223,7,255,192,63,254,15,255,
 128,127,252,31,255,0,127,248,31,254,0,255,224,63,252,1,255,192,127,248,3,255,128,255,224,7,255,1,255,192,15,254,1,
 255,128,31,248,3,255,0,63,240,7,254,0,63,224,15,252,0,127,192,15,248,0,255,128,31,240,0,127,224,0,15,252,127,224,0,
 63,240,255,192,0,127,224,255,192,1,255,128,255,128,3,254,1,255,128,15,252,1,255,128,63,240,3,255,0,127,224,3,255,1,
 255,128,3,254,3,254,0,7,254,15,252,0,7,252,63,240,0,15,252,127,224,0,15,253,255,128,0,15,251,255,0,0,31,255,252,0,
 0,31,255,240,0,0,63,255,224,0,0,63,255,128,0,0,127,255,0,0,0,127,252,0,0,0,127,240,0,0,0,255,224,0,0,1,255,192,0,0,
 3,255,128,0,0,15,255,128,0,0,31,255,0,0,0,127,255,0,0,0,255,254,0,0,3,255,254,0,0,15,255,254,0,0,31,247,252,0,0,
 127,239,252,0,0,255,143,248,0,3,255,31,248,0,7,252,31,240,0,31,248,63,240,0,127,224,63,240,0,255,128,63,224,3,255,
 0,127,224,7,252,0,127,192,31,248,0,255,192,63,224,0,255,128,255,128,0,255,131,255,0,1,255,135,252,0,1,255,31,248,0,
 3,255,0,127,224,0,15,252,127,192,0,31,240,255,192,0,127,224,255,128,0,255,129,255,128,3,255,1,255,0,7,252,3,255,0,
 31,248,3,254,0,63,224,7,254,0,255,192,7,252,1,255,0,15,252,7,254,0,15,248,15,248,0,31,248,63,240,0,31,240,127,192,
 0,63,224,255,128,0,63,227,254,0,0,127,199,252,0,0,127,223,240,0,0,255,191,224,0,0,255,255,128,0,1,255,255,0,0,1,
 255,252,0,0,3,255,248,0,0,3,255,224,0,0,7,255,192,0,0,7,255,0,0,0,15,254,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,
 0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,
 0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,7,255,255,
 255,248,15,255,255,255,240,31,255,255,255,224,63,255,255,255,192,127,255,255,255,128,255,255,255,255,1,255,255,255,
 254,3,255,255,255,252,0,0,0,127,240,0,0,0,255,224,0,0,3,255,128,0,0,15,254,0,0,0,63,252,0,0,0,127,240,0,0,1,255,
 192,0,0,7,255,128,0,0,15,254,0,0,0,63,248,0,0,0,255,224,0,0,1,255,192,0,0,7,255,0,0,0,31,252,0,0,0,63,248,0,0,0,
 255,224,0,0,3,255,128,0,0,15,255,0,0,0,31,252,0,0,0,127,240,0,0,1,255,192,0,0,3,255,128,0,0,15,254,0,0,0,63,248,0,
 0,0,127,240,0,0,1,255,192,0,0,7,255,0,0,0,15,254,0,0,0,63,248,0,0,0,255,224,0,0,1,255,128,0,0,7,255,255,255,252,15,
 255,255,255,248,31,255,255,255,240,63,255,255,255,224,127,255,255,255,192,255,255,255,255,129,255,255,255,255,3,
 255,255,255,254,0,0,7,255,248,0,0,15,255,240,0,0,31,255,224,0,0,63,255,192,0,0,127,255,128,0,0,255,255,0,0,1,254,0,
 0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,
 7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,
 240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,
 224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,
 192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,255,192,0,0,127,
 255,128,0,0,255,255,0,0,1,255,254,0,0,3,255,252,0,0,7,255,248,0,31,192,0,0,0,31,192,0,0,0,63,128,0,0,0,63,128,0,0,
 0,127,0,0,0,0,127,0,0,0,0,254,0,0,0,0,254,0,0,0,1,252,0,0,0,3,252,0,0,0,3,248,0,0,0,7,240,0,0,0,7,240,0,0,0,15,224,
 0,0,0,15,224,0,0,0,31,192,0,0,0,31,192,0,0,0,63,128,0,0,0,63,128,0,0,0,127,0,0,0,0,127,0,0,0,0,254,0,0,0,0,254,0,0,
 0,1,252,0,0,0,1,252,0,0,0,3,248,0,0,0,7,248,0,0,0,7,240,0,0,0,15,224,0,0,0,15,224,0,0,0,31,192,0,0,0,31,192,0,0,0,
 63,128,0,0,0,63,128,0,0,0,127,0,0,0,0,127,0,0,0,0,254,0,0,0,0,254,0,0,0,1,252,0,0,0,1,252,0,0,0,3,248,0,0,0,3,248,
 0,0,0,7,240,0,0,0,15,240,0,0,0,15,224,0,0,0,31,192,0,0,0,31,192,0,0,0,63,128,0,0,0,63,128,0,0,0,127,0,0,0,0,127,0,
 0,0,0,254,0,0,0,0,254,0,0,127,255,128,0,0,255,255,0,0,1,255,254,0,0,3,255,252,0,0,7,255,248,0,0,15,255,240,0,0,0,
 31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,
 192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,
 128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,
 0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,
 3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,3,255,252,0,0,7,
 255,248,0,0,15,255,240,0,0,31,255,224,0,0,63,255,192,0,0,127,255,128,0,0,1,255,0,0,0,7,255,0,0,0,15,254,0,0,0,63,
 254,0,0,0,255,254,0,0,3,255,254,0,0,15,255,254,0,0,31,255,252,0,0,127,239,252,0,1,255,143,252,0,7,254,15,252,0,31,
 248,15,252,0,63,192,7,248,0,255,0,7,248,3,252,0,7,248,15,240,0,7,248,63,192,0,7,248,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,1,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
 255,255,255,128,3,254,0,0,0,3,254,0,0,0,3,254,0,0,0,3,254,0,0,0,3,252,0,0,0,3,252,0,0,0,3,252,0,0,0,3,252,0,0,0,3,
 248,0,0,0,3,248,0,0,0,3,248,0,0,0,3,248,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,255,128,0,1,255,255,224,0,
 15,255,255,240,0,31,255,255,240,0,63,255,255,240,0,127,255,255,240,0,255,255,255,224,1,252,0,255,224,3,0,0,127,192,
 0,0,0,127,128,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,255,255,252,0,15,255,255,248,0,127,255,255,240,3,255,255,255,
 224,15,255,255,255,192,31,255,255,255,128,127,255,255,255,0,255,240,3,254,3,255,128,7,252,7,254,0,15,248,15,248,0,
 31,240,31,240,0,127,224,63,224,0,255,192,127,192,1,255,128,255,192,7,255,1,255,128,31,254,1,255,192,255,252,3,255,
 255,255,248,7,255,255,255,240,7,255,255,191,224,7,255,255,127,192,7,255,248,255,128,3,255,225,255,0,1,254,0,0,0,7,
 252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,
 248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,129,252,0,1,255,15,254,0,3,254,127,255,0,7,253,255,255,0,15,
 251,255,254,0,31,255,255,254,0,63,255,255,254,0,127,252,31,252,0,255,240,31,252,1,255,192,31,248,3,255,128,63,240,
 7,254,0,63,224,15,252,0,127,224,31,240,0,255,192,63,224,0,255,128,127,192,1,255,0,255,128,3,254,1,255,0,7,252,3,
 254,0,15,248,7,252,0,31,240,15,248,0,63,224,31,240,0,127,192,63,224,0,255,128,127,192,1,255,0,255,192,7,254,1,255,
 128,15,248,3,255,128,63,240,7,255,0,127,224,15,255,1,255,128,31,255,7,255,0,63,255,255,254,0,127,255,255,248,0,255,
 191,255,224,1,255,127,255,192,3,254,127,255,0,7,252,63,248,0,0,0,31,192,0,0,0,63,224,0,0,7,255,248,0,0,31,255,252,
 0,0,255,255,252,0,3,255,255,248,0,15,255,255,240,0,63,255,255,224,0,127,248,15,192,1,255,192,7,128,3,255,0,3,0,15,
 252,0,2,0,31,240,0,0,0,63,224,0,0,0,255,192,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,
 224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,252,0,2,0,15,252,0,12,0,31,
 252,0,56,0,31,254,3,240,0,63,255,255,224,0,63,255,255,192,0,63,255,255,128,0,63,255,255,0,0,63,255,252,0,0,31,255,
 224,0,0,7,254,0,0,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,
 224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,1,252,15,248,0,15,254,31,240,0,127,255,63,224,
 1,255,255,127,192,3,255,254,255,128,15,255,255,255,0,63,255,255,254,0,127,240,127,252,1,255,192,127,248,3,255,0,
 127,240,7,252,0,255,224,15,248,0,255,192,63,240,1,255,128,127,192,3,255,0,255,128,3,254,1,255,0,7,252,3,254,0,15,
 248,7,252,0,31,240,15,248,0,63,224,31,240,0,127,192,63,224,0,255,128,127,192,1,255,0,255,128,3,254,1,255,0,7,252,3,
 255,0,31,248,3,254,0,63,240,7,252,0,255,224,15,252,1,255,192,31,252,7,255,128,31,252,31,255,0,63,255,255,254,0,63,
 255,255,252,0,63,255,239,248,0,127,255,223,240,0,127,255,63,224,0,63,248,127,192,0,31,192,0,0,0,1,255,0,0,0,31,255,
 192,0,0,255,255,224,0,3,255,255,240,0,15,255,255,240,0,63,255,255,240,0,255,255,255,224,1,255,128,255,224,7,254,0,
 127,192,15,248,0,127,192,63,224,0,255,128,127,192,0,255,0,255,0,1,255,3,254,0,3,254,7,255,255,255,252,15,255,255,
 255,248,31,255,255,255,240,63,255,255,255,224,127,255,255,255,192,255,255,255,255,129,255,255,255,255,3,254,0,0,0,
 7,252,0,0,0,15,248,0,0,0,15,240,0,0,0,31,240,0,0,0,63,224,0,0,128,63,224,0,7,0,127,240,0,62,0,127,248,3,252,0,255,
 255,255,248,0,255,255,255,240,0,255,255,255,224,0,255,255,255,192,0,127,255,255,0,0,63,255,240,0,0,15,254,0,0,0,0,
 7,255,192,0,0,127,255,128,0,1,255,255,0,0,7,255,254,0,0,31,255,252,0,0,63,255,248,0,0,255,255,240,0,1,255,128,0,0,
 3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,1,255,255,255,240,3,255,255,255,224,7,255,
 255,255,192,15,255,255,255,128,31,255,255,255,0,63,255,255,254,0,127,255,255,252,0,0,127,192,0,0,0,255,128,0,0,1,
 255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,
 254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,
 252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,15,224,0,0,0,
 127,248,255,128,1,255,249,255,0,7,255,251,254,0,31,255,255,252,0,127,255,255,248,0,255,255,255,240,3,255,131,255,
 224,7,254,3,255,192,31,248,3,255,128,63,224,7,255,0,127,192,7,254,1,255,128,15,252,3,254,0,15,248,7,252,0,31,240,
 15,248,0,63,224,31,240,0,127,192,63,224,0,255,128,127,192,1,255,0,255,128,3,254,1,255,0,7,252,3,254,0,15,248,7,252,
 0,31,240,15,252,0,127,224,15,248,0,255,192,31,248,3,255,128,63,240,7,255,0,127,240,31,254,0,127,240,127,252,0,255,
 255,255,248,0,255,255,255,240,0,255,255,255,224,0,255,255,127,192,0,255,252,255,128,0,255,241,255,0,0,127,131,254,
 0,0,0,7,252,0,0,0,15,248,0,0,0,63,240,0,128,0,127,192,1,192,1,255,128,3,240,15,255,0,7,255,255,252,0,15,255,255,
 248,0,31,255,255,224,0,63,255,255,128,0,127,255,254,0,0,127,255,240,0,0,7,255,0,0,7,252,0,0,0,15,248,0,0,0,31,240,
 0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,
 0,0,0,127,192,0,0,0,255,131,252,0,1,255,31,254,0,3,254,127,254,0,7,253,255,254,0,15,251,255,252,0,31,255,255,252,0,
 63,255,255,248,0,127,248,63,240,0,255,224,63,240,1,255,128,127,224,3,255,0,127,192,7,252,0,255,128,15,248,1,255,0,
 31,240,3,254,0,63,224,7,252,0,127,192,15,248,0,255,128,31,240,1,255,0,63,224,3,254,0,127,192,7,252,0,255,128,15,
 248,1,255,0,31,240,3,254,0,63,224,7,252,0,127,192,15,248,0,255,128,31,240,1,255,0,63,224,3,254,0,127,192,7,252,0,
 255,128,15,248,1,255,0,31,240,3,254,0,63,224,7,252,0,127,192,15,248,0,255,128,31,240,1,255,0,63,224,3,254,0,127,
 192,7,252,0,255,128,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,
 192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,255,255,0,
 0,7,255,254,0,0,15,255,252,0,0,31,255,248,0,0,63,255,240,0,0,127,255,224,0,0,255,255,192,0,0,0,255,128,0,0,1,255,0,
 0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,
 0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,
 15,248,0,0,255,255,255,254,1,255,255,255,252,3,255,255,255,248,7,255,255,255,240,15,255,255,255,224,31,255,255,255,
 192,63,255,255,255,128,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,
 63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,255,
 255,128,0,3,255,255,0,0,7,255,254,0,0,15,255,252,0,0,31,255,248,0,0,63,255,240,0,0,127,255,224,0,0,0,127,192,0,0,0,
 255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,
 255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,
 254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,
 252,0,0,0,15,248,0,0,0,31,240,0,0,0,127,224,0,0,0,255,128,0,0,7,255,0,0,63,255,254,0,0,127,255,252,0,0,255,255,240,
 0,1,255,255,192,0,3,255,255,0,0,7,255,252,0,0,15,255,192,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,
 127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,
 255,128,0,0,1,255,0,31,252,3,254,0,127,240,7,252,1,255,192,15,248,7,255,0,31,240,31,252,0,63,224,127,240,0,127,193,
 255,192,0,255,135,255,0,1,255,31,252,0,3,254,127,240,0,7,253,255,192,0,15,255,255,0,0,31,255,248,0,0,63,255,240,0,
 0,127,255,240,0,0,255,255,224,0,1,255,255,224,0,3,255,255,224,0,7,255,255,192,0,15,254,127,192,0,31,248,255,192,0,
 63,224,255,128,0,127,193,255,128,0,255,129,255,128,1,255,1,255,0,3,254,3,255,0,7,252,3,255,0,15,248,7,254,0,31,240,
 7,254,0,63,224,7,252,0,127,192,15,252,0,255,128,15,252,1,255,0,31,248,3,254,0,31,248,7,252,0,63,248,31,255,248,0,0,
 63,255,240,0,0,127,255,224,0,0,255,255,192,0,1,255,255,128,0,3,255,255,0,0,7,255,254,0,0,0,7,252,0,0,0,15,248,0,0,
 0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,
 0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,
 0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,
 0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,254,0,0,0,7,254,0,0,0,15,254,0,0,0,31,255,255,0,0,63,255,254,0,0,63,255,
 252,0,0,63,255,248,0,0,63,255,240,0,0,63,255,224,0,0,15,255,192,0,3,224,62,0,31,207,241,255,0,63,191,243,255,0,127,
 255,239,255,0,255,255,255,254,1,255,255,255,252,3,255,255,255,248,7,252,255,207,248,15,240,255,15,240,31,225,254,
 31,224,63,195,252,63,192,127,135,248,127,128,255,15,240,255,1,254,31,225,254,3,252,63,195,252,7,248,127,135,248,15,
 240,255,15,240,31,225,254,31,224,63,195,252,63,192,127,135,248,127,128,255,15,240,255,1,254,31,225,254,3,252,63,
 195,252,7,248,127,135,248,15,240,255,15,240,31,225,254,31,224,63,195,252,63,192,127,135,248,127,128,255,15,240,255,
 1,254,31,225,254,3,252,63,195,252,7,248,127,135,248,15,240,255,15,240,31,225,254,31,224,63,195,252,63,192,127,135,
 248,127,128,0,0,31,224,0,15,248,255,240,0,31,243,255,240,0,63,239,255,240,0,127,223,255,224,0,255,255,255,224,1,
 255,255,255,192,3,255,193,255,128,7,255,1,255,128,15,252,3,255,0,31,248,3,254,0,63,240,7,252,0,127,192,15,248,0,
 255,128,31,240,1,255,0,63,224,3,254,0,127,192,7,252,0,255,128,15,248,1,255,0,31,240,3,254,0,63,224,7,252,0,127,192,
 15,248,0,255,128,31,240,1,255,0,63,224,3,254,0,127,192,7,252,0,255,128,15,248,1,255,0,31,240,3,254,0,63,224,7,252,
 0,127,192,15,248,0,255,128,31,240,1,255,0,63,224,3,254,0,127,192,7,252,0,255,128,15,248,1,255,0,31,240,3,254,0,63,
 224,7,252,0,0,1,254,0,0,0,31,255,128,0,0,255,255,192,0,3,255,255,192,0,15,255,255,192,0,63,255,255,192,0,255,255,
 255,192,1,255,195,255,128,7,254,1,255,128,15,248,1,255,0,63,240,3,255,0,127,192,3,254,0,255,128,7,252,3,254,0,15,
 252,7,252,0,15,248,15,248,0,31,240,31,240,0,63,224,63,224,0,127,192,127,192,0,255,128,255,128,1,255,1,255,0,3,254,
 3,254,0,7,252,7,252,0,15,248,15,252,0,63,240,15,248,0,127,192,31,240,0,255,128,63,240,3,255,0,63,224,7,252,0,127,
 224,31,248,0,127,240,255,224,0,255,255,255,192,0,255,255,255,0,0,255,255,252,0,0,255,255,240,0,0,255,255,192,0,0,
 127,254,0,0,0,31,224,0,0,0,0,15,224,0,15,248,127,240,0,31,243,255,248,0,63,239,255,248,0,127,223,255,240,0,255,255,
 255,240,1,255,255,255,240,3,255,224,255,224,7,255,128,255,224,15,254,0,255,192,31,252,1,255,128,63,240,1,255,0,127,
 224,3,255,0,255,128,7,254,1,255,0,7,252,3,254,0,15,248,7,252,0,31,240,15,248,0,63,224,31,240,0,127,192,63,224,0,
 255,128,127,192,1,255,0,255,128,3,254,1,255,0,7,252,3,254,0,15,248,7,254,0,63,240,15,252,0,127,192,31,252,1,255,
 128,63,248,3,255,0,127,248,15,254,0,255,248,63,248,1,255,255,255,240,3,255,255,255,192,7,253,255,255,0,15,251,255,
 254,0,31,243,255,240,0,63,225,255,192,0,127,192,254,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,
 0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,0,15,224,0,0,0,127,240,
 255,128,3,255,249,255,0,15,255,251,254,0,31,255,247,252,0,127,255,255,248,1,255,255,255,240,3,255,131,255,224,15,
 254,3,255,192,31,248,3,255,128,63,224,7,255,0,127,192,7,254,1,255,128,15,252,3,254,0,31,248,7,252,0,31,240,15,248,
 0,63,224,31,240,0,127,192,63,224,0,255,128,127,192,1,255,0,255,128,3,254,1,255,0,7,252,3,254,0,15,248,7,252,0,31,
 240,15,248,0,63,224,31,248,0,255,192,31,240,1,255,128,63,224,7,255,0,127,224,15,254,0,255,224,63,252,0,255,224,255,
 248,1,255,255,255,240,1,255,255,255,224,1,255,255,127,192,3,255,254,255,128,3,255,249,255,0,1,255,195,254,0,0,254,
 7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,
 15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,127,0,0,255,135,255,192,1,255,31,255,192,3,254,127,255,128,7,
 253,255,255,0,15,255,255,254,0,31,255,255,252,0,63,255,128,248,0,127,252,0,112,0,255,240,0,32,1,255,192,0,0,3,255,
 0,0,0,7,254,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,
 0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,
 0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,0,3,255,128,0,0,63,255,240,0,1,255,
 255,248,0,7,255,255,240,0,31,255,255,224,0,127,255,255,192,0,255,255,255,128,3,255,192,63,0,7,254,0,14,0,15,248,0,
 4,0,31,240,0,0,0,63,224,0,0,0,127,224,0,0,0,255,240,0,0,0,255,254,0,0,1,255,255,128,0,3,255,255,224,0,3,255,255,
 240,0,3,255,255,240,0,1,255,255,240,0,0,255,255,240,0,0,63,255,224,0,0,7,255,224,0,0,1,255,192,0,0,1,255,128,0,0,1,
 255,0,0,0,3,254,0,16,0,7,252,0,60,0,31,248,0,127,0,127,240,0,255,255,255,192,1,255,255,255,128,3,255,255,254,0,7,
 255,255,248,0,15,255,255,224,0,7,255,255,0,0,0,127,248,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,
 255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,127,255,255,254,0,255,255,255,252,1,255,
 255,255,248,3,255,255,255,240,7,255,255,255,224,15,255,255,255,192,31,255,255,255,128,0,31,240,0,0,0,63,224,0,0,0,
 127,192,0,0,0,255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,
 255,128,0,0,1,255,0,0,0,3,254,0,0,0,7,252,0,0,0,15,248,0,0,0,31,240,0,0,0,63,224,0,0,0,127,192,0,0,0,255,192,0,0,1,
 255,192,0,0,1,255,255,224,0,3,255,255,192,0,7,255,255,128,0,7,255,255,0,0,7,255,254,0,0,7,255,252,0,0,1,255,248,0,
 7,252,0,255,128,15,248,1,255,0,31,240,3,254,0,63,224,7,252,0,127,192,15,248,0,255,128,31,240,1,255,0,63,224,3,254,
 0,127,192,7,252,0,255,128,15,248,1,255,0,31,240,3,254,0,63,224,7,252,0,127,192,15,248,0,255,128,31,240,1,255,0,63,
 224,3,254,0,127,192,7,252,0,255,128,15,248,1,255,0,31,240,3,254,0,63,224,7,252,0,127,192,15,248,0,255,128,31,240,1,
 255,0,63,224,3,254,0,127,192,7,252,0,255,128,15,248,3,255,0,31,248,7,254,0,63,240,31,252,0,63,240,127,248,0,127,
 255,255,240,0,255,255,255,224,0,255,255,127,192,1,255,254,255,128,1,255,249,255,0,1,255,227,254,0,0,255,0,0,0,31,
 248,0,31,248,63,240,0,63,240,63,224,0,127,192,127,192,0,255,128,255,192,3,255,0,255,128,7,252,1,255,0,15,248,3,254,
 0,63,240,3,254,0,127,192,7,252,0,255,128,15,248,1,255,0,15,248,7,252,0,31,240,15,248,0,63,224,31,240,0,63,192,63,
 192,0,127,192,255,128,0,255,129,255,0,0,255,3,252,0,1,255,15,248,0,3,254,31,240,0,3,252,63,192,0,7,248,127,128,0,
 15,249,255,0,0,15,243,252,0,0,31,231,248,0,0,63,223,240,0,0,63,255,192,0,0,127,255,128,0,0,255,255,0,0,0,255,254,0,
 0,1,255,248,0,0,3,255,240,0,0,7,255,224,0,0,7,255,128,0,0,15,255,0,0,255,0,0,1,255,254,0,0,3,255,252,0,0,7,251,248,
 0,0,15,231,240,0,0,31,207,240,0,0,127,159,224,0,0,255,63,192,0,1,254,63,128,0,3,248,127,1,252,7,240,254,7,252,15,
 225,254,15,248,63,195,252,31,240,127,135,248,63,224,255,7,240,127,193,252,15,225,255,195,248,31,195,255,135,240,63,
 135,223,31,224,127,143,190,63,192,127,63,124,127,0,254,126,252,254,1,252,248,249,252,3,249,241,243,248,7,243,227,
 231,240,15,255,199,255,224,15,255,7,255,128,31,254,15,255,0,63,252,31,254,0,127,248,63,252,0,255,240,127,248,0,255,
 192,127,224,1,255,128,255,192,3,255,1,255,128,7,254,3,255,0,15,248,3,254,0,31,252,0,127,240,31,252,1,255,192,31,
 248,3,255,0,63,248,15,254,0,63,240,31,248,0,63,240,127,224,0,63,224,255,128,0,127,227,255,0,0,127,199,252,0,0,127,
 223,240,0,0,255,255,224,0,0,255,255,128,0,0,255,254,0,0,0,255,252,0,0,1,255,240,0,0,1,255,192,0,0,1,255,0,0,0,7,
 255,0,0,0,15,254,0,0,0,63,254,0,0,0,255,254,0,0,1,255,254,0,0,7,255,252,0,0,31,247,252,0,0,127,239,252,0,0,255,143,
 248,0,3,255,31,248,0,15,252,31,248,0,31,240,31,240,0,127,224,63,240,1,255,128,63,240,7,255,0,127,240,15,252,0,127,
 224,63,248,0,255,224,255,224,0,255,224,63,240,0,31,248,63,224,0,63,224,127,192,0,255,192,255,192,1,255,128,255,128,
 3,254,1,255,128,15,252,1,255,0,31,248,3,254,0,63,224,7,254,0,255,192,7,252,1,255,0,15,248,3,254,0,31,248,15,252,0,
 31,240,31,240,0,63,224,63,224,0,63,224,255,192,0,127,193,255,0,0,255,131,254,0,0,255,143,248,0,1,255,31,240,0,1,
 254,63,224,0,3,254,255,128,0,7,253,255,0,0,7,255,252,0,0,15,255,248,0,0,15,255,240,0,0,31,255,192,0,0,63,255,128,0,
 0,63,255,0,0,0,127,252,0,0,0,127,248,0,0,0,255,224,0,0,1,255,192,0,0,1,255,128,0,0,3,254,0,0,0,7,252,0,0,0,15,248,
 0,0,0,63,224,0,0,0,127,192,0,0,1,255,0,0,0,3,254,0,0,0,31,252,0,0,31,255,240,0,0,63,255,192,0,0,127,255,128,0,0,
 255,254,0,0,1,255,248,0,0,3,255,224,0,0,7,255,0,0,0,3,255,255,255,192,7,255,255,255,128,15,255,255,255,0,31,255,
 255,254,0,63,255,255,252,0,127,255,255,248,0,255,255,255,240,0,0,0,255,224,0,0,3,255,128,0,0,15,254,0,0,0,31,248,0,
 0,0,127,224,0,0,1,255,128,0,0,7,254,0,0,0,31,248,0,0,0,127,240,0,0,1,255,192,0,0,7,255,0,0,0,31,252,0,0,0,127,240,
 0,0,0,255,192,0,0,3,255,0,0,0,15,252,0,0,0,63,248,0,0,0,255,224,0,0,3,255,128,0,0,15,254,0,0,0,63,248,0,0,0,127,
 255,255,252,0,255,255,255,248,1,255,255,255,240,3,255,255,255,224,7,255,255,255,192,15,255,255,255,128,31,255,255,
 255,0,0,0,3,255,0,0,0,63,254,0,0,1,255,252,0,0,3,255,248,0,0,15,255,240,0,0,31,255,224,0,0,127,224,0,0,0,255,128,0,
 0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,
 252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,1,255,0,0,0,3,254,0,0,0,15,248,0,0,0,127,
 240,0,0,63,255,192,0,0,127,255,0,0,0,255,248,0,0,1,255,240,0,0,3,255,248,0,0,7,255,248,0,0,0,63,248,0,0,0,31,240,0,
 0,0,31,240,0,0,0,63,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,
 0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,248,0,0,0,31,248,0,0,0,
 31,255,224,0,0,63,255,192,0,0,63,255,128,0,0,127,255,0,0,0,63,254,0,0,0,31,252,0,0,0,254,0,0,0,1,252,0,0,0,3,248,0,
 0,0,7,240,0,0,0,15,224,0,0,0,31,192,0,0,0,63,128,0,0,0,127,0,0,0,0,254,0,0,0,1,252,0,0,0,3,248,0,0,0,7,240,0,0,0,
 15,224,0,0,0,31,192,0,0,0,63,128,0,0,0,127,0,0,0,0,254,0,0,0,1,252,0,0,0,3,248,0,0,0,7,240,0,0,0,15,224,0,0,0,31,
 192,0,0,0,63,128,0,0,0,127,0,0,0,0,254,0,0,0,1,252,0,0,0,3,248,0,0,0,7,240,0,0,0,15,224,0,0,0,31,192,0,0,0,63,128,
 0,0,0,127,0,0,0,0,254,0,0,0,1,252,0,0,0,3,248,0,0,0,7,240,0,0,0,15,224,0,0,0,31,192,0,0,0,63,128,0,0,0,127,0,0,0,0,
 254,0,0,0,1,252,0,0,0,3,248,0,0,0,7,240,0,0,0,15,224,0,0,0,31,192,0,0,0,63,128,0,0,0,127,0,0,0,0,254,0,0,0,1,252,0,
 0,0,3,248,0,0,0,7,240,0,0,0,15,224,0,0,0,31,192,0,0,0,63,128,0,0,0,127,0,0,0,0,254,0,0,0,1,252,0,0,0,3,248,0,0,0,7,
 240,0,0,0,15,224,0,0,0,31,192,0,0,0,63,128,0,0,0,127,0,0,3,255,0,0,0,7,255,192,0,0,15,255,224,0,0,31,255,192,0,0,
 63,255,192,0,0,127,255,128,0,0,1,255,128,0,0,1,255,0,0,0,3,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,
 0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,
 0,127,128,0,0,0,255,128,0,0,1,255,0,0,0,1,255,0,0,0,3,255,128,0,0,3,255,252,0,0,3,255,248,0,0,1,255,240,0,0,3,255,
 224,0,0,31,255,192,0,0,127,255,128,0,1,255,192,0,0,3,254,0,0,0,15,248,0,0,0,31,240,0,0,0,63,192,0,0,0,127,128,0,0,
 0,255,0,0,0,1,254,0,0,0,3,252,0,0,0,7,248,0,0,0,15,240,0,0,0,31,224,0,0,0,63,192,0,0,0,127,128,0,0,0,255,0,0,0,1,
 254,0,0,0,3,252,0,0,0,15,248,0,0,0,31,240,0,0,0,127,224,0,0,127,255,128,0,0,255,255,0,0,1,255,252,0,0,3,255,248,0,
 0,7,255,192,0,0,15,252,0,0,0,0,127,0,0,16,7,255,192,0,96,31,255,224,1,192,255,255,248,31,129,255,255,255,255,3,255,
 255,255,254,7,255,255,255,252,15,192,127,255,248,28,0,63,255,192,48,0,15,255,0,0,0,7,240,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0};
const GFXglyph BigGlyphs[] PROGMEM={{0,39,0,39,0,0},{0,39,47,39,0,-47},{230,39,47,39,0,-47},{460,39,47,39,0,-47},
 {690,39,57,39,0,-48},{968,39,45,39,0,-45},{1188,39,49,39,0,-48},{1427,39,47,39,0,-47},{1657,39,57,39,0,-49},{1935,
 39,57,39,0,-49},{2213,39,48,39,0,-48},{2447,39,36,39,0,-36},{2623,39,20,39,0,-11},{2721,39,23,39,0,-23},{2834,39,
 11,39,0,-11},{2888,39,53,39,0,-47},{3147,39,49,39,0,-48},{3386,39,47,39,0,-47},{3616,39,48,39,0,-48},{3850,39,49,
 39,0,-48},{4089,39,47,39,0,-47},{4319,39,48,39,0,-47},{4553,39,49,39,0,-48},{4792,39,47,39,0,-47},{5022,39,49,39,0,
 -48},{5261,39,49,39,0,-48},{5500,39,33,39,0,-33},{5661,39,42,39,0,-33},{5866,39,37,39,0,-37},{6047,39,31,39,0,-31},
 {6199,39,37,39,0,-37},{6380,39,48,39,0,-48},{6614,39,54,39,0,-44},{6878,39,47,39,0,-47},{7108,39,47,39,0,-47},
 {7338,39,49,39,0,-48},{7577,39,47,39,0,-47},{7807,39,47,39,0,-47},{8037,39,47,39,0,-47},{8267,39,49,39,0,-48},
 {8506,39,47,39,0,-47},{8736,39,47,39,0,-47},{8966,39,48,39,0,-47},{9200,39,47,39,0,-47},{9430,39,47,39,0,-47},
 {9660,39,47,39,0,-47},{9890,39,47,39,0,-47},{10120,39,49,39,0,-48},{10359,39,47,39,0,-47},{10589,39,57,39,0,-48},
 {10867,39,47,39,0,-47},{11097,39,49,39,0,-48},{11336,39,47,39,0,-47},{11566,39,48,39,0,-47},{11800,39,47,39,0,-47},
 {12030,39,47,39,0,-47},{12260,39,47,39,0,-47},{12490,39,47,39,0,-47},{12720,39,47,39,0,-47},{12950,39,57,39,0,-49},
 {13228,39,53,39,0,-47},{13487,39,57,39,0,-49},{13765,39,47,39,0,-47},{13995,39,15,39,0,0},{14069,39,51,39,0,-51},
 {14318,39,37,39,0,-36},{14499,39,50,39,0,-49},{14743,39,37,39,0,-36},{14924,39,50,39,0,-49},{15168,39,37,39,0,-36},
 {15349,39,49,39,0,-49},{15588,39,49,39,0,-36},{15827,39,49,39,0,-49},{16066,39,52,39,0,-52},{16320,39,65,39,0,-52},
 {16637,39,49,39,0,-49},{16876,39,49,39,0,-49},{17115,39,36,39,0,-36},{17291,39,36,39,0,-36},{17467,39,37,39,0,-36},
 {17648,39,49,39,0,-36},{17887,39,49,39,0,-36},{18126,39,36,39,0,-36},{18302,39,37,39,0,-36},{18483,39,45,39,0,-45},
 {18703,39,36,39,0,-35},{18879,39,35,39,0,-35},{19050,39,35,39,0,-35},{19221,39,35,39,0,-35},{19392,39,48,39,0,-35},
 {19626,39,35,39,0,-35},{19797,39,59,39,0,-49},{20085,39,64,39,0,-49},{20397,39,59,39,0,-49},{20685,39,26,39,0,
 -26}};
const GFXfont BigFont PROGMEM={(uint8_t*)BigBits,(GFXglyph*)BigGlyphs,32,126,67};

// ------------------------------- Graphics ---------------------------------

// Shared main numbers: 33 px tall, with room inside the CO2 eye.
const uint8_t MainNumberFontBits[] PROGMEM={
 7,255,0,31,252,0,127,240,1,255,192,7,255,0,31,252,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,252,0,3,
 240,0,15,192,0,63,0,0,252,0,3,240,0,15,192,0,0,0,240,0,3,192,0,30,
 0,0,120,0,1,224,0,15,0,0,60,0,0,224,0,7,128,0,30,0,0,240,0,3,
 192,0,30,0,0,120,0,1,224,0,15,0,0,60,0,0,240,0,7,128,0,30,0,0,
 240,0,3,192,0,30,0,0,120,0,1,224,0,15,0,0,60,0,0,240,0,7,128,0,
 28,0,0,112,0,3,192,0,15,0,0,120,0,1,224,0,15,0,0,0,252,0,15,252,
 0,63,240,1,255,224,7,255,128,63,63,1,248,126,7,192,248,31,3,224,124,15,129,240,
 62,15,128,124,62,1,240,248,7,195,227,31,15,159,124,62,125,240,249,247,195,227,159,15,
 140,124,62,1,240,248,7,193,240,62,7,192,248,31,3,224,124,15,129,248,126,3,243,240,
 7,255,128,31,254,0,63,240,0,255,192,0,252,0,1,252,0,127,240,1,255,192,7,255,
 0,31,252,0,125,240,1,135,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,
 124,0,1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,0,31,0,0,124,0,
 1,240,0,7,192,0,31,0,0,124,0,1,240,0,7,192,7,255,252,31,255,240,127,255,
 193,255,255,7,255,252,1,248,0,127,252,1,255,248,7,255,240,31,255,192,124,63,129,192,
 126,6,0,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,31,128,0,252,0,
 3,224,0,31,128,0,252,0,3,224,0,63,128,0,252,0,7,224,0,31,0,0,248,0,
 7,224,0,63,0,1,248,0,7,192,0,63,255,224,255,255,131,255,254,15,255,248,63,255,
 224,3,248,0,127,252,1,255,240,7,255,224,31,255,192,112,127,1,0,254,0,0,248,0,
 3,224,0,15,128,0,62,0,3,240,0,31,128,7,252,0,31,240,0,127,128,1,255,0,
 7,254,0,1,252,0,3,248,0,3,224,0,15,128,0,62,0,0,248,0,3,224,0,15,
 131,128,126,15,131,248,63,255,192,255,255,3,255,248,15,255,192,3,248,0,0,31,128,0,
 126,0,3,248,0,15,224,0,127,128,3,254,0,15,248,0,123,224,1,239,128,7,190,0,
 60,248,0,243,224,7,143,128,28,62,0,240,248,7,195,224,30,15,128,120,62,3,192,248,
 15,3,224,63,255,240,255,255,195,255,255,15,255,252,63,255,240,0,62,0,0,248,0,3,
 224,0,15,128,0,62,0,0,248,0,3,224,31,255,128,127,254,1,255,248,7,255,224,31,
 255,128,120,0,1,224,0,7,128,0,30,0,0,120,0,1,224,0,7,190,0,31,254,0,
 127,252,1,255,248,7,255,224,31,255,192,64,63,128,0,126,0,0,248,0,3,224,0,15,
 128,0,62,0,0,248,0,3,224,0,31,130,0,254,14,7,240,63,255,128,255,254,3,255,
 240,15,255,128,3,240,0,0,62,0,7,255,0,63,252,1,255,240,7,255,192,63,3,0,
 248,4,7,192,0,31,0,0,124,0,1,240,0,15,143,128,62,127,0,255,254,3,255,252,
 15,255,248,63,207,224,254,15,131,240,63,15,192,252,63,1,240,252,7,193,240,31,7,192,
 124,31,1,240,124,15,193,240,62,7,224,248,15,255,224,31,255,0,63,248,0,255,192,0,
 124,0,63,255,224,255,255,131,255,254,15,255,248,63,255,224,0,31,128,0,126,0,3,240,
 0,15,192,0,62,0,0,248,0,7,224,0,31,0,0,124,0,3,240,0,15,192,0,62,
 0,0,248,0,7,224,0,63,0,0,252,0,3,240,0,31,128,0,126,0,1,240,0,7,
 192,0,63,0,0,248,0,3,224,0,31,128,0,126,0,1,240,0,0,252,0,15,252,0,
 127,240,3,255,224,31,255,192,124,31,1,240,62,7,128,248,30,3,224,120,15,129,224,62,
 7,192,240,31,7,192,31,254,0,127,240,0,255,128,7,255,0,31,254,1,248,252,7,193,
 240,30,3,224,248,7,131,224,30,15,128,120,62,1,224,248,15,131,240,62,7,193,248,31,
 255,192,127,255,0,127,248,1,255,192,0,252,0,1,240,0,31,248,0,127,240,7,255,224,
 31,255,128,252,63,3,224,124,15,129,248,62,3,224,248,15,131,224,62,15,128,248,62,3,
 224,248,31,131,224,126,15,231,248,63,255,224,127,255,129,255,254,3,255,248,7,243,224,7,
 143,128,0,62,0,0,248,0,7,192,0,31,1,0,248,6,7,224,31,255,128,127,252,1,
 255,224,7,255,0,3,224,0,
};
const GFXglyph MainNumberFontGlyphs[] PROGMEM={
 {0,22,16,22,0,-16},
 {44,22,7,22,0,-7},
 {64,22,36,22,0,-32},
 {163,22,33,22,0,-33},
 {254,22,32,22,0,-32},
 {342,22,33,22,0,-33},
 {433,22,33,22,0,-33},
 {524,22,32,22,0,-32},
 {612,22,33,22,0,-32},
 {703,22,33,22,0,-33},
 {794,22,32,22,0,-32},
 {882,22,33,22,0,-33},
 {973,22,33,22,0,-33},
};
const GFXfont MainNumberFont PROGMEM={(uint8_t*)MainNumberFontBits,(GFXglyph*)MainNumberFontGlyphs,45,57,46};
// Environment values and units preserve NumberFont glyph height/strokes.
// Trim empty side bearings only, so full-size inHg fits alongside BARO.
const uint8_t EnvValueFontBits[] PROGMEM={
 31,0,1,252,0,31,240,1,227,192,14,14,0,112,112,3,131,128,30,60,0,127,192,49,
 252,7,135,192,240,0,30,0,7,192,0,240,0,30,0,3,192,248,120,15,226,0,255,128,
 15,30,0,112,112,3,131,128,28,28,0,241,224,3,254,0,15,224,0,62,0,255,255,255,
 255,255,255,254,0,0,0,0,0,0,0,0,0,0,0,255,255,255,255,240,7,240,7,252,
 7,255,7,255,199,227,243,224,249,240,125,240,31,248,15,252,7,254,3,255,49,255,188,255,
 222,127,230,63,240,31,248,15,252,7,254,3,207,131,231,193,243,241,248,255,248,63,248,15,
 248,3,240,0,15,224,63,240,31,248,15,252,7,62,0,31,0,15,128,7,192,3,224,1,
 240,0,248,0,124,0,62,0,31,0,15,128,7,192,3,224,1,240,0,248,0,124,0,62,
 0,31,3,255,255,255,255,255,255,255,255,192,31,240,63,254,31,255,143,255,231,131,250,0,
 252,0,62,0,31,0,15,128,7,192,7,192,7,224,7,224,3,224,3,224,3,224,3,224,
 3,224,3,224,3,224,3,224,3,224,3,255,255,255,255,255,255,255,255,192,31,224,63,252,
 31,255,143,255,199,255,243,1,248,0,124,0,62,0,31,0,31,1,255,128,255,0,127,0,
 63,224,31,248,0,126,0,31,128,7,192,3,224,1,252,3,255,255,255,255,253,255,252,255,
 252,31,248,0,0,126,0,31,128,15,224,7,248,1,254,0,255,128,63,224,30,248,15,62,
 3,207,129,227,224,240,248,60,62,30,15,143,131,227,192,248,255,255,255,255,255,255,255,255,
 255,0,62,0,15,128,3,224,0,248,0,62,0,15,128,127,254,63,255,31,255,143,255,199,
 128,3,192,1,224,0,240,0,120,0,63,240,31,254,15,255,199,255,243,3,248,0,126,0,
 31,0,15,128,7,192,3,224,1,248,1,247,3,251,255,253,255,252,255,248,31,240,0,3,
 252,7,255,7,255,135,255,195,240,99,224,17,240,0,240,0,249,248,125,254,63,255,159,255,
 239,227,247,224,255,224,63,240,31,248,15,252,7,254,3,239,1,247,193,243,241,248,255,252,
 63,252,15,252,1,248,0,255,255,255,255,255,255,255,255,240,1,248,0,248,0,252,0,124,
 0,126,0,63,0,31,0,31,128,15,128,15,192,7,192,7,224,3,224,1,240,1,248,0,
 248,0,252,0,124,0,126,0,62,0,31,0,31,128,0,7,240,15,254,15,255,135,255,199,
 227,243,224,249,240,124,248,62,124,31,31,31,7,255,1,255,1,255,193,255,241,248,252,248,
 62,248,15,252,7,254,3,255,1,255,193,251,241,249,255,252,127,252,31,252,3,248,0,7,
 224,15,252,15,255,15,255,199,227,247,224,251,224,61,240,31,248,15,252,7,254,3,255,1,
 255,193,251,241,253,255,254,127,255,31,239,135,231,192,3,192,3,226,1,241,131,240,255,248,
 127,248,63,248,15,224,0,255,255,255,255,255,255,255,255,255,255,248,0,248,0,248,0,248,
 0,248,0,255,254,255,254,255,254,255,254,255,254,248,0,248,0,248,0,248,0,248,0,248,
 0,248,0,248,0,248,0,248,0,248,0,248,15,252,7,254,3,255,1,255,128,255,192,127,
 224,63,240,31,248,15,252,7,255,255,255,255,255,255,255,255,255,255,255,240,31,248,15,252,
 7,254,3,255,1,255,128,255,192,127,224,63,240,31,248,15,252,7,192,7,207,143,247,207,
 255,239,255,247,227,251,224,255,240,127,240,31,248,15,252,7,254,3,255,1,255,128,255,224,
 253,240,126,252,127,127,255,159,255,199,251,225,249,240,0,248,0,124,192,252,127,254,63,254,
 31,254,3,252,0,3,224,1,240,0,248,0,124,0,62,0,31,0,0,0,0,0,0,0,
 63,240,31,248,15,252,7,254,0,31,0,15,128,7,192,3,224,1,240,0,248,0,124,0,
 62,0,31,0,15,128,7,192,3,224,127,255,255,255,255,255,255,255,248,248,248,251,252,255,
 254,255,254,254,63,252,31,248,31,248,31,248,31,248,31,248,31,248,31,248,31,248,31,248,
 31,248,31,248,31,248,31,248,31,248,31,
};
const GFXglyph EnvValueFontGlyphs[] PROGMEM={
 {0,22,0,22,0,0},
 {0,0,0,8,0,0},
 {0,0,0,8,0,0},
 {0,0,0,8,0,0},
 {0,0,0,8,0,0},
 {0,21,26,23,0,-26},
 {69,0,0,8,0,0},
 {69,0,0,8,0,0},
 {69,0,0,8,0,0},
 {69,0,0,8,0,0},
 {69,0,0,8,0,0},
 {69,0,0,8,0,0},
 {69,0,0,8,0,0},
 {69,11,13,13,0,-13},
 {87,6,6,8,0,-6},
 {92,0,0,8,0,0},
 {92,17,26,19,0,-26},
 {148,17,26,19,0,-26},
 {204,17,26,19,0,-26},
 {260,17,26,19,0,-26},
 {316,18,26,20,0,-26},
 {375,17,26,19,0,-26},
 {431,17,26,19,0,-26},
 {487,17,26,19,0,-26},
 {543,17,26,19,0,-26},
 {599,17,26,19,0,-26},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,0,0,8,0,0},
 {655,16,26,18,0,-26},
 {707,0,0,8,0,0},
 {707,17,26,19,0,-26},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,0,0,8,0,0},
 {763,17,27,19,0,-20},
 {821,0,0,8,0,0},
 {821,17,29,19,0,-29},
 {883,0,0,8,0,0},
 {883,0,0,8,0,0},
 {883,0,0,8,0,0},
 {883,0,0,8,0,0},
 {883,16,20,18,0,-20},
};
const GFXfont EnvValueFont PROGMEM={(uint8_t*)EnvValueFontBits,(GFXglyph*)EnvValueFontGlyphs,32,110,39};
constexpr uint16_t rgb(int r,int g,int b) { return ((r&248)<<8)|((g&252)<<3)|(b>>3); }
constexpr uint16_t BLACK=rgb(4,8,13), WHITE=rgb(236,245,245), CYAN=rgb(102,222,240);
constexpr uint16_t DIM=rgb(88,127,144), GRID=rgb(20,42,55), DARK=rgb(11,24,34);
constexpr uint16_t AMBER=rgb(239,185,93), GREEN=rgb(132,218,165), PINK=rgb(243,132,147);
uint16_t mix(uint16_t a,uint16_t b,float f) {
 f=clipf(f,0,1);
 int ar=(a>>11)&31, ag=(a>>5)&63, ab=a&31;
 int br=(b>>11)&31, bg=(b>>5)&63, bb=b&31;
 return ((int)(ar+(br-ar)*f)<<11)|((int)(ag+(bg-ag)*f)<<5)|(int)(ab+(bb-ab)*f);
}
uint16_t brighterText(uint16_t color) {
 // Original +20%, then an additional +30%: 1.20 * 1.30 = 1.56.
 // Round once in RGB565 and saturate channels at full intensity.
 const int r=clipi((((color>>11)&31)*39+12)/25,0,31);
 const int g=clipi((((color>>5)&63)*39+12)/25,0,63);
 const int b=clipi(((color&31)*39+12)/25,0,31);
 return (r<<11)|(g<<5)|b;
}
int textWidth(Adafruit_GFX& d,const char* s,const GFXfont* f) {
 d.setFont(f); d.setTextSize(1); int16_t x,y; uint16_t w,h;
 d.getTextBounds(s,0,0,&x,&y,&w,&h); return w;
}
void text(Adafruit_GFX& d,int x,int y,const char* s,const GFXfont* f=&LabelFont,uint16_t c=WHITE) {
 d.setTextWrap(false); d.setFont(f); d.setTextSize(1); d.setTextColor(brighterText(c));
 int16_t bx,by; uint16_t w,h; d.getTextBounds(s,0,0,&bx,&by,&w,&h);
 d.setCursor(x-bx,y-by); d.print(s);
}
void center(Adafruit_GFX& d,int x,int y,const char* s,const GFXfont* f=&LabelFont,uint16_t c=WHITE) {
 text(d,x-textWidth(d,s,f)/2,y,s,f,c);
}
void right(Adafruit_GFX& d,int x,int y,const char* s,const GFXfont* f=&LabelFont,uint16_t c=WHITE) {
 text(d,x-textWidth(d,s,f),y,s,f,c);
}
void fmt(Adafruit_GFX& d,int x,int y,const GFXfont* f,uint16_t c,const char* format,...) {
 char b[96]; va_list args; va_start(args,format); vsnprintf(b,sizeof(b),format,args); va_end(args);
 text(d,x,y,b,f,c);
}
void value(Adafruit_GFX& d,int x,int y,float v,int decimals=1,const GFXfont* f=&BigFont,
           uint16_t c=WHITE,int maxWidth=216) {
 char b[32];
 if(isfinite(v)) snprintf(b,sizeof(b),"%.*f",decimals,(double)v); else strcpy(b,"--");
 if(textWidth(d,b,f)>maxWidth) f=&NumberFont;
 if(textWidth(d,b,f)>maxWidth) f=&LabelFont;
 center(d,x,y,b,f,c);
}
void datum(Adafruit_GFX& d,int y,const char* name,float v,int decimals,const char* unit,
           uint16_t c=CYAN) {
 char b[36]; text(d,12,y,name,&LabelFont,WHITE);
 if(isfinite(v))snprintf(b,sizeof(b),"%.*f%s",decimals,(double)v,unit);
 else snprintf(b,sizeof(b),"--%s",unit);
 right(d,228,y,b,&LabelFont,c);
}
void row(Adafruit_GFX& d,int y,const char* name,const char* v,uint16_t c=CYAN) {
 text(d,12,y,name,&LabelFont); right(d,228,y,v,&LabelFont,c);
}
void rule(Adafruit_GFX& d,int y,uint16_t c=GRID) { d.drawFastHLine(12,y,216,c); }
void furniture(Adafruit_GFX& d) {
 // All screens share a clear background, without the old edge registration frame.
 d.fillScreen(BLACK);
}
void smallNote(Adafruit_GFX& d,int y,const char* s,uint16_t c=DIM) { center(d,120,y,s,&SmallFont,c); }
void consoleHeader(Adafruit_GFX& d,const char* s,int index,int total,bool fresh) {
 text(d,12,12,s,&LabelFont,CYAN);
 char b[32]; snprintf(b,sizeof(b),"%d/%d",detail?infoOrdinal:index+1,detail?infoTotal:total); right(d,228,14,b,&SmallFont,DIM);
 rule(d,33);
 if(!fresh) { d.fillRect(12,214,216,15,DARK); smallNote(d,216,"NO CURRENT SAMPLE",AMBER); }
}
void dot(Adafruit_GFX& d,int x,int y,bool fresh,uint64_t last) {
 float pulse=fresh?clipf(1.0f-(nowMs()-last)/450.0f,0,1):0;
 d.fillCircle(x,y,3,fresh?mix(DIM,WHITE,pulse):GRID);
}
void arc(Adafruit_GFX& d,int cx,int cy,int r,float from,float to,uint16_t c,int width=1) {
 int previousX=0,previousY=0; bool first=true;
 for(float angle=from;angle<=to+0.1f;angle+=3) {
  float a=angle*0.01745329252f;
  int x=cx+lroundf(r*cosf(a)), y=cy+lroundf(r*sinf(a));
  if(!first) { d.drawLine(previousX,previousY,x,y,c);
   for(int w=1;w<width;++w)
    d.drawLine(cx+lroundf((r-w)*cosf((angle-3)*0.01745329252f)),
     cy+lroundf((r-w)*sinf((angle-3)*0.01745329252f)),
     cx+lroundf((r-w)*cosf(a)),cy+lroundf((r-w)*sinf(a)),c);
  }
  first=false;previousX=x;previousY=y;
 }
}
void segments(Adafruit_GFX& d,int cx,int cy,int r,uint16_t c,float offset=0) {
 for(int i=0;i<8;++i) arc(d,cx,cy,r,offset+i*45+4,offset+i*45+34,c,2);
}

// ------------------------- Particle acquisition field ---------------------
struct Contact { float x=0,y=0; uint8_t kind=0; uint64_t born=0, hit=0; };
Contact contacts[38];
uint32_t rngState=0x63C3B680;
float rand01() { rngState^=rngState<<13; rngState^=rngState>>17; rngState^=rngState<<5;
 return (rngState&0xFFFFFF)/16777216.0f; }
void makeContact(Contact& c,uint64_t at) {
 float a=rand01()*6.2831853f, r=80+rand01()*23;
 c.x=r*cosf(a); c.y=r*sinf(a); c.born=at; c.hit=0;
 uint32_t n=pm.counts[0], fine=pm.counts[0]-pm.counts[2], medium=pm.counts[2]-pm.counts[3];
 float pick=rand01()*n; c.kind=(pick<fine)?0:((pick<fine+medium)?1:2);
}
int contactCount() {
 if(!pmFresh() || !pm.countsOK || pm.counts[0]==0)return 0;
 return clipi((int)ceilf(38*log1pf(pm.counts[0])/log1pf(12000.0f)),1,38);
}
// Four primary instruments. Numeric bars are display ranges, not safety limits.
void mainHeader(Adafruit_GFX& d,const char* title){
 d.fillRect(12,12,3,18,PINK);text(d,23,12,title,&LabelFont,WHITE);rule(d,35);
}
void rangeMarker(Adafruit_GFX& d,int x,int y){
 d.fillTriangle(x-4,y-6,x+4,y-6,x,y-1,WHITE);d.drawFastVLine(x,y,11,WHITE);
}
void mainBar(Adafruit_GFX& d,float v,float lo,float hi,const char* left,const char* middle,const char* end,bool iaqBands=false){
 const int x=12,y=201,w=216;
 if(iaqBands){
  const int stops[8]={0,50,100,150,200,250,350,500};
  const uint16_t colors[7]={GREEN,rgb(168,203,103),AMBER,rgb(230,146,72),PINK,rgb(181,94,136),rgb(121,65,118)};
  for(int i=0;i<7;++i){int a=x+w*stops[i]/500,b=x+w*stops[i+1]/500;d.fillRect(a,y,b-a-1,9,colors[i]);}
 }else{d.fillRect(x,y,w,9,GRID);if(isfinite(v))d.fillRect(x,y,lroundf(w*clipf((v-lo)/(hi-lo),0,1)),9,CYAN);}
 if(isfinite(v))rangeMarker(d,x+lroundf((w-1)*clipf((v-lo)/(hi-lo),0,1)),y);
 text(d,x,216,left,&SmallFont,DIM);center(d,120,216,middle,&SmallFont,DIM);right(d,228,216,end,&SmallFont,DIM);
 if(isfinite(v)&&(v<lo||v>hi))right(d,228,181,v<lo?"BELOW SCALE":"ABOVE SCALE",&SmallFont,AMBER);
}
void particleMain(Adafruit_GFX& d){
 mainHeader(d,"PARTICLE COUNT");bool fresh=pmFresh()&&pm.countsOK;uint64_t n=nowMs();
 const int cx=120,cy=111;
 for(int r:{32,50,68})arc(d,cx,cy,r,0,360,GRID);
 float angle=(n%2800)*6.2831853f/2800.0f;
 for(int j=0;j<8;++j){float a=angle-j*.024f;d.drawLine(cx,cy,cx+lroundf(67*cosf(a)),cy+lroundf(67*sinf(a)),mix(BLACK,fresh?CYAN:DIM,(8-j)/14.0f));}
 if(fresh)for(int i=0;i<contactCount();++i){
  Contact& c=contacts[i];if(!c.born||n-c.born>10000+(i%5)*700)makeContact(c,n);
  float a=atan2f(c.y,c.x);if(a<0)a+=6.2831853f;float crossed=angle-a;if(crossed<0)crossed+=6.2831853f;
  if(crossed<.22f)c.hit=n;float glow=c.hit?expf(-(n-c.hit)/650.0f):0;
  int x=cx+lroundf(c.x*.65f),y=cy+lroundf(c.y*.65f);uint16_t col=mix(DIM,WHITE,glow);
  if(c.kind==0)d.drawPixel(x,y,col);else if(c.kind==1)d.fillCircle(x,y,1,col);else d.drawCircle(x,y,2,col);
 }
 d.fillRect(22,78,196,66,BLACK);value(d,120,89,fresh?pm.counts[0]:NAN,0,&MainNumberFont,WHITE,194);
 center(d,120,134,"PARTICLES / 0.1 L",&SmallFont,CYAN);
 center(d,120,158,">0.3 um",&LabelFont,WHITE);
 text(d,12,182,fresh?"COUNT SCALE":pmFresh()?"COUNT DATA INVALID":"PMS WAIT / WARMUP",&SmallFont,fresh?DIM:AMBER);
 mainBar(d,fresh?pm.counts[0]:NAN,0,10000,"0","5,000","10,000");
}
void co2Main(Adafruit_GFX& d){
 mainHeader(d,"CO2");bool fresh=co2Fresh();float amount=fresh?clipf((co2.ppm-400)/1600,0,1):0;
 float motion=fresh?1.3f*sinf((nowMs()%8000)*6.2831853f/8000):0;
 // Center between the title's bottom at y=25 and the caption's top at y=182.
 // The ring spans y=43..163, leaving about 18 clear rows on either side.
 constexpr int cx=120,cy=103,outerRadius=60,spokeRadius=57;
 segments(d,cx,cy,outerRadius,DIM);float r=54-amount*5+motion;
 for(int j=0;j<6;++j){float a=j*60+15,q=a*.01745329252f;
  arc(d,cx,cy,(int)r,a,a+42,CYAN,2);
  d.drawLine(cx+lroundf(spokeRadius*cosf(q)),cy+lroundf(spokeRadius*sinf(q)),cx+lroundf(r*cosf(q+.12f)),cy+lroundf(r*sinf(q+.12f)),DIM);}
 // Preserve readable five-digit outliers without enlarging the circle.
 const GFXfont* numberFont=fresh&&co2.ppm>=10000?&EnvValueFont:&MainNumberFont;
 value(d,cx,cy-25,fresh?co2.ppm:NAN,0,numberFont,WHITE,96);
 center(d,cx,cy+16,"ppm",&LabelFont,CYAN);
 text(d,12,182,fresh?"CONCENTRATION / ppm":"SCD41 WAIT / STALE",&SmallFont,fresh?DIM:AMBER);
 mainBar(d,fresh?co2.ppm:NAN,400,5000,"400","2,700","5,000");
}
void gasMainRibbons(Adafruit_GFX& d,bool live){
 // Decorative mixed-VOC contours, not separate gas measurements or sampled waveforms.
 // Keep every line outside the numeric field and the scale/caption below it.
 const float phase=live?(nowMs()%14000)*6.2831853f/14000.0f:0;
 const uint16_t colors[5]={CYAN,AMBER,CYAN,rgb(178,137,240),CYAN};
 for(int band=0;band<2;++band)for(int strand=0;strand<5;++strand){
  const int base=(band?147:48)+strand*6;
  int px=18,py=base;
  for(int x=21;x<=222;x+=3){
   const float u=(x-18)/204.0f,envelope=4*u*(1-u);
   const float wave=3.5f*sinf(u*6.2831853f+phase+strand*.45f+band*1.7f)
    +1.5f*sinf(u*12.566371f-phase+strand*.7f);
   const int y=base+lroundf(envelope*wave);
   const float glow=(strand==2?.48f:strand%2?.32f:.29f)*1.69f*envelope; // Another +30%.
   d.drawLine(px,py,x,y,mix(BLACK,live?colors[strand]:DIM,live?glow:glow*.40f));
   px=x;py=y;
  }
 }
}
void gasMain(Adafruit_GFX& d){
 mainHeader(d,"VoC AIR QUALITY");
 const bool ready=iaqReady();gasMainRibbons(d,ready);
 value(d,120,92,ready?iaq.index:NAN,0,&MainNumberFont,WHITE,196);
 // Calibration levels belong to the information page. Explain unavailable data only.
 if(!ready) {
  const char* status=!iaq.initialized?"SENSOR INITIALIZING / FAULT":
   !iaqFresh()?"WAITING FOR GAS SAMPLE":"WAITING FOR VALID READING";
  d.fillRect(12,149,216,20,BLACK);smallNote(d,153,status,AMBER);
 }
 text(d,12,182,"INDOOR GAS INDEX",&SmallFont,DIM);
 mainBar(d,ready?iaq.index:NAN,0,500,"0","250","500",true);
}
struct PressureTrend {bool ready=false;int direction=0;float fiveMinuteChange=NAN;};
PressureTrend pressureTrend(){
 PressureTrend result;if(!envFresh())return result;
 const uint64_t at=nowMs();uint64_t first=0,last=0;int count=0;
 double meanX=0,meanY=0,sxx=0,sxy=0;
 for(int i=0;i<envHistory.count;++i){const EnvSample& a=envHistory.at(i);
  if(a.ms>at||at-a.ms>300000||!isfinite(a.p))continue;
  // Restart the fit after missing data. No direction is inferred over a sensor outage.
  if(count&&a.ms-last>15000){count=0;meanX=meanY=sxx=sxy=0;}
  if(!count)first=a.ms;last=a.ms;++count;
  double x=-(double)(at-a.ms)/1000.0,y=a.p-100000.0;
  double dx=x-meanX,dy=y-meanY;meanX+=dx/count;meanY+=dy/count;
  sxx+=dx*(x-meanX);sxy+=dx*(y-meanY);
 }
 if(count<20||last-first<120000||at-last>ENV_STALE_MS||sxx<=0)return result;
 result.ready=true;result.fiveMinuteChange=(float)(sxy/sxx*300.0/3386.389);
 // UI noise deadband (0.002 inHg per five minutes), not a meteorological category.
 result.direction=result.fiveMinuteChange>0.002f?1:result.fiveMinuteChange<-.002f?-1:0;
 return result;
}
void pressureBar(Adafruit_GFX& d,float pressure,int y){
 const int left=78,rightEdge=195;d.fillRect(left,y+2,118,8,GRID);
 if(!isfinite(pressure))return;
 int w=lroundf(117*clipf((pressure-28)/3,0,1)),marker=left+w;
 d.fillRect(left,y+2,w,8,mix(GRID,CYAN,.40f));PressureTrend t=pressureTrend();
 // Fade length encodes direction only; the white marker retains the exact pressure scale.
 if(t.ready&&t.direction){for(int j=17;j>=0;--j){int x=marker-t.direction*j;
  if(x>=left&&x<=rightEdge)d.drawFastVLine(x,y+2,8,mix(GRID,CYAN,1.0f-j/19.0f));}}
 else if(t.ready)d.fillRect(left,y+2,w,8,CYAN);
 rangeMarker(d,marker,y+2);
 // Direction glyph sits in the existing gap before the 31 inHg endpoint.
 if(!t.ready){text(d,200,y,"?",&SmallFont,AMBER);return;}
 const int x=204;
 if(t.direction>0){d.drawFastVLine(x,y+2,8,CYAN);d.drawLine(x-3,y+5,x,y+2,CYAN);d.drawLine(x+3,y+5,x,y+2,CYAN);}
 else if(t.direction<0){d.drawFastVLine(x,y+2,8,CYAN);d.drawLine(x-3,y+6,x,y+9,CYAN);d.drawLine(x+3,y+6,x,y+9,CYAN);}
 else d.drawFastHLine(x-3,y+5,7,CYAN);
}

void environmentReading(Adafruit_GFX& d,int top,float reading,int decimals,const char* unit) {
 char b[32];if(isfinite(reading))snprintf(b,sizeof(b),"%.*f",decimals,(double)reading);else strcpy(b,"--");
 // One font and one baseline for each number/unit pair. Reserve the same unit
 // column for F, %, and inHg; no unit is reduced to fit.
 const GFXfont* f=&EnvValueFont;const int unitX=228-textWidth(d,"inHg",f),baseline=top+26;
 int16_t bx,by;uint16_t w,h;d.setFont(f);d.setTextSize(1);
 d.getTextBounds(b,0,0,&bx,&by,&w,&h);
 text(d,unitX-8-w,baseline+by,b,f,WHITE);
 d.getTextBounds(unit,0,0,&bx,&by,&w,&h);
 text(d,unitX,baseline+by,unit,f,CYAN);
}
void environmentMain(Adafruit_GFX& d){
 mainHeader(d,"ENVIRONMENT");bool fresh=envFresh();
 const float temperature=environmentTemperatureF(), humidity=environmentHumidity();
 text(d,12,52,"TEMP",&LabelFont,DIM);environmentReading(d,47,temperature,0,"F");
 text(d,12,96,"RH",&LabelFont,DIM);environmentReading(d,90,humidity,0,"%");
 text(d,12,138,"BARO",&LabelFont,DIM);environmentReading(d,133,fresh?env.p/3386.389f:NAN,2,"inHg");
 const float vals[3]={temperature,humidity,fresh?env.p/3386.389f:NAN};
 const float lo[3]={40,0,28},hi[3]={100,100,31};const char* labels[3]={"F","%RH","inHg"};
 for(int i=0;i<3;++i){int y=180+i*20;char b[12];text(d,12,y,labels[i],&SmallFont,DIM);
  snprintf(b,sizeof(b),"%.0f",lo[i]);right(d,70,y,b,&SmallFont,DIM);snprintf(b,sizeof(b),"%.0f",hi[i]);right(d,228,y,b,&SmallFont,DIM);
  if(i==2){pressureBar(d,vals[i],y);continue;}
  d.fillRect(78,y+2,118,8,GRID);if(isfinite(vals[i])){int w=lroundf(117*clipf((vals[i]-lo[i])/(hi[i]-lo[i]),0,1));d.fillRect(78,y+2,w,8,CYAN);rangeMarker(d,78+w,y+2);}
 }
}

void radar(Adafruit_GFX& d) {
 bool fresh=pmFresh(); uint64_t n=nowMs();
 const int cx=120,cy=113;
 segments(d,cx,cy,110,DIM); arc(d,cx,cy,103,0,360,GRID);
 arc(d,cx,cy,77,0,360,GRID);
 // Geometric markings are deliberately unlabeled: this is not azimuth/range.
 for(int a=0;a<360;a+=90) {
  float q=a*0.01745329252f;
  d.drawLine(cx+lroundf(98*cosf(q)),cy+lroundf(98*sinf(q)),
             cx+lroundf(104*cosf(q)),cy+lroundf(104*sinf(q)),DIM);
 }
 float angle=(n%2800)*6.2831853f/2800.0f;
 for(int i=9;i>=0;--i) {
  float a=angle-i*0.018f;
  d.drawLine(cx+lroundf(76*cosf(a)),cy+lroundf(76*sinf(a)),
   cx+lroundf(101*cosf(a)),cy+lroundf(101*sinf(a)),mix(BLACK,fresh?CYAN:DIM,(10-i)/12.0f));
 }
 int count=contactCount();
 static uint64_t lastDraw=0;
 float dt=lastDraw?clipf((n-lastDraw)/1000.0f,0,0.15f):0; lastDraw=n;
 for(int i=0;i<count;++i) {
  Contact& c=contacts[i];
  if(!c.born || n-c.born>10000ULL+(i%5)*700ULL)makeContact(c,n);
  c.x+=(INTAKE_ON_LEFT?1:-1)*dt*1.4f;
  float r2=c.x*c.x+c.y*c.y;
  if(r2>104*104 || r2<79*79)makeContact(c,n);
  float a=atan2f(c.y,c.x); if(a<0)a+=6.2831853f;
  float crossed=angle-a; if(crossed<0)crossed+=6.2831853f;
  if(crossed<0.22f)c.hit=n;
  float light=c.hit?expf(-(n-c.hit)/650.0f):0;
  int x=cx+lroundf(c.x),y=cy+lroundf(c.y);
  uint16_t col=mix(DIM,WHITE,light);
  if(c.kind==0)d.drawPixel(x,y,col);
  else if(c.kind==1)d.fillCircle(x,y,1,col);
  else d.drawCircle(x,y,2,col);
  if(light>0.7f && c.kind>0) { d.drawFastHLine(x-4,y,2,CYAN); d.drawFastHLine(x+3,y,2,CYAN); }
 }
 // On-screen intake convention; does not represent a measured flow direction.
 int sx=INTAKE_ON_LEFT?10:229, sign=INTAKE_ON_LEFT?1:-1;
 d.drawLine(sx,102,sx+sign*7,105,DIM); d.drawLine(sx,124,sx+sign*7,121,DIM);
 if(fresh) { int x=sx+sign*(int)((n/75)%13); d.drawFastVLine(x,110,7,CYAN); }
 if(fresh && pm.eventMs && n-pm.eventMs<1400)
  arc(d,cx,cy,88+(n-pm.eventMs)*21/1400,0,360,mix(CYAN,BLACK,(n-pm.eventMs)/1400.0f));
 // Central masking guarantees that contact marks never obscure the reading.
 d.fillRoundRect(42,75,156,85,10,BLACK);
 value(d,120,79,fresh?pm.pm25:NAN,0,&BigFont,WHITE,150);
 center(d,120,142,"PM2.5  ug/m3",&SmallFont,CYAN);
 smallNote(d,225,fresh?(pm.countsOK?"AIR SAMPLE":"COUNT DATA INVALID"):(nowMs()-pmsStartedMs<PMS_WARMUP_MS?"PMS WARMING / 30 SEC":"PMS WAITING / STALE"),fresh?DIM:AMBER);
}

// --------------------------- CO2 eye / iris -------------------------------
void eye(Adafruit_GFX& d) {
 bool fresh=co2Fresh(); float amount=fresh?clipf((co2.ppm-400)/1600.0f,0,1):0;
 float breathe=fresh?1.5f*sinf((nowMs()%8000)*6.2831853f/8000.0f):0;
 segments(d,120,112,109,DIM); arc(d,120,112,101,20,160,GRID,2);
 arc(d,120,112,101,200,340,GRID,2);
 float r=88-amount*9+breathe;
 for(int j=0;j<6;++j) {
  float a=j*60+15, q=a*0.01745329252f;
  arc(d,120,112,(int)r,a,a+42,CYAN,3);
  int x=120+lroundf(99*cosf(q)),y=112+lroundf(99*sinf(q));
  d.drawLine(x,y,120+lroundf(r*cosf(q+0.12f)),112+lroundf(r*sinf(q+0.12f)),DIM);
 }
 arc(d,120,112,69,0,360,GRID);
 value(d,120,77,fresh?co2.ppm:NAN,0,&BigFont,WHITE,184);
 center(d,120,140,"CO2  ppm",&LabelFont,CYAN);
 smallNote(d,225,fresh?"CO2 / MEASURED":ENABLE_SCD41?"SCD41 WAITING / STALE":"SCD41 NOT ENABLED",fresh?DIM:AMBER);
}

// ------------------------------ Gas ribbon --------------------------------
struct GasRange { float lo=0,hi=1; int n=0; };
GasRange gasRange(uint64_t at) {
 GasRange r; r.lo=INFINITY; r.hi=-INFINITY;
 for(int i=0;i<gasHistory.count;++i) {
  const GasSample& s=gasHistory.at(i); if(at<s.ms || at-s.ms>120000)continue;
  r.lo=fminf(r.lo,fminf(s.raw,s.base)); r.hi=fmaxf(r.hi,fmaxf(s.raw,s.base)); ++r.n;
 }
 if(!r.n) { r.lo=0;r.hi=1;return r; }
 // Minimum span resists turning normal sensor noise into theatrical peaks.
 float span=fmaxf(r.hi-r.lo,fmaxf(2.0f,(r.hi+r.lo)*0.025f));
 float middle=(r.hi+r.lo)*0.5f; r.lo=fmaxf(0,middle-span*0.65f);r.hi=middle+span*0.65f;
 return r;
}
int plotY(float v,float lo,float hi,int top,int bottom) {
 return bottom-lroundf(clipf((v-lo)/(hi-lo),0,1)*(bottom-top));
}
void gasRibbon(Adafruit_GFX& d) {
 uint64_t n=nowMs(); bool fresh=gasFresh(); GasRange range=gasRange(n);
 const int left=12,rightX=228,top=43,bottom=125;
 for(int x=12;x<=228;x+=36) for(int y=43;y<=125;y+=7)d.drawPixel(x,y,GRID);
 for(int y=43;y<=125;y+=41)d.drawFastHLine(left,y,217,GRID);
 text(d,12,14,"R(t)",&LabelFont,CYAN); right(d,228,16,"120s / TIME",&SmallFont,DIM);
 if(range.n) {
  fmt(d,12,32,&SmallFont,DIM,"%.1f",range.hi);
  fmt(d,12,128,&SmallFont,DIM,"%.1f kOhm",range.lo);
  bool prev=false; int px=0,py=0,pf=0,pb=0; uint64_t pt=0;
  for(int i=0;i<gasHistory.count;++i) {
   const GasSample& s=gasHistory.at(i); if(n<s.ms || n-s.ms>120000)continue;
   int x=rightX-(int)((n-s.ms)*216/120000);
   int y=plotY(s.raw,range.lo,range.hi,top,bottom);
   int fy=plotY(s.filtered,range.lo,range.hi,top,bottom);
   int by=plotY(s.base,range.lo,range.hi,top,bottom);
   if(prev && s.ms-pt<=ENV_STALE_MS) {
    d.drawLine(px,pb,x,by,fresh?mix(BLACK,AMBER,0.65f):GRID);
    d.drawLine(px,py,x,y,fresh?CYAN:DIM);
    d.drawLine(px,pf,x,fy,fresh?WHITE:DIM);
   } else d.drawPixel(x,y,fresh?CYAN:DIM);
   prev=true; px=x;py=y;pf=fy;pb=by;pt=s.ms;
  }
  // Cursor inspects the displayed past, never invents extrapolated data.
  int cursor=12+(int)((n%6000)*216/6000);
  d.drawFastVLine(cursor,top,bottom-top,GRID);
  d.drawFastHLine(cursor-2,top,5,DIM);
  if(fresh && prev) d.fillCircle(px,py,2,CYAN);
  if(gas.eventMs && n-gas.eventMs<=120000) {
   int x=rightX-(int)((n-gas.eventMs)*216/120000);
   d.drawFastVLine(x,top,8,AMBER); d.drawFastHLine(x-4,top,9,AMBER);
   if(n-gas.eventMs<10000)right(d,228,29,"STEP EVENT",&SmallFont,AMBER);
  }
 } else center(d,120,78,"ACQUIRING HISTORY",&SmallFont,AMBER);
 value(d,108,150,fresh?gas.raw:NAN,1,&BigFont,WHITE,194);
 text(d,192,186,"kOhm",&SmallFont,CYAN);
 char b[40];
 if(!fresh)snprintf(b,sizeof(b),"GAS WAITING / STALE");
 else if(gas.samples<BASE_MIN_SAMPLES)snprintf(b,sizeof(b),"BASELINE %lu/%lu",(unsigned long)gas.samples,(unsigned long)BASE_MIN_SAMPLES);
 else snprintf(b,sizeof(b),"F/B %+.1f%%",(double)gas.deviation);
 smallNote(d,214,b,fresh?DIM:AMBER);dot(d,224,217,fresh,gas.ms);
}

// --------------------------- Climate instruments --------------------------

// Lungs screen removed. Gas IAQ is rendered by gasMain().

// --------------------- Real history, one channel at a time ----------------
const char* METRIC_NAMES[7]={"GAS / kOhm","PM2.5 / ug/m3","CO2 / ppm","Si7021 TEMP / F","RH AVERAGE / %","PRESSURE / inHg","BOSCH IAQ"};
struct PlotPoint { uint64_t ms=0;float v=NAN; };
int metricCount(int m) { return m==6?iaqHistory.count:m==0?gasHistory.count:m==1?pmHistory.count:m==2?co2History.count:m==3?temperatureHistory.count:m==4?humidityHistory.count:envHistory.count; }
PlotPoint metricPoint(int m,int i) {
 PlotPoint p;
 if(m==6) {const ScalarSample&s=iaqHistory.at(i);p.ms=s.ms;p.v=s.value;}
 else if(m==0) { const GasSample&s=gasHistory.at(i);p.ms=s.ms;p.v=s.raw; }
 else if(m==1 || m==2) { const ScalarSample&s=m==1?pmHistory.at(i):co2History.at(i);p.ms=s.ms;p.v=s.value; }
 else if(m==3) { const ScalarSample&s=temperatureHistory.at(i);p.ms=s.ms;p.v=fahrenheit(s.value); }
 else if(m==4) { const ScalarSample&s=humidityHistory.at(i);p.ms=s.ms;p.v=s.value; }
 else { const EnvSample&s=envHistory.at(i);p.ms=s.ms;p.v=s.p/3386.389f; }
 return p;
}
bool metricFresh(int m) { return m==6?iaqReady():m==0?gasFresh():m==1?pmFresh():m==2?co2Fresh():m==3?si7021Fresh():m==4?environmentHumidityFresh():envFresh(); }
void trendScreen(Adafruit_GFX& d) {
 int m=trendMetric, count=metricCount(m);uint64_t n=nowMs();
 text(d,12,15,METRIC_NAMES[m],&LabelFont,CYAN);
 if(count==0) {value(d,120,59,NAN);smallNote(d,145,"WAITING FOR HISTORY",AMBER);return;}
 uint64_t window=300000;
 float lo=INFINITY,hi=-INFINITY;int included=0;
 for(int i=0;i<count;++i) { PlotPoint p=metricPoint(m,i);if(n-p.ms>window)continue;
  lo=fminf(lo,p.v);hi=fmaxf(hi,p.v);++included; }
 if(!included) {value(d,120,59,NAN);smallNote(d,145,"HISTORY EXPIRED",AMBER);return;}
 float span=fmaxf(hi-lo,m==5?0.01f:m==4?2.0f:1.0f);
 float middle=(lo+hi)/2;lo=middle-span*0.65f;hi=middle+span*0.65f;
 PlotPoint current=metricPoint(m,count-1);
 value(d,120,48,metricFresh(m)?current.v:NAN,m>=1&&m<=4?0:m==5?2:1,&NumberFont);
 for(int x=12;x<=228;x+=54)for(int y=95;y<=177;y+=7)d.drawPixel(x,y,GRID);
 rule(d,177,DIM);bool prev=false;int px=0,py=0;uint64_t pt=0;
 for(int i=0;i<count;++i) {
  PlotPoint p=metricPoint(m,i);if(n-p.ms>window)continue;
  int x=228-(int)((n-p.ms)*216/window),y=plotY(p.v,lo,hi,95,177);
  uint32_t gap=m==1?PM_STALE_MS:m==2?CO2_STALE_MS:m==3?SI7021_STALE_MS:ENV_STALE_MS;
  if(prev && p.ms-pt<=gap)d.drawLine(px,py,x,y,metricFresh(m)?CYAN:DIM);
  else d.drawPixel(x,y,DIM);
  px=x;py=y;pt=p.ms;prev=true;
 }
 const int axisDecimals=m==3||m==4?0:2;
 fmt(d,12,187,&SmallFont,DIM,"%.*f",axisDecimals,lo);char b[28];snprintf(b,sizeof(b),"%.*f",axisDecimals,hi);right(d,228,187,b,&SmallFont,DIM);
 smallNote(d,213,metricFresh(m)?"5 MIN / AUTO SCALE":"HISTORICAL / NOT LIVE",metricFresh(m)?DIM:AMBER);
}

void serviceScreen(Adafruit_GFX& d) {
 uint64_t up=nowMs()/1000;
 if(up<360000)fmt(d,12,13,&LabelFont,GREEN,"UP %02lu:%02lu:%02lu",(unsigned long)(up/3600),
  (unsigned long)(up/60)%60,(unsigned long)up%60);
 else fmt(d,12,13,&LabelFont,GREEN,"UP %lud %02lu:%02lu",(unsigned long)(up/86400),
  (unsigned long)(up/3600)%24,(unsigned long)(up/60)%60);
 const char* names[5]={"PMS5003","BME680","SCD41","Si7021","VOC IAQ"};
 bool good[5]={pmFresh(),envFresh(),co2Fresh(),si7021Fresh(),iaqReady()};
 const char* states[5]={pmFresh()?"LIVE":"WAIT",envFresh()?"LIVE":env.present?"WAIT":"FAULT",
  co2Fresh()?"LIVE":ENABLE_SCD41?"WAIT":"OFF",si7021Fresh()?"LIVE":ENABLE_SI7021?"WAIT":"OFF",
  iaqReady()?"LIVE":iaq.initialized?"LEARN":"FAULT"};
 const char* notes[5]={"UART / CHECKSUM FRAMES",gasFresh()?"GAS + CLIMATE":"CLIMATE / GAS CHECK",
  ENABLE_SCD41?"I2C / CRC CHECKED":"NOT ENABLED","ENV TEMP + RH / 0x40","BOSCH IAQ / GAS INDEX"};
 for(int i=0;i<5;++i) {
  int y=43+i*30;d.fillRect(12,y,4,13,good[i]?GREEN:AMBER);
  text(d,24,y,names[i],&LabelFont);right(d,228,y,states[i],&LabelFont,good[i]?GREEN:AMBER);
  text(d,24,y+17,notes[i],&SmallFont,DIM);
 }
 rule(d,195);
 row(d,205,"WI-FI",WiFi.status()==WL_CONNECTED?"ONLINE":"OFFLINE",WiFi.status()==WL_CONNECTED?GREEN:DIM);
}

// -------------------- BME680 mechanism / cutaway -------------------------
// gas-info1: preserved heater and gas chamber animation. This is a schematic visualization of
// the physical sensing principle, driven by real Rgas where available.
// Particle paths and the cutaway geometry are explanatory animation, not a
// claim that the BME680 identifies individual molecules or measures heater
// temperature. heaterTargetC is the programmed heater setpoint.
#define HEATER_TARGET_C heaterTargetC
#define HEATER_DURATION_MS heaterDurationMs
void gasMechanism(Adafruit_GFX& d) {
 const bool fresh=gasFresh();
 const uint64_t n=nowMs();
 const float activity=fresh&&gas.samples>=BASE_MIN_SAMPLES?
  clipf(fabsf(gas.deviation)/12.0f,0.0f,1.0f):0.18f;
 const uint16_t metal=rgb(104,122,132);
 const uint16_t metalHi=rgb(176,194,199);
 const uint16_t mox=rgb(117,126,118);
 const uint16_t heat=env.heaterStable?AMBER:mix(DIM,AMBER,0.38f);
 consoleHeader(d,"BME // CORE",0,9,envFresh());

 // Side-view metal package and lid.
 d.fillRoundRect(29,47,182,117,13,DARK);
 d.drawRoundRect(29,47,182,117,13,metal);
 d.drawFastHLine(43,60,154,metalHi);
 d.fillRect(39,61,162,11,metal);
 d.drawFastHLine(39,72,162,GRID);
 // Central vent in the lid.
 d.fillRoundRect(101,55,38,13,6,BLACK);
 d.drawRoundRect(101,55,38,13,6,metalHi);
 center(d,169,47,"VENT",&SmallFont,DIM);
 d.drawLine(148,54,139,59,DIM);

 // MOX sensing film: porous-looking grains in a thin suspended layer.
 d.fillRoundRect(49,94,142,22,4,mox);
 d.drawRoundRect(49,94,142,22,4,metalHi);
 for(int i=0;i<24;++i) {
  int x=53+(i*37)%134, y=98+((i*19)%14);
  uint16_t c=(i%5==0)?mix(mox,CYAN,0.45f):mix(BLACK,mox,0.70f);
  d.drawCircle(x,y,1,c);
 }
 text(d,158,91,"MOX",&SmallFont,CYAN);
 d.drawLine(151,100,190,100,DIM);

 // Heater below the MOX film. Make the hot plate slimmer so the MOX layer
 // reads more clearly above it. The glow gently breathes; it is not a measured
 // temperature animation. The displayed 320 C is the configured setpoint.
 float hp=0.5f+0.5f*sinf((n%1700)*6.2831853f/1700.0f);
 uint16_t glow=mix(rgb(90,43,15),heat,0.35f+0.55f*hp);
 d.fillRoundRect(62,126,116,15,4,rgb(44,25,15));
 d.drawRoundRect(62,126,116,15,4,glow);
 d.drawLine(70,133,83,129,glow);d.drawLine(83,129,98,137,glow);
 d.drawLine(98,137,113,129,glow);d.drawLine(113,129,128,137,glow);
 d.drawLine(128,137,143,129,glow);d.drawLine(143,129,158,133,glow);
 text(d,164,120,"HEAT",&SmallFont,AMBER);
 d.drawLine(157,129,190,129,DIM);

 // Measurement path and Rs block. Static wiring is shown, but no moving
 // "electron" dot: the electrical readout is real while the internal routing
 // is only a schematic.
 d.drawLine(61,116,61,150,DIM); d.drawLine(61,150,96,150,DIM);
 d.drawLine(179,116,179,150,DIM); d.drawLine(179,150,144,150,DIM);
 d.fillRoundRect(96,143,48,21,4,BLACK); d.drawRoundRect(96,143,48,21,4,CYAN);
 center(d,120,146,"Rs",&LabelFont,CYAN);
 text(d,184,147,"MEAS",&SmallFont,DIM);
 d.drawLine(146,153,181,153,DIM);

 // Heat radiation from the hot plate into the MOX layer. These plumes shimmer
 // in place; they should feel like convection from a hot plate, not marching
 // arrows or packets translating sideways across the screen. Brightness still
 // follows the real heater-stability status bit when available.
 const float heatPhase=(n%1400)/1400.0f;
 for(int j=0;j<5;++j) {
  const int baseX=82+j*19;
  const float localPhase=heatPhase*6.2831853f + j*0.83f;
  const uint16_t hc=mix(rgb(105,45,10),AMBER,env.heaterStable?0.88f:0.40f);
  for(int k=0;k<17;++k) {
   const int yy=125-k;
   if(yy<105 || yy>125) continue;
   const float taper=1.0f-(k/17.0f);
   const float shimmer=(0.55f+0.45f*sinf(localPhase + k*0.72f));
   const int xx=baseX + (int)(2.2f*sinf(localPhase*0.8f + k*0.58f))
                        + (int)(1.4f*sinf(localPhase*1.7f + k*0.27f));
   const uint16_t pix=mix(DARK,hc,clipf(0.28f + taper*0.62f*shimmer,0.0f,1.0f));
   d.drawPixel(xx,yy,pix);
   if(k>2 && k<15 && ((k+j)&1)==0) {
    if(xx>30) d.drawPixel(xx-1,yy,mix(DARK,pix,0.42f));
    if(xx<208) d.drawPixel(xx+1,yy,mix(DARK,pix,0.42f));
   }
   if(k%5==0 && yy>107) d.drawPixel(xx,yy-1,mix(DARK,pix,0.30f));
  }
 }

 // Animated gas particles enter through the vent and interact at the MOX
 // surface. Colors distinguish animated contacts only; they are not gas IDs.
 for(int i=0;i<9;++i) {
  uint32_t cyc=(uint32_t)((n/18+i*173)%1450);
  float t=cyc/1450.0f;
  int x,y; uint16_t c=(i%3==0)?AMBER:CYAN;
  if(t<0.67f) {
   float q=t/0.67f;
   x=120+(int)(sinf(i*2.41f+q*4.5f)*(7+12*q));
   y=38+(int)(q*61);
  } else {
   float q=(t-0.67f)/0.33f;
   int dir=(i&1)?1:-1;
   x=120+dir*(int)(8+q*55);
   y=100+(int)(3*sinf(q*8+i));
  }
  int r=(i%4==0)?2:1; d.fillCircle(x,y,r,c);
  // Contact flare at the sensing layer; its brightness gets stronger when the
  // live filtered signal is farther from the local baseline.
  if(t>0.64f && t<0.78f) {
   float f=1.0f-fabsf(t-0.71f)/0.07f;
   d.drawCircle(x,y,2+(int)(activity*2),mix(mox,c,clipf(f*(0.45f+0.5f*activity),0,1)));
  }
 }
 // No animated flow arrow: the descending gas particles themselves show
 // entry through the vent without inventing another moving indicator.

 // Live measurement strip. Conductance is mathematically derived from Rgas:
 // R in kOhm -> G in microSiemens = 1000/R.
 const float conductance=fresh&&gas.raw>0?1000.0f/gas.raw:NAN;
 char leftVal[22],rightVal[22];
 if(fresh) snprintf(leftVal,sizeof(leftVal),"R %.1fk",(double)gas.raw);
 else snprintf(leftVal,sizeof(leftVal),"R --");
 if(isfinite(conductance)) snprintf(rightVal,sizeof(rightVal),"G %.1fuS",(double)conductance);
 else snprintf(rightVal,sizeof(rightVal),"G --");
 rule(d,176,DIM);
 text(d,12,184,leftVal,&LabelFont,WHITE);
 right(d,228,184,rightVal,&LabelFont,CYAN);
 fmt(d,12,208,&SmallFont,AMBER,"SET %uC / %ums",(unsigned)HEATER_TARGET_C,(unsigned)HEATER_DURATION_MS);
 const char* heatState=!env.flagsKnown?"HEATER ?":env.heaterStable?"HEATER READY":"HEATING";
 right(d,228,208,heatState,&SmallFont,env.heaterStable?GREEN:AMBER);
 smallNote(d,222,"PARTICLES SCHEMATIC / HEAT STATUS REAL",DIM);
}

// -------------------------- The engineering layer -------------------------
// Each equation here uses the exact coefficients/operands stored at acquisition.
// It is a live snapshot; the values are not recomputed with a different clock.
void gasDetails(Adafruit_GFX& d,int section,int displayedIndex=-1,int displayedTotal=8) {
 bool fresh=gasFresh();
 const char* titles[8]={"BME // RAW","BME // FILTER","BME // BASE","BME // RATIO",
  "BME // DEW","BME // WATER","BME // SENSOR","BME // STATS"};
 consoleHeader(d,titles[section],displayedIndex>=0?displayedIndex:section,displayedTotal,section==4||section==5?envFresh():fresh);
 switch(section) {
 case 0:
  datum(d,48,"T",envFresh()?env.t:NAN,0," C");
  datum(d,75,"RH",envFresh()?env.rh:NAN,0," %");
  datum(d,102,"P",envFresh()?env.p/3386.389f:NAN,3," inHg");
  datum(d,129,"Rgas",fresh?gas.raw:NAN,2," k");
  datum(d,156,"AGE",env.have?(nowMs()-env.ms)/1000.0f:NAN,1," s");
  row(d,183,"GAS FLAGS",env.flagsKnown?(env.gasAccepted?"VALID":"REJECT"):"UNKNOWN",fresh?GREEN:AMBER);
  if(fresh)smallNote(d,216,"k = kOhm / RAW GAS");
  break;
 case 1:
  text(d,12,46,fresh&&gas.seeded?"INITIALIZE FROM R":"a = 1-exp(-dt/tau)",&SmallFont,CYAN);
  text(d,12,68,fresh&&gas.seeded?"F = R":"F = F0 + a*(R-F0)",&LabelFont);
  datum(d,94,"dt",fresh?gas.dt:NAN,3," s");
  datum(d,114,"tau",FAST_TAU_S,1," s");
  datum(d,134,"a",fresh?gas.alpha:NAN,5,"");
  datum(d,154,"R",fresh?gas.raw:NAN,3," k");
  datum(d,174,"F0",fresh?gas.fPrev:NAN,3," k");
  datum(d,194,"F",fresh?gas.f:NAN,3," k");
  if(fresh)smallNote(d,218,gas.seeded?"SEED: F = R":"LIVE OPERANDS / kOhm");
  break;
 case 2:
  text(d,12,46,fresh&&gas.seeded?"INITIALIZE FROM R":"b = 1-exp(-dt/tau)",&SmallFont,AMBER);
  text(d,12,68,fresh&&gas.seeded?"B = R":"B = B0 + b*(F-B0)",&LabelFont);
  datum(d,94,"dt",fresh?gas.dt:NAN,3," s");
  datum(d,114,"tau",BASE_TAU_S,0," s",AMBER);
  datum(d,134,"b",fresh?gas.beta:NAN,5,"",AMBER);
  datum(d,154,"F",fresh?gas.f:NAN,3," k");
  datum(d,174,"B0",fresh?gas.basePrev:NAN,3," k",AMBER);
  datum(d,194,"B",fresh?gas.base:NAN,3," k",AMBER);
  if(fresh)smallNote(d,218,gas.seeded?"SEED: B = R":"LOCAL REF / NOT CLEAN AIR",AMBER);
  break;
 case 3:
  center(d,120,47,"q = F / B",&LabelFont);
  value(d,120,76,fresh?gas.ratio:NAN,4,&NumberFont,CYAN);
  center(d,120,116,"d = 100*(q-1)",&LabelFont);
  value(d,120,146,fresh?gas.deviation:NAN,2,&NumberFont,WHITE);
  center(d,120,181,"% FROM LOCAL BASE",&SmallFont,DIM);
  if(fresh)smallNote(d,217,"RAW RATIO / SEPARATE FROM IAQ",AMBER);
  break;
 case 4: {
  Moisture m=envFresh()?moisture(env.t,env.rh):Moisture();
  text(d,12,46,"g = ln(RH/100)",&LabelFont);
  text(d,12,69,"    + a*T/(b+T)",&SmallFont,CYAN);
  text(d,12,95,"Td = b*g/(a-g)",&LabelFont);
  fmt(d,12,119,&SmallFont,DIM,"a=%.2f  b=%.2f",MAG_A,MAG_B);
  datum(d,143,"g",m.gamma,5,"");
  datum(d,169,"Td",m.dew,0," C");
  datum(d,192,"Td",isfinite(m.dew)?fahrenheit(m.dew):NAN,0," F");
  if(envFresh())smallNote(d,218,m.valid?(env.rh==0?"RH=0: DEW UNDEFINED":"MAGNUS / LIQUID WATER"):"MODEL NEEDS T=0..60C",m.valid?DIM:AMBER);
  break;
 }
 case 5: {
  Moisture m=envFresh()?moisture(env.t,env.rh):Moisture();
  text(d,12,46,"es = 6.112*exp(z)",&SmallFont,CYAN);
  text(d,12,64,"z = a*T/(b+T)",&SmallFont,DIM);
  text(d,12,85,"e = es*RH/100",&LabelFont);
  text(d,12,110,"AH = K*e/(T+273.15)",&SmallFont);
  fmt(d,12,131,&SmallFont,DIM,"K=100000/461.5=%.3f",100000.0/WATER_VAPOR_R);
  datum(d,151,"es",m.es,2," hPa");
  datum(d,174,"e",m.e,2," hPa");
  datum(d,197,"AH",m.ah,2," g/m3");
  if(envFresh())smallNote(d,220,m.valid?"a,b: DEW PAGE / T IN C":"MODEL NEEDS T=0..60C",m.valid?DIM:AMBER);
  break;
 }
 case 6:
  datum(d,47,"HEAT SET",heaterTargetC,0," C");
  datum(d,73,"DURATION",heaterDurationMs,0," ms");
  row(d,99,"STABLE",envFresh()&&env.flagsKnown?(env.heaterStable?"YES":"NO"):"--");
  row(d,125,"GAS VALID",envFresh()&&env.flagsKnown?(env.gasValid?"YES":"NO"):"--");
  char os[40];snprintf(os,sizeof(os),"%u/%u/%u",osTemp?1u<<clipi(osTemp,1,5):0,osHum?1u<<clipi(osHum,1,5):0,osPres?1u<<clipi(osPres,1,5):0);
  row(d,151,"OS T/H/P",os);
  row(d,177,"BSEC",iaq.initialized?"ON":"FAULT",iaq.initialized?GREEN:AMBER);
  if(fresh)smallNote(d,216,"TARGET != MEASURED TEMP",AMBER);
  break;
 case 7: {
  double mean=0,m2=0;float low=INFINITY,high=-INFINITY;int count=0;
  for(int i=0;i<gasHistory.count;++i) { const GasSample& s=gasHistory.at(i);
   if(nowMs()-s.ms>120000)continue;
   ++count;double delta=s.raw-mean;mean+=delta/count;m2+=delta*(s.raw-mean);
   low=fminf(low,s.raw);high=fmaxf(high,s.raw);
  }
  text(d,12,46,"sd=sqrt(sum((R-u)^2)/N)",&SmallFont);
  datum(d,75,"N / 120s",count,0,"");
  datum(d,101,"u",count?(float)mean:NAN,3," k");
  datum(d,127,"sd",count?sqrtf((float)(m2/count)):NAN,3," k");
  datum(d,153,"MIN",count?low:NAN,2," k");
  datum(d,179,"MAX",count?high:NAN,2," k");
  if(fresh)smallNote(d,216,"POPULATION SD / NOT CONF",AMBER);
  break;
 }
 }
}
void particleFlow(Adafruit_GFX& d,int y,uint16_t n,int kind) {
 if(!n)return;
 int dots=clipi((int)ceilf(15*log1pf(n)/log1pf(12000.0f)),1,15);
 for(int i=0;i<dots;++i) {
  int x=20+(int)((nowMs()/80+i*197/dots)%197);
  if(kind==0)d.drawPixel(x,y,CYAN);
  else if(kind==1)d.fillCircle(x,y,1,CYAN);
  else d.drawCircle(x,y,2,CYAN);
 }
}
void particleDetails(Adafruit_GFX& d){
 bool fresh=pmFresh(),counts=fresh&&pm.countsOK;
 consoleHeader(d,"PMS // COMBINED",0,2,fresh);
 const char* massLabels[3]={"PM1","PM2.5","PM10"};const float mass[3]={(float)pm.pm1,(float)pm.pm25,(float)pm.pm10};
 for(int i=0;i<3;++i){int x=48+i*72;center(d,x,44,massLabels[i],&SmallFont,CYAN);value(d,x,61,fresh?mass[i]:NAN,0,&NumberFont,WHITE,64);}
 smallNote(d,94,"MASS / ug/m3");rule(d,110);
 const char* size[6]={">0.3",">0.5",">1.0",">2.5",">5.0",">10"};
 text(d,12,116,"um",&SmallFont,DIM);right(d,228,116,"COUNTS / 0.1L",&SmallFont,DIM);
 for(int i=0;i<6;++i){int col=i/3,rowIndex=i%3,x=12+col*112,y=134+rowIndex*17;
  text(d,x,y,size[i],&SmallFont,CYAN);char number[12];if(counts)snprintf(number,sizeof(number),"%u",pm.counts[i]);else strcpy(number,"--");
  right(d,x+104,y,number,&SmallFont,WHITE);}
 rule(d,185);
 const char* bandLabels[3]={"0.3-1","1-2.5",">2.5"};
 const float band[3]={counts?(float)(pm.counts[0]-pm.counts[2]):NAN,counts?(float)(pm.counts[2]-pm.counts[3]):NAN,counts?(float)pm.counts[3]:NAN};
 for(int i=0;i<3;++i){int x=48+i*72;center(d,x,191,bandLabels[i],&SmallFont,CYAN);value(d,x,207,band[i],0,&LabelFont,WHITE,65);
  // Keep the three particle-size animations within their respective columns.
  if(counts&&band[i]>0){int dx=x-25+(int)((nowMs()/80+i*17)%51);if(i==0)d.drawPixel(dx,229,CYAN);else if(i==1)d.fillCircle(dx,229,1,CYAN);else d.drawCircle(dx,229,2,CYAN);}}
}

void environmentDetails(Adafruit_GFX& d) {
 if(subpage==0) { gasDetails(d,0,0,4); return; }
 if(subpage==1) { gasDetails(d,4,1,4); return; }
 if(subpage==2) { gasDetails(d,5,2,4); return; }
 consoleHeader(d,"BME // PRESS",3,4,envFresh());
 text(d,12,47,"inHg = Pa/3386.389",&SmallFont);
 datum(d,80,"P",envFresh()?env.p:NAN,1," Pa");
 datum(d,109,"P",envFresh()?env.p/100:NAN,2," hPa");
 datum(d,138,"P",envFresh()?env.p/3386.389f:NAN,3," inHg");
 text(d,12,176,"Station pressure only.",&SmallFont,DIM);
 text(d,12,197,"No sea-level correction.",&SmallFont,DIM);
}
void eyeDetails(Adafruit_GFX& d){
 consoleHeader(d,"CO2 // RAW",0,2,co2Fresh());
 datum(d,48,"CO2",co2Fresh()?co2.ppm:NAN,0," ppm");
 datum(d,78,"AGE",co2.have?(nowMs()-co2.ms)/1000.0f:NAN,1," s");
 datum(d,108,"GOOD",co2.good,0,"");datum(d,138,"ERRORS",co2.bad,0,"");
 text(d,12,180,"SCD41 periodic / CRC-8",&SmallFont,DIM);
}

void environmentSources(Adafruit_GFX& d) {
 const bool sf=si7021Fresh(),hf=environmentHumidityFresh();
 consoleHeader(d,"ENV // SOURCES",7,8,hf);
 datum(d,48,"Si TEMP",environmentTemperatureF(),0," F");
 datum(d,76,"Si RH",sf?si7021.rh:NAN,0," %");
 datum(d,104,"BME RH",envFresh()?env.rh:NAN,0," %");
 datum(d,132,"AVG RH",environmentHumidity(),0," %");
 datum(d,160,"Si AGE",si7021.have?(nowMs()-si7021.ms)/1000.0f:NAN,1," s");
 datum(d,188,"Si ERR",si7021.bad,0,"");
 smallNote(d,217,hf?"RH = (Si + BME) / 2":!sf?"Si7021 / WAIT":"BME680 RH / WAIT",hf?DIM:AMBER);
}



void systemDetails(Adafruit_GFX& d) {
 consoleHeader(d,subpage==0?"SYS // TIMING":subpage==1?"SYS // HEALTH":subpage==2?"SYS // CONFIG":"SYS // LIGHT",subpage,4,true);
 if(subpage==0) {
  datum(d,48,"FPS",perf.fps,1,"");
  datum(d,77,"DRAW AVG",perf.frames?perf.drawUs/1000.0f/perf.frames:NAN,1," ms");
  datum(d,106,"DRAW MAX",perf.maxDrawUs/1000.0f,1," ms");
  datum(d,135,"LATE",perf.overruns,0,"");
  datum(d,164,"SPI",spiHz/1000000.0f,0," MHz");
  smallNote(d,207,"FRAME TARGET 20 FPS");
 } else if(subpage==1) {
  datum(d,48,"HEAP",ESP.getFreeHeap()/1024.0f,1," KiB");
  datum(d,75,"MIN HEAP",ESP.getMinFreeHeap()/1024.0f,1," KiB");
  datum(d,102,"BME FAIL",env.failed,0,"");
  datum(d,129,"GAS SKIP",env.gasRejected,0,"");
  datum(d,156,"PMS BAD",pm.bad,0,"");
  datum(d,183,"COUNT BAD",pm.countBad,0,"");
 } else if(subpage==3) {
  datum(d,48,"LIGHT",lightFresh()&&!light.saturated?light.lux:NAN,1," lx");
  datum(d,77,"RAW",lightFresh()?light.raw:NAN,0,"");
  datum(d,106,"TARGET",light.target*100.0f/255,0," %");
  datum(d,135,"OUTPUT",light.duty*100.0f/255,0," %");
  row(d,164,"PWM",light.pwmReady?"ON":"UNWIRED");
  smallNote(d,207,light.saturated?"LIGHT SATURATED":lightFresh()?"VEML7700 / LIVE":"VEML7700 / WAIT",lightFresh()?DIM:AMBER);
 } else {
  row(d,48,"DISPLAY","240x240"); row(d,77,"MODE","SPI3");
  row(d,106,"I2C","100 kHz");
  datum(d,135,"BUTTON",BUTTON_GPIO,0,"");
  datum(d,164,"PMS RX",PMS_RX_GPIO,0,"");
  smallNote(d,207,"BUTTON -1 = SERIAL ONLY");
 }
}
void iaqDetails(Adafruit_GFX& d){
 consoleHeader(d,"BOSCH // IAQ",0,1,iaqFresh());
 datum(d,47,"IAQ",iaqReady()?iaq.index:NAN,1,"");
 datum(d,73,"ACCURACY",iaq.accuracy,0," /3");
 row(d,99,"RUN-IN",iaq.runIn?"READY":"WAIT",iaq.runIn?GREEN:AMBER);
 row(d,125,"STABLE",iaq.stabilized?"YES":"WAIT",iaq.stabilized?GREEN:AMBER);
 datum(d,151,"bVOC eq",iaqReady()?iaq.vocEquivalent:NAN,2," ppm");
 datum(d,177,"BSEC CODE",iaq.bsecStatus,0,"");
 smallNote(d,216,iaq.restored?"CALIBRATION STATE RESTORED":"CALIBRATION STATE LEARNING");
}
void gasProcessing(Adafruit_GFX& d){
 bool fresh=gasFresh();consoleHeader(d,"GAS // PROCESS",2,7,fresh);
 datum(d,47,"RAW R",fresh?gas.raw:NAN,2," k");
 datum(d,75,"FILTER F",fresh?gas.f:NAN,2," k");
 datum(d,103,"BASE B",fresh?gas.base:NAN,2," k");
 datum(d,131,"F / B",fresh?gas.ratio:NAN,4,"");
 datum(d,159,"DELTA",fresh?gas.deviation:NAN,2," %");
 text(d,12,187,"tau F=6s / B=180s",&SmallFont,CYAN);
 smallNote(d,209,"LOCAL BASE / RAW GAS");
 // Full coefficients and previous operands remain available in the /status JSON.
}

void detailsScreen(Adafruit_GFX& d){
 const uint8_t selected=subpage;infoOrdinal=selected+1;infoTotal=detailCount();
 if(serviceMode){
  if(selected==0)serviceScreen(d);else {subpage=selected-1;systemDetails(d);}
 }else switch(page){
  case EYE:if(selected==0)eyeDetails(d);else {trendMetric=2;trendScreen(d);}break;
  case AIR_SAMPLE:if(selected==0)particleDetails(d);else {trendMetric=1;trendScreen(d);}break;
  case CHEMISTRY:switch(selected){
   case 0:gasMechanism(d);break;
   case 1:gasRibbon(d);break;
   case 2:gasProcessing(d);break;
   case 3:gasDetails(d,6);break;
   case 4:gasDetails(d,7);break;
   case 5:iaqDetails(d);break;
   case 6:trendMetric=6;trendScreen(d);break;
  }break;
  case ENVIRONMENT:if(selected<4)environmentDetails(d);else if(selected==7)environmentSources(d);else {trendMetric=selected-1;trendScreen(d);}break;
  default:break;
 }
 subpage=selected;
}


// -------------------------------- Boot ------------------------------------
void bootScreen(Adafruit_GFX& d) {
 uint64_t elapsed=nowMs()-bootAt;
 center(d,120,34,"BREATHE",&NumberFont,WHITE);
 smallNote(d,73,"ENVIRONMENTAL INSTRUMENT",CYAN);
 rule(d,101,DIM);
 if(elapsed>300)row(d,116,"FRAMEBUF",canvas?"READY":"FAULT",canvas?GREEN:PINK);
 if(elapsed>650)row(d,143,"BME680",envFresh()?"LIVE":env.present?"FOUND":"WAIT",envFresh()?GREEN:AMBER);
 if(elapsed>1000)row(d,170,"PMS5003",pmFresh()?"LIVE":"WAIT",pmFresh()?GREEN:AMBER);
 if(elapsed>1350)row(d,197,"CO2",co2Fresh()?"LIVE":ENABLE_SCD41?"WAIT":"OFF",co2Fresh()?GREEN:AMBER);
 // A completion indicator for the animation, not a fictional sensor self-test.
 int w=(int)clipf(elapsed/2800.0f*216,0,216);d.drawFastHLine(12,225,w,CYAN);
}
void drawFrame() {
 if(!canvas)return;
 Adafruit_GFX& d=*canvas; furniture(d);
 uint64_t n=nowMs(), bootElapsed=n-bootAt;
 if(bootActive && bootElapsed<2800) bootScreen(d);
 else {
  if(bootActive && bootElapsed>=3300) { bootActive=false; pageSince=n; }
  if(detail)detailsScreen(d);
  else switch(page) {
   case EYE:co2Main(d);break;case AIR_SAMPLE:particleMain(d);break;
   case CHEMISTRY:gasMain(d);break;case ENVIRONMENT:environmentMain(d);break;default:break;
  }
  if(bootActive) {
   // Mechanical shutter opens onto the real screen; sensor acquisition continues.
   int radius=(int)(clipf((bootElapsed-2800)/500.0f,0,1)*171);
   for(int y=0;y<240;++y) {
    int dy=y-120, dx=radius*radius-dy*dy;
    if(dx<=0)d.drawFastHLine(0,y,240,BLACK);
    else { int reach=(int)sqrtf((float)dx);int left=clipi(120-reach,0,240);
     int rightEdge=clipi(120+reach,0,240);d.drawFastHLine(0,y,left,BLACK);
     d.drawFastHLine(rightEdge,y,240-rightEdge,BLACK); }
   }
  }
 }
 tft.drawRGBBitmap(0,0,canvas->getBuffer(),240,240);
}

// -------------------------- Serial + local status -------------------------
bool wifiStarted=false, wasConnected=false;
uint32_t reconnectAt=0, logAt=0, wifiDropouts=0, httpRequests=0;
String jsonNumber(double v,int decimals=3) { return isfinite(v)?String(v,decimals):String("null"); }
String jsonBool(bool v) { return v?String("true"):String("false"); }
String statusJSON() {
 const bool ef=envFresh(), gf=gasFresh(), pf=pmFresh(), cf=co2Fresh(), sf=si7021Fresh();
 Moisture m=ef?moisture(env.t,env.rh):Moisture();
 String s; s.reserve(3300);
 s="{\"firmware\":\"BREATHE 4.4.6\",\"page\":\"";s+=PAGE_NAMES[page];
 s+="\",\"details\":"+jsonBool(detail)+",\"subpage\":"+String(subpage+1);
 char id[24];screenID(id,sizeof(id));s+=",\"screen_id\":\""+String(id)+"\"";
 s+=",\"auto_tour\":"+jsonBool(autoTour)+",\"uptime_s\":"+jsonNumber(nowMs()/1000.0,1);
 s+=",\"bme\":{\"found\":"+jsonBool(env.present)+",\"fresh\":"+jsonBool(ef);
 s+=",\"age_s\":"+jsonNumber(env.have?(nowMs()-env.ms)/1000.0:NAN,1);
 s+=",\"temperature_C\":"+jsonNumber(ef?env.t:NAN);
 s+=",\"humidity_pct\":"+jsonNumber(ef?env.rh:NAN);
 s+=",\"station_pressure_Pa\":"+jsonNumber(ef?env.p:NAN,1);
 s+=",\"dewpoint_C_water_model\":"+jsonNumber(m.dew);
 s+=",\"absolute_humidity_g_m3\":"+jsonNumber(m.ah);
 s+=",\"heater_target_C\":"+String(heaterTargetC);
 s+=",\"heater_duration_ms\":"+String(heaterDurationMs);
 s+=",\"heater_stable\":"+String(ef&&env.flagsKnown?(env.heaterStable?"true":"false"):"null");
 s+=",\"gas_valid_flag\":"+String(ef&&env.flagsKnown?(env.gasValid?"true":"false"):"null");
 s+=",\"good\":"+String(env.good)+",\"failed\":"+String(env.failed)+"}";
 s+=",\"gas_model\":{\"fresh\":"+jsonBool(gf);
 s+=",\"raw_kOhm\":"+jsonNumber(gf?gas.raw:NAN);
 s+=",\"filtered_kOhm\":"+jsonNumber(gf?gas.f:NAN);
 s+=",\"baseline_kOhm\":"+jsonNumber(gf?gas.base:NAN);
 s+=",\"previous_raw_kOhm\":"+jsonNumber(gf?gas.rawPrev:NAN);
 s+=",\"previous_filtered_kOhm\":"+jsonNumber(gf?gas.fPrev:NAN);
 s+=",\"previous_baseline_kOhm\":"+jsonNumber(gf?gas.basePrev:NAN);
 s+=",\"dt_s\":"+jsonNumber(gf?gas.dt:NAN);
 s+=",\"alpha\":"+jsonNumber(gf?gas.alpha:NAN,6);
 s+=",\"beta\":"+jsonNumber(gf?gas.beta:NAN,6);
 s+=",\"ratio_F_B\":"+jsonNumber(gf?gas.ratio:NAN,6);
 s+=",\"deviation_pct\":"+jsonNumber(gf?gas.deviation:NAN);
 s+=",\"sample_step_pct\":"+jsonNumber(gf?gas.step:NAN);
 s+=",\"local_reference_samples\":"+String(gas.samples);
 s+=",\"event_count\":"+String(gas.events);
 s+=",\"event_step_pct\":"+jsonNumber(gas.eventStep);
 s+=",\"iaq\":"+jsonNumber(iaqReady()?iaq.index:NAN,1);
 s+=",\"iaq_accuracy\":"+String(iaq.accuracy);
 s+=",\"iaq_stabilized\":"+jsonBool(iaq.stabilized)+",\"iaq_run_in\":"+jsonBool(iaq.runIn);
 s+=",\"voc_equivalent_ppm\":"+jsonNumber(iaqReady()?iaq.vocEquivalent:NAN,3);
 s+=",\"BSEC\":\"enabled\",\"bsec_status\":"+String(iaq.bsecStatus)+"}";
 s+=",\"pms\":{\"fresh\":"+jsonBool(pf)+",\"counts_valid\":"+jsonBool(pf&&pm.countsOK);
 s+=",\"age_s\":"+jsonNumber(pm.have?(nowMs()-pm.ms)/1000.0:NAN,1);
 s+=",\"pm1_atmospheric_ug_m3\":"+jsonNumber(pf?pm.pm1:NAN,0);
 s+=",\"pm25_atmospheric_ug_m3\":"+jsonNumber(pf?pm.pm25:NAN,0);
 s+=",\"pm10_atmospheric_ug_m3\":"+jsonNumber(pf?pm.pm10:NAN,0);
 s+=",\"cumulative_counts_per_0_1L\":[";
 for(int i=0;i<6;++i) { if(i)s+=",";s+=jsonNumber(pf?pm.counts[i]:NAN,0); }
 s+="],\"thresholds_um\":[0.3,0.5,1,2.5,5,10],\"bad_frames\":"+String(pm.bad);
 s+=",\"good_frames\":"+String(pm.good);
 char byteCount[24]; snprintf(byteCount,sizeof(byteCount),"%llu",(unsigned long long)pmsDiag.bytes);
 s+=",\"rx_bytes\":"+String(byteCount);
 s+=",\"last_byte_age_s\":"+jsonNumber(pmsDiag.bytes?(nowMs()-pmsDiag.lastByteMs)/1000.0:NAN,1);
 s+=",\"recent_hex\":\""+pmsDiag.hex()+"\"";
 s+=",\"rx_gpio\":"+String(PMS_RX_GPIO)+",\"tx_gpio\":"+String(PMS_TX_GPIO);
 s+=",\"baud\":9600,\"sensor_error\":"+String(pm.error)+"}";
 s+=",\"co2\":{\"enabled\":"+jsonBool(ENABLE_SCD41)+",\"fresh\":"+jsonBool(cf);
 s+=",\"ppm\":"+jsonNumber(cf?co2.ppm:NAN,0)+"}";
 s+=",\"si7021\":{\"enabled\":"+jsonBool(ENABLE_SI7021)+",\"found\":"+jsonBool(si7021.present);
 s+=",\"configured\":"+jsonBool(si7021.configured)+",\"fresh\":"+jsonBool(sf);
 s+=",\"temperature_C\":"+jsonNumber(sf?si7021.t:NAN)+",\"humidity_pct\":"+jsonNumber(sf?si7021.rh:NAN);
 s+=",\"age_s\":"+jsonNumber(si7021.have?(nowMs()-si7021.ms)/1000.0:NAN,1);
 s+=",\"heater_enabled\":"+String(si7021.configured?((si7021.userRegister&4)?"true":"false"):"null");
 s+=",\"good\":"+String(si7021.good)+",\"failed\":"+String(si7021.bad)+"}";
 s+=",\"environment\":{\"temperature_source\":\"Si7021\",\"humidity_sources\":[\"Si7021\",\"BME680\"]";
 s+=",\"temperature_F\":"+jsonNumber(environmentTemperatureF());
 s+=",\"humidity_average_pct\":"+jsonNumber(environmentHumidity());
 s+=",\"temperature_fresh\":"+jsonBool(sf)+",\"humidity_fresh\":"+jsonBool(environmentHumidityFresh());
 s+=",\"pressure_inHg\":"+jsonNumber(ef?env.p/3386.389f:NAN)+"}";
 s+=",\"light\":{\"found\":"+jsonBool(light.present)+",\"fresh\":"+jsonBool(lightFresh());
 s+=",\"lux_estimate\":"+jsonNumber(lightFresh()&&!light.saturated?light.lux:NAN,1);
 s+=",\"saturated\":"+jsonBool(light.saturated)+",\"raw\":"+jsonNumber(lightFresh()?light.raw:NAN,0);
 s+=",\"target_pct\":"+jsonNumber(light.target*100.0/255,1);
 s+=",\"pwm_enabled\":"+jsonBool(light.pwmReady)+"}";
 s+=",\"display\":{\"fps\":"+jsonNumber(perf.fps,2);
 s+=",\"mean_draw_ms\":"+jsonNumber(perf.frames?perf.drawUs/1000.0/perf.frames:NAN);
 s+=",\"max_draw_ms\":"+jsonNumber(perf.maxDrawUs/1000.0);
 s+=",\"frame_overruns\":"+String(perf.overruns)+",\"spi_hz\":"+String(spiHz)+"}";
 s+=",\"wifi_connected\":"+jsonBool(WiFi.status()==WL_CONNECTED);
 s+=",\"wifi_dropouts\":"+String(wifiDropouts)+",\"http_requests\":"+String(httpRequests);
 s+=",\"heap_free\":"+String(ESP.getFreeHeap())+",\"heap_min\":"+String(ESP.getMinFreeHeap())+"}";
 return s;
}
void printStatus() {
 Serial.print("STATUS ");Serial.println(statusJSON());
 if(WiFi.status()==WL_CONNECTED)Serial.printf("WEB: http://%s/\n",WiFi.localIP().toString().c_str());
}
void command(char c) {
 switch(c) {
 case 'n':case 'N':navigate(1);break;
 case 'p':case 'P':navigate(-1);break;
 case 'l':case 'L':toggleDetails();break;
 case 'h':case 'H':home();break;
 case 'g':case 'G':home();page=CHEMISTRY;break;
 case 'e':case 'E':home();page=ENVIRONMENT;break;
 case 's':case 'S':case 'r':case 'R':printStatus();break;
 case 'a':case 'A':case ' ':autoTour=!autoTour;serviceMode=false;detail=false;subpage=0;pageSince=nowMs();break;
 case 'b':case 'B':home();bootActive=true;bootAt=nowMs();break;
 case '1':spiHz=10000000;tft.setSPISpeed(spiHz);Serial.println("SPI 10MHz");break;
 case '4':spiHz=40000000;tft.setSPISpeed(spiHz);Serial.println("SPI 40MHz");break;
 default:break;
 }
}
const char WEB_PAGE[] PROGMEM=R"HTML(<!doctype html><html><meta name="viewport" content="width=device-width,initial-scale=1"><title>BREATHE</title>
<style>body{background:#080e16;color:#e5f2f3;font:16px monospace;margin:24px;max-width:850px}button{background:#122733;color:#b2eef4;border:1px solid #467284;padding:12px;margin:3px;font-size:16px}pre{white-space:pre-wrap;font-size:14px}small{color:#9eb7c5}</style>
<h1>BREATHE // 4.4.6</h1><p>Local instrument controls</p>
<button onclick="key('p')">Previous</button><button onclick="key('n')">Next</button><button onclick="key('l')">Details / Return</button><button onclick="key('h')">Home</button><button onclick="key('g')">Bosch</button>
<p><small>Gas AQI displays the Bosch IAQ index after run-in and calibration; it is not EPA AQI. Raw gas and bVOC equivalent remain distinct. Missing/stale current values are null. Times are uptime, not wall-clock time.</small></p>
<pre id="status">Connecting...</pre><script>
async function key(c){await fetch('/control?key='+c,{method:'POST'});poll(false)}
async function poll(again=true){try{const r=await fetch('/status',{cache:'no-store'});document.getElementById('status').textContent=JSON.stringify(await r.json(),null,2)}catch(e){document.getElementById('status').textContent='Connection unavailable'}if(again)setTimeout(poll,3000)}poll();
</script></html>)HTML";
void startWiFi() {
 if(!WIFI_PASSWORD[0] || !strcmp(WIFI_PASSWORD,"PUT_YOUR_WIFI_PASSWORD_HERE")) {
  WiFi.mode(WIFI_OFF);Serial.println("WiFi disabled: enter password locally. Sensors/UI still work.");return;
 }
 WiFi.mode(WIFI_STA);WiFi.setHostname("breathe");WiFi.setSleep(false);
 WiFi.begin(WIFI_SSID,WIFI_PASSWORD);wifiStarted=true;reconnectAt=millis()+30000;
 server.on("/",[](){++httpRequests;server.send(200,"text/html",WEB_PAGE);});
 server.on("/status",[](){++httpRequests;server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",statusJSON());});
 server.on("/control",HTTP_POST,[](){
  ++httpRequests;String k=server.arg("key");
  if(k.length()!=1 || !strchr("nplhg",k[0])) {server.send(400,"text/plain","Unknown control");return;}
  command(k[0]);server.send(200,"text/plain","OK");
 });
 server.begin();Serial.println("WiFi connecting; no wait is imposed on the display.");
}
void serviceNetwork() {
 if(!wifiStarted)return;
 bool connected=WiFi.status()==WL_CONNECTED;
 if(connected && !wasConnected)Serial.printf("WEB: http://%s/\n",WiFi.localIP().toString().c_str());
 if(!connected && wasConnected)++wifiDropouts;
 wasConnected=connected;
 if(!connected && due32(millis(),reconnectAt)) {WiFi.reconnect();reconnectAt=millis()+30000;}
 server.handleClient();
}

// ------------------------------ Arduino -----------------------------------
void setup() {
 Serial.begin(115200); // No wait for a USB serial monitor to be opened.
 Serial.println("\nBREATHE INSTRUMENT OS 4.4.6");
 WiFi.mode(WIFI_OFF);
 SPI.begin(8,-1,10,-1);tft.init(240,240,SPI_MODE3);tft.setRotation(0);tft.setSPISpeed(10000000);
 tft.fillScreen(BLACK);center(tft,120,92,"BREATHE",&NumberFont,WHITE);
 canvas=new(std::nothrow) GFXcanvas16(240,240);
 if(canvas && !canvas->getBuffer()) {delete canvas;canvas=nullptr;}
 if(!canvas) {
  tft.fillScreen(BLACK);center(tft,120,78,"FRAMEBUFFER",&LabelFont,PINK);
  center(tft,120,108,"UNAVAILABLE",&LabelFont,PINK);
  Serial.println("FRAMEBUFFER_UNAVAILABLE: sensors and serial remain active.");
 }
 spiHz=FINAL_SPI_HZ;tft.setSPISpeed(spiHz);
 Wire.begin(6,7);Wire.setClock(100000);Wire.setTimeOut(50);
 beginBME();
 beginBacklight(); serviceLight();
 if(ENABLE_PMS) {
  pmsPort.setRxBufferSize(256);
  pinMode(PMS_RX_GPIO,INPUT_PULLUP); // BEFORE UART claims RX (ESP32 core 3.x safe)
  pmsStartedMs=nowMs();
  pmsPort.begin(9600,SERIAL_8N1,PMS_RX_GPIO,PMS_TX_GPIO);
  Serial.println("PMS5003 UART D7 RX / D6 TX; stabilizing for 30 seconds.");
 }
 if(BUTTON_GPIO>=0)pinMode(BUTTON_GPIO,INPUT_PULLUP);
 rngState=esp_random();if(!rngState)rngState=0x63C3B680;
 bootAt=pageSince=perf.startMs=perf.fpsAt=nowMs();
 startWiFi();logAt=millis()+10000;
 Serial.println("n/p next/previous | l details/return | h home | g Bosch | e environment");
 Serial.println("a/SPACE auto-tour | s status | b boot | 1/4 10/40MHz SPI");
 if(BUTTON_GPIO<0)Serial.println("Physical button disabled: confirmed working configuration was GPIO -1.");
 Serial.println("No demo sensor readings. Bosch BSEC2 IAQ enabled; readiness gated. SCD41 on I2C 0x62.");
 Serial.println("Environment: Si7021 temperature (0x40), RH average of Si7021 + BME680, BME680 pressure.");
}
void loop() {
 static uint64_t nextFrame=0,previousLoop=0;
 uint64_t now=(uint64_t)esp_timer_get_time();
 if(previousLoop) { uint32_t gap=(uint32_t)(now-previousLoop);if(gap>perf.maxGapUs)perf.maxGapUs=gap; }
 previousLoop=now;
 serviceButton();
 for(int i=0;i<32 && Serial.available();++i)command((char)Serial.read());
 servicePMS();serviceCO2();serviceBME();serviceSi7021();recordEnvironmentHumidity();serviceLight();serviceNetwork();
 if(autoTour && !detail && !bootActive && nowMs()-pageSince>=TOUR_MS) {
  page=(Page)((page+1)%PAGE_COUNT);pageSince=transitionAt=nowMs();
 }
 if(due32(millis(),logAt)) {logAt=millis()+10000;printStatus();}
 now=(uint64_t)esp_timer_get_time();
 if(canvas && now>=nextFrame) {
  nextFrame=now+FRAME_US;
  uint64_t start=(uint64_t)esp_timer_get_time();drawFrame();
  uint32_t duration=(uint32_t)((uint64_t)esp_timer_get_time()-start);
  ++perf.frames;++perf.windowFrames;perf.drawUs+=duration;
  if(duration>perf.maxDrawUs)perf.maxDrawUs=duration;
  if(duration>FRAME_US)++perf.overruns;
  // Drain UART after the relatively long full-frame SPI transaction.
  servicePMS();serviceButton();
 }
 if(nowMs()-perf.fpsAt>=2000) {
  perf.fps=perf.windowFrames*1000.0f/(nowMs()-perf.fpsAt);
  perf.windowFrames=0;perf.fpsAt=nowMs();
 }
 delay(1); // yield to USB, Wi-Fi, RTOS, and watchdog
}
// -------------------------- END OF COMPLETE FILE --------------------------
