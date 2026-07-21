// 0x02 = USB Vendor ID Source
// 0x12BA = Harmonix / Sony Rhythm Device
// 0x0100 = Guitar Controller ID
// 0x0111 = Version
// hid->pnp(0x02, 0x12BA, 0x0100, 0x0111);
// hid->hidInfo(0x00, 0x01);

// A guitar maps its physical components into standard gamepad fields.Frets: Green, Red, Yellow, Blue, Orange are mapped to standard digital buttons.Strum Bar: Mapped to the D-Pad (Hat Switch). Strumming Up triggers D-pad Up; strumming Down triggers D-pad Down.Whammy Bar: Mapped to a single analog axis (typically the Left X-axis).Tilt (Star Power): Mapped to another analog axis (typically the Left Y-axis) or a specific button

/*
  ESP32 BLE Gamepad GUITAR Rockband Stratocaster (wii edition)
    
    5 frets + solo frets (5 gpio)
    2 strum
    4 dpad
    2 start+select
    1 home
    1 reset/pair
    1 whammy ADC

    to-do
    accelerometer for tilt/star power
    pickup encoder
    RGB strum bar
    Status indicator lights for power/battery/charging
      Theres a small PCB for 4 LED's (originally player indicator?), we can re-use that for battery level indicator
      Need to add a WS2812 LED under the home button for power/charging status indicator
    reset BLE button
  
  Library required:
    "ESP32-BLE-Gamepad" by lemmingDev
    "Battery_18650_Stats" by Danilo

  BOARD SPECS:
    Lilygo T-Energy S3
    LG 3000mah 18650

  Board settings (changed from defaults):
    USB CDC           = enabled
    CPU freqency      = 160Mhz or 80Mhz
    Flash size        = 16MB (128Mb)
    Partition scheme  = 16M Flash (3M/9.9M)
    PSRAM             = disabled

  WIRING:
  (each: 1 leg -> GPIO, opposite leg -> GND)

                                         +-----------------------+
                                         | O      | BAT |      O |
                                         |        |GPIO3|        |
                                     3V3 | [ ]               [ ] | GND    / GND
                                   RESET | [ ]               [ ] | GPIO1  / RESET BUTTON
               -----------------/ GPIO37 | [ ]               [ ] | GPIO2  / HOME BUTTON / WAKE FROM SLEEP
                          START / GPIO35 | [ ]            TX [ ] | GPIO43 /-----------------
           SELECT / MENU / BACK / GPIO36 | [ ]            RX [ ] | GPIO44 /
                             UP / GPIO45 | [ ]               [ ] | GPIO42 /-----------------
                           DOWN / GPIO48 | [ ]               [ ] | GND    / GND
               -----------------/ GPIO47 | [ ]               [ ] | GPIO4
                          RIGHT / GPIO21 | [ ]               [ ] | GPIO5
                             UP / GPIO14 | [ ]               [ ] | GPIO6
                           DOWN / GPIO13 | [ ]               [ ] | GPIO7
                           LEFT / GPIO12 | [ ]               [ ] | GPIO17 /-----------------
               -----------------/ GPIO11 | [ ]   HX6610S     [ ] | GPIO18
                            GND /    GND | [ ]    |          [ ] | GPIO0  /-----------------/ strapping pin / pull-up only
                                / GPIO10 | [x]----+          [ ] | GPIO8  / ORANGE FRET
                     WHAMMY ADC /  GPIO9 | [ ]               [ ] | GPIO41 / BLUE FRET
 pull-down only / strap ------- / GPIO46 | [ ]               [ ] | GPIO40 / YELLOW FRET
                                      NC | [ ]               [ ] | GPIO39 / RED FRET
                                      5V | [ ]               [ ] | GPIO38 / GREEN FRET
                                      NC | [ ]               [ ] | NC
                                         |                       |
                                         | [USB]  [QWIIC]        |
                                         | [USB]                 |
                                         | O                   O |
                                         +-----------------------+

  NOTE:
    GPIO10 is bridged to pin 6 of the HX6610S with a small resistor for charging detection. When battery is charging, the red LED is lit up and pin 6 on the HX6610S is pulled low.
    Make sure to set GPIO10 to internal pullup.

    Additionally, you can do the same thing to pin 7 on the HX6610S for standby indication. (powered on but not charging)

    GPIO43 / GPIO44 are the RX/TX pins on the QWIIC connector for an i2c tilt sensor


  All digital pins use internal pull-ups — no external resistors needed.

  NOTE:
  ======
  Only RTC IO can be used as a source for external wake
  source. They are pins: 0,2,4,12-15,25-27,32-39.

  ADC pin is for a Lilygo T-Energy S3 battery voltage divider
*/

#include <Arduino.h>
#include <BleGamepad.h>
#include <Battery18650Stats.h>
#include "driver/rtc_io.h"
#include "InterruptButton.h"

#include <NimBLEDevice.h>

// DEBUG
//#define DEBUG

// 18650 Battery Stuffs
#define ADC_PIN 3                           // Undefine to disable battery detection/monitoring
#define ADC_DIVIDER 2500
#define CHARGING_PIN 10                     
#define BATTERY_REPORT_INTERVAL (5 * 1000)  // 5 Seconds
#define BATTERY_TIMEOUT (10 * (60 * 1000))  // 10 Minutes
#define WAKEUP_PIN GPIO_NUM_2               // Wakeup GPIO ext0, currently same pin as the HOME button
RTC_DATA_ATTR bool wakeup = false;          // Wakeup tracker
Battery18650Stats battery(ADC_PIN);
uint8_t batteryLevel = 0;
bool    batteryCharging = false;
bool    batteryState = false;
bool    keyPressed = false;
unsigned long lastBatteryCheck = 0;
unsigned long lastTimeoutCheck = 0;

// ===================== Configuración =====================
constexpr uint8_t NUM_BUTTONS_TOTAL = 16;   // límite del HID 
constexpr uint8_t NUM_HATS          = 1;

// Pines
constexpr uint8_t PIN_FRET_GREEN  = 38;
constexpr uint8_t PIN_FRET_RED    = 39;
constexpr uint8_t PIN_FRET_YELLOW = 40;
constexpr uint8_t PIN_FRET_BLUE   = 41;
constexpr uint8_t PIN_FRET_ORANGE = 8;

constexpr uint8_t PIN_STRUM_UP    = 45;
constexpr uint8_t PIN_STRUM_DOWN  = 48;

constexpr uint8_t PIN_START       = 35;
constexpr uint8_t PIN_MENU        = 36;

constexpr uint8_t PIN_HOME        = WAKEUP_PIN;
constexpr uint8_t PIN_RESET       = 1;

constexpr uint8_t PIN_UP          = 14;
constexpr uint8_t PIN_DOWN        = 13;
constexpr uint8_t PIN_LEFT        = 12;
constexpr uint8_t PIN_RIGHT       = 21;

//constexpr uint8_t PIN_TILT        = 10; // todo: Upgrade to ADXL345 i2c/QWIIC
constexpr uint8_t PIN_WHAMMY_ADC  = 9; // ADC2_2

// Interrupt pins
InterruptButton fretGreen   (PIN_FRET_GREEN, LOW, GPIO_MODE_INPUT, 750, 250, 333, 2000);
InterruptButton fretRed     (PIN_FRET_RED, LOW, GPIO_MODE_INPUT, 750, 250, 333, 2000);
InterruptButton fretYellow  (PIN_FRET_YELLOW, LOW, GPIO_MODE_INPUT, 750, 250, 333, 2000);
InterruptButton fretBlue    (PIN_FRET_BLUE, LOW, GPIO_MODE_INPUT, 750, 250, 333, 2000);
InterruptButton fretOrange  (PIN_FRET_ORANGE, LOW, GPIO_MODE_INPUT, 750, 250, 333, 2000);

InterruptButton strumUp     (PIN_STRUM_UP, LOW, GPIO_MODE_INPUT, 750, 250, 333, 5000);
InterruptButton strumDown   (PIN_STRUM_DOWN, LOW, GPIO_MODE_INPUT, 750, 250, 333, 5000);

InterruptButton start       (PIN_START, LOW, GPIO_MODE_INPUT, 750, 250, 333, 8000);
InterruptButton menu        (PIN_MENU, LOW, GPIO_MODE_INPUT, 750, 250, 333, 8000);
InterruptButton home        (PIN_HOME, LOW, GPIO_MODE_INPUT, 750, 250, 333, 8000);
InterruptButton reset       (PIN_RESET, LOW, GPIO_MODE_INPUT, 750, 250, 333, 8000);

InterruptButton up          (PIN_UP, LOW, GPIO_MODE_INPUT, 750, 250, 333, 8000);
InterruptButton down        (PIN_DOWN, LOW, GPIO_MODE_INPUT, 750, 250, 333, 8000);
InterruptButton left        (PIN_LEFT, LOW, GPIO_MODE_INPUT, 750, 250, 333, 8000);
InterruptButton right       (PIN_RIGHT, LOW, GPIO_MODE_INPUT, 750, 250, 333, 8000);

// Debounce
constexpr uint8_t  WHAMMY_INTERVAL_MS = 6;  // 4–8 ms

// Whammy (ADC) 
constexpr int RAW_MIN = 100;
constexpr int RAW_MAX = 3900;
constexpr int16_t X_MIN = 0;        // 0..32767 (trigger/slider o RZ)
constexpr int16_t X_MAX = 32767;

constexpr float   EMA_ALPHA      = 0.25f;
constexpr int16_t HID_DEADBAND_X = 400;   // histéresis en HID

// ---- NUEVOS parámetros de robustez ----
constexpr int      RAW_NOISE_FLOOR     = 25;   // ruido típico en cuentas ADC
constexpr int      GATE_ENTER_DELTA    = 140;  // distancia a la base para "enganchar"
constexpr int      GATE_EXIT_DELTA     = 80;   // menor que ENTER p/ histéresis
constexpr float    BASELINE_ALPHA_IDLE = 0.02f;// lo rápido que re-centra en reposo
uint32_t lastWhammyMs = 0;
int      rawEma       = -1;
int16_t  lastSentX    = -32768;

// Estado del "gate" (enganche del whammy)
bool     whammyEngaged = false;
int      baselineRaw    = -1;  // línea base en cuentas ADC

// ===================== Mapeo de botones =====================
enum Button {
  BTN_FRET_GREEN = 1,  // IDs HID = 1..
  BTN_FRET_RED,
  BTN_FRET_YELLOW,
  BTN_FRET_BLUE,
  BTN_FRET_ORANGE,
  BTN_STRUM_UP,
  BTN_STRUM_DOWN,
  BTN_START,
  BTN_MENU,
  BTN_UP,
  BTN_DOWN,
  BTN_LEFT,
  BTN_RIGHT,
  BTN_HOME,
  BTN_RESET = 255
};

// ===================== Estado =====================
BleGamepad bleGamepad("GHero-MKII-2", "VFR8221512", 100);

// ===================== Utils =====================
static inline int constrainInt(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline int16_t mapRawToHidX(int raw) {
  raw = constrainInt(raw, RAW_MIN, RAW_MAX);
  float t = (float)(raw - RAW_MIN) / (float)(RAW_MAX - RAW_MIN);
  int32_t x = (int32_t)(X_MIN + t * (X_MAX - X_MIN)); // 0..32767
  return (int16_t)constrainInt(x, X_MIN, X_MAX);
}

// ===================== Prototipos =====================
bool handleWhammy();
bool readBattery();
bool readCharging();
void batteryReport();
bool handleBattery();

void buttonDown(uint8_t i) {
  bleGamepad.press(i);
  keyPressed = true;
}

void buttonUp(uint8_t i) {
  bleGamepad.release(i);
  keyPressed = true;
}

// ===================== Timers =====================
void IRAM_ATTR bleReportTimerFunc(void* arg) {
  bleGamepad.sendReport();
}

const esp_timer_create_args_t bleTimer_args = {
    .callback = &bleReportTimerFunc,    // Point to callback function
    .name = "bleReportTimer"            // Optional string identifier
};
esp_timer_handle_t bleReportTimer = NULL;

// ===================== Setup =====================
void setup() {
  // Start serial for debugging
#ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Starting serial");
#endif

  // Charging state monitor
#ifdef CHARGING_PIN
  pinMode(CHARGING_PIN, INPUT_PULLUP);
#endif

  // Enable wakeup pin using ext0
  esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 0);

  // Setup interrupt buttons
  InterruptButton::setMenuCount(0);
  InterruptButton::setMenuLevel(0);
  //InterruptButton::m_RTOSservicerStackDepth = 8192;
  interruptButtonBind();
  interruptButtonDisableUnused();

  // Whammy ADC
#ifdef ARDUINO_ARCH_ESP32
  analogSetPinAttenuation(PIN_WHAMMY_ADC, ADC_11db);
  analogReadResolution(12);
#endif
  pinMode(PIN_WHAMMY_ADC, INPUT);

  // BLE-Gamepad Config
  BleGamepadConfiguration cfg;
  cfg.setVid(0x0200);
  cfg.setPid(0x1bad);
  cfg.setSerialNumber("13371337");
  cfg.setSoftwareRevision("0.2");
  cfg.setHardwareRevision("1.0");

  // Set the preferred local MTU size BEFORE initializing the gamepad/BLE stack
  // Valid values are between 23 and 517.
  NimBLEDevice::setMTU(247);

  cfg.setAutoReport(false);
  cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  cfg.setButtonCount(NUM_BUTTONS_TOTAL);
  cfg.setHatSwitchCount(NUM_HATS);
  //cfg.setWhichAxes(false, false, false, false, false, true, false, false); // X, Y, Z, rX, rY, rZ, Slider1, Slider2

  bleGamepad.begin(&cfg);

  // Setup bleReportTimer 2ms rate
  esp_timer_create(&bleTimer_args, &bleReportTimer);

  bleGamepad.sendReport();
}

// ===================== Loop =====================
void loop() {
  if (!bleGamepad.isConnected()) return;

  // start timer if not running
  if (bleReportTimer != NULL) if (esp_timer_is_active(bleReportTimer) == 0) esp_timer_start_periodic(bleReportTimer, 2000);

  unsigned long now = millis();
  bool changed = false;

  // Start serial monitor if USB is connected
  //if ((HWCDCSerial) && !(Serial)) {
  //  Serial.begin(115200);
  //  Serial.println("Restarting serial");
  //}

  // deinit rtc_gpio on wakeup
  if (wakeup) {
    rtc_gpio_deinit(WAKEUP_PIN);
    wakeup = false;
    Serial.println("I'm awake!");
    lastTimeoutCheck = now;
  }

  changed |= handleWhammy();

  // Reset sleep timeout if any keys pressed
  if (changed || keyPressed) lastTimeoutCheck = now;

#ifdef ADC_PIN
  changed |= handleBattery();

  if (lastTimeoutCheck + BATTERY_TIMEOUT < now) {
    Serial.println("Going to sleep!");

    rtc_gpio_pullup_en(WAKEUP_PIN);
    rtc_gpio_pulldown_dis(WAKEUP_PIN);
    wakeup = true;

    esp_deep_sleep_start();
  }
#endif

  // Send report if any changes
  //if (changed || keyPressed) bleGamepad.sendReport(); keyPressed = false;

  // Fall back to sending reports on changes if timer isn't working
  if (bleReportTimer != NULL) if (esp_timer_is_active(bleReportTimer) == 0) if (changed || keyPressed) bleGamepad.sendReport();

  // Reset keypressed to false
  if (changed || keyPressed) keyPressed = false;
}

// ===================== FUNCTIONS =====================

static inline int analogReadMedian5(uint8_t pin) {
  int v[5];
  v[0] = analogRead(pin);
  v[1] = analogRead(pin);
  v[2] = analogRead(pin);
  v[3] = analogRead(pin);
  v[4] = analogRead(pin);
  // ordenación simple (insertion sort)
  for (int i=1;i<5;i++){
    int key=v[i]; int j=i-1;
    while (j>=0 && v[j]>key){ v[j+1]=v[j]; j--; }
    v[j+1]=key;
  }
  return v[2]; // mediana
}

int avgAnalogRead(int pinNumber, int reads) {
  int totalValue = 0;
  for (int i = 0; i < reads; i++) {
    totalValue += analogRead(pinNumber);
  }
  return (int) (totalValue / reads);
}

bool handleWhammy() {
  const uint32_t now = millis();
  if (now - lastWhammyMs < WHAMMY_INTERVAL_MS) return false;
  lastWhammyMs = now;

  // 1) Lectura robusta
  int raw = analogReadMedian5(PIN_WHAMMY_ADC);
  if (rawEma < 0) { rawEma = raw; baselineRaw = raw; }  // init
  // EMA sobre la mediana
  rawEma = (int)(EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * rawEma);

  // Inicializa/ajusta línea base (sólo cuando no estamos enganchados)
  if (!whammyEngaged) {
    // Si estamos cerca (ruido), recentra lentamente
    if (abs(rawEma - baselineRaw) <= RAW_NOISE_FLOOR) {
      baselineRaw = (int)(BASELINE_ALPHA_IDLE * rawEma + (1.0f - BASELINE_ALPHA_IDLE) * baselineRaw);
    }
  }

  // 2) Lógica de puerta con histéresis en crudo (antes de mapear a HID)
  int delta = abs(rawEma - baselineRaw);
  if (!whammyEngaged) {
    if (delta >= GATE_ENTER_DELTA) whammyEngaged = true;
    else {
      // Quieto → no enviar nada y mantener salida en el neutral actual
      int16_t neutralX = mapRawToHidX(baselineRaw);
      if (abs(neutralX - lastSentX) >= HID_DEADBAND_X) {
        bleGamepad.setRZ(neutralX+16368);   // o setSlider1(neutralX)
        lastSentX = neutralX;
        return true;
      }
      return false;
    }
  } else { // estaba enganchado
    if (delta <= GATE_EXIT_DELTA) {
      whammyEngaged = false;
      // al desenganchar, vuelve suave hacia la base
    }
  }

  // 3) Mapear y aplicar deadband en HID
  int16_t x = mapRawToHidX(rawEma);
  if (abs(x - lastSentX) < HID_DEADBAND_X) return false;

  // Fix clone hero whammy axis
  x = map(x, 0, 32737, 16368, 32737);

  bleGamepad.setRZ(x);
  lastSentX = x;
  return true;
}

void batteryReport() {
  Serial.print("Volts: ");
  Serial.println(battery.getBatteryVolts());
	
  Serial.print("Charge level: ");
  Serial.println(battery.getBatteryChargeLevel());

  Serial.print("ADC: ");
  Serial.println(analogRead(ADC_PIN));
}

bool readBattery() {
  bool batteryChange = false;
  uint8_t batteryLevelNew = 0;

  // Check if battery is conencted
  if (avgAnalogRead(ADC_PIN, 20) < ADC_DIVIDER+10) {

#ifdef DEBUG
    batteryReport();
#endif

    batteryLevelNew = battery.getBatteryChargeLevel();
    batteryState = true;
  } else {
    Serial.println("No battery detected!");

    batteryLevelNew = 0;
    batteryState = false;
  }

  // Only if battery level changes
  if (batteryLevel != batteryLevelNew) {
    Serial.println("Updated battery state");

    // Set BLE power state
    if (batteryState) {
      bleGamepad.setBatteryPowerInformation(POWER_STATE_PRESENT);
    } else {
      bleGamepad.setBatteryPowerInformation(POWER_STATE_NOT_PRESENT);
    }
    if (batteryLevelNew <= 30) {
      bleGamepad.setPowerLevel(POWER_STATE_CRITICAL); 
    } else {
      bleGamepad.setPowerLevel(POWER_STATE_GOOD);
    }

    bleGamepad.setBatteryLevel(batteryLevelNew);
    batteryLevel = batteryLevelNew;

    batteryChange = true;
  }

  return batteryChange;
}

bool readCharging() {
  bool batteryChange = false;
  bool batteryChargingNew = false;

  // Check charging state
  int charging = digitalRead(CHARGING_PIN);
  if (charging == 0) {          // Charging
    batteryChargingNew = true;
  } else if (charging == 1) {   // Not charging
    batteryChargingNew = false;
  }
  Serial.print("Charging pin: ");
  Serial.println(charging);

  if (batteryCharging != batteryChargingNew) {
    if (batteryChargingNew) {        // Charging
      Serial.println("Charging!");
      bleGamepad.setChargingState(POWER_STATE_CHARGING);
    } else { // Not charging
      Serial.println("Not Charging!");
      bleGamepad.setChargingState(POWER_STATE_NOT_CHARGING);
    }

    batteryCharging = batteryChargingNew;
    batteryChange = true;
  }

  return batteryChange;
}

bool handleBattery() {
  bool anyChange = false;
  unsigned long now = millis();

  // Update BLE battery level on an interval
  if (lastBatteryCheck == 0 || lastBatteryCheck + BATTERY_REPORT_INTERVAL < now) {
    lastBatteryCheck = now;
    anyChange |= readBattery();
    anyChange |= readCharging();
  }

  return anyChange;
}

void interruptButtonDisableUnused() {
  fretGreen.disableEvent(Event_LongKeyPress);
  fretGreen.disableEvent(Event_AutoRepeatPress);
  fretGreen.disableEvent(Event_DoubleClick);
  
  fretRed.disableEvent(Event_LongKeyPress);
  fretRed.disableEvent(Event_AutoRepeatPress);
  fretRed.disableEvent(Event_DoubleClick);

  fretYellow.disableEvent(Event_LongKeyPress);
  fretYellow.disableEvent(Event_AutoRepeatPress);
  fretYellow.disableEvent(Event_DoubleClick);

  fretBlue.disableEvent(Event_LongKeyPress);
  fretBlue.disableEvent(Event_AutoRepeatPress);
  fretBlue.disableEvent(Event_DoubleClick);

  fretOrange.disableEvent(Event_LongKeyPress);
  fretOrange.disableEvent(Event_AutoRepeatPress);
  fretOrange.disableEvent(Event_DoubleClick);

  strumUp.disableEvent(Event_LongKeyPress);
  strumUp.disableEvent(Event_AutoRepeatPress);
  strumUp.disableEvent(Event_DoubleClick);

  strumDown.disableEvent(Event_LongKeyPress);
  strumDown.disableEvent(Event_AutoRepeatPress);
  strumDown.disableEvent(Event_DoubleClick);

  // start.disableEvent(Event_All);
  // menu.disableEvent(Event_All);
  // home.disableEvent(Event_All);
  // reset.disableEvent(Event_All);

  // up.disableEvent(Event_All);
  // down.disableEvent(Event_All);
  // left.disableEvent(Event_All);
  // right.disableEvent(Event_All);
}

void interruptButtonBind() {
  fretGreen.bind    (Event_KeyDown, [](){buttonDown(BTN_FRET_GREEN); });
  fretGreen.bind    (Event_KeyUp,   [](){buttonUp(BTN_FRET_GREEN); });
  fretRed.bind      (Event_KeyDown, [](){buttonDown(BTN_FRET_RED); });
  fretRed.bind      (Event_KeyUp,   [](){buttonUp(BTN_FRET_RED); });
  fretYellow.bind   (Event_KeyDown, [](){buttonDown(BTN_FRET_YELLOW); });
  fretYellow.bind   (Event_KeyUp,   [](){buttonUp(BTN_FRET_YELLOW); });
  fretBlue.bind     (Event_KeyDown, [](){buttonDown(BTN_FRET_BLUE); });
  fretBlue.bind     (Event_KeyUp,   [](){buttonUp(BTN_FRET_BLUE); });
  fretOrange.bind   (Event_KeyDown, [](){buttonDown(BTN_FRET_ORANGE); });
  fretOrange.bind   (Event_KeyUp,   [](){buttonUp(BTN_FRET_ORANGE); });

  strumUp.bind      (Event_KeyDown, [](){buttonDown(BTN_STRUM_UP); });
  strumUp.bind      (Event_KeyUp,   [](){buttonUp(BTN_STRUM_UP); });
  strumDown.bind    (Event_KeyDown, [](){buttonDown(BTN_STRUM_DOWN); });
  strumDown.bind    (Event_KeyUp,   [](){buttonUp(BTN_STRUM_DOWN); });

  start.bind        (Event_KeyDown, [](){buttonDown(BTN_START); });
  start.bind        (Event_KeyUp,   [](){buttonUp(BTN_START); });
  menu.bind         (Event_KeyDown, [](){buttonDown(BTN_MENU); });
  menu.bind         (Event_KeyUp,   [](){buttonUp(BTN_MENU); });
  home.bind         (Event_KeyDown, [](){buttonDown(BTN_HOME); });
  home.bind         (Event_KeyUp,   [](){buttonUp(BTN_HOME); });
  reset.bind        (Event_KeyDown, [](){buttonDown(BTN_RESET); });
  reset.bind        (Event_KeyUp,   [](){buttonUp(BTN_RESET); });

  up.bind           (Event_KeyDown, [](){buttonDown(BTN_UP); });
  up.bind           (Event_KeyUp,   [](){buttonUp(BTN_UP); });
  down.bind         (Event_KeyDown, [](){buttonDown(BTN_DOWN); });
  down.bind         (Event_KeyUp,   [](){buttonUp(BTN_DOWN); });
  left.bind         (Event_KeyDown, [](){buttonDown(BTN_LEFT); });
  left.bind         (Event_KeyUp,   [](){buttonUp(BTN_LEFT); });
  right.bind        (Event_KeyDown, [](){buttonDown(BTN_RIGHT); });
  right.bind        (Event_KeyUp,   [](){buttonUp(BTN_RIGHT); });
}