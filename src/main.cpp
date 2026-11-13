#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ---------- ESP32 PINS ----------
#define DATA_PIN     23
#define CLOCK_PIN    18
#define LATCH_PIN    19

#define LOAD_PIN     4
#define BUTTON_DATA  16

#define NORTH_POT 34
#define SOUTH_POT 35
#define EAST_POT  32
#define WEST_POT  33

// ---------- LCD ----------
#define SDA_PIN 21
#define SCL_PIN 22

// ---------- WIFI ----------
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

const char* MQTT_SERVER = "thingsboard.cloud";
const int MQTT_PORT = 1883;

const char* ACCESS_TOKEN = "YOUR TOKEN";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- TELEMETRY ----------
unsigned long lastTelemetrySend = 0;
const unsigned long TELEMETRY_INTERVAL = 2000;

// ---------- LCD UPDATE ----------
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 250;

// ---------- ROAD DEFINITIONS ----------
enum Road {
  NORTH = 0,
  SOUTH,
  EAST,
  WEST
};

enum SignalState {
  GREEN,
  YELLOW,
  ALL_RED
};

const char* roadNames[] = {
  "NORTH", "SOUTH", "EAST", "WEST"
};

// ---------- LED MAPPING ----------
const byte redBit[]    = {0, 3, 6, 9};
const byte yellowBit[] = {1, 4, 7, 10};
const byte greenBit[]  = {2, 5, 8, 11};

const uint16_t RED_MASK =
  (1 << 0) | (1 << 3) | (1 << 6) | (1 << 9);

// ---------- TIMING ----------
const unsigned long YELLOW_TIME = 2000;
const unsigned long ALL_RED_TIME = 500;
const unsigned long DEBOUNCE_TIME = 30;

// ---------- CONTROLLER VARIABLES ----------
SignalState state = ALL_RED;

int currentRoad = WEST;

unsigned long stateStart = 0;
unsigned long greenDuration = 5000;

byte pendingEmergency = 0;
bool currentEmergency = false;

// ---------- BUTTON VARIABLES ----------
byte rawPrevious = 0x0F;
byte stableButtons = 0x0F;

unsigned long lastChange[4] = {0, 0, 0, 0};

// ---------- FUNCTION DECLARATIONS ----------
int readDensity(int road);
const char* densityName(int value);
void updateLCD();
void connectWiFi();
void connectMQTT();
void sendTrafficTelemetry();

// ---------- LCD HELPERS ----------
void printLCDLine(byte row, const char* message) {

  char line[17];

  snprintf(line, sizeof(line), "%-16.16s", message);

  lcd.setCursor(0, row);
  lcd.print(line);
}

int firstPendingEmergency() {

  for (int offset = 1; offset <= 4; offset++) {

    int road = (currentRoad + offset) % 4;

    if (pendingEmergency & (1 << road)) {
      return road;
    }
  }

  return -1;
}

// ---------- LCD UPDATE ----------
void updateLCD() {

  unsigned long now = millis();

  if (now - lastLCDUpdate < LCD_UPDATE_INTERVAL) {
    return;
  }

  lastLCDUpdate = now;

  char line1[17];
  char line2[17];

  int waitingRoad = firstPendingEmergency();

  if (state == GREEN) {

    int density = readDensity(currentRoad);

    snprintf(
      line1,
      sizeof(line1),
      "%.5s: %.6s",
      roadNames[currentRoad],
      densityName(density)
    );

    unsigned long elapsed = now - stateStart;
    unsigned long remaining = 0;

    if (elapsed < greenDuration) {
      remaining = (greenDuration - elapsed + 999) / 1000;
    }

    if (currentEmergency) {

      snprintf(
        line2,
        sizeof(line2),
        "EMERG GREEN %lus",
        remaining
      );
    }
    else {

      snprintf(
        line2,
        sizeof(line2),
        "GREEN: %02lu SEC",
        remaining
      );
    }
  }

  else if (state == YELLOW) {

    snprintf(
      line1,
      sizeof(line1),
      "%.5s: YELLOW",
      roadNames[currentRoad]
    );

    if (waitingRoad >= 0) {

      snprintf(
        line2,
        sizeof(line2),
        "WAIT: %.5s",
        roadNames[waitingRoad]
      );
    }
    else {
      snprintf(line2, sizeof(line2), "SIGNAL CHANGING");
    }
  }

  else {

    snprintf(line1, sizeof(line1), "ALL RED");

    if (waitingRoad >= 0) {

      snprintf(
        line2,
        sizeof(line2),
        "EMERG: %.5s",
        roadNames[waitingRoad]
      );
    }
    else {
      snprintf(line2, sizeof(line2), "NEXT: WAITING");
    }
  }

  printLCDLine(0, line1);
  printLCDLine(1, line2);
}

// ---------- SHIFT REGISTER ----------
void updateRegisters(uint16_t ledMask) {

  byte firstIC = ledMask & 0xFF;
  byte secondIC = (ledMask >> 8) & 0xFF;

  digitalWrite(LATCH_PIN, LOW);

  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, secondIC);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, firstIC);

  digitalWrite(LATCH_PIN, HIGH);
}

void showAllRed() {
  updateRegisters(RED_MASK);
}

void showRoadSignal(int road, SignalState signal) {

  uint16_t mask = RED_MASK;

  mask &= ~(1 << redBit[road]);

  if (signal == GREEN) {
    mask |= (1 << greenBit[road]);
  }
  else if (signal == YELLOW) {
    mask |= (1 << yellowBit[road]);
  }

  updateRegisters(mask);
}

// ---------- POTENTIOMETER ----------
const int potPins[] = {
  NORTH_POT,
  SOUTH_POT,
  EAST_POT,
  WEST_POT
};

int readDensity(int road) {
  return analogRead(potPins[road]);
}

const char* densityName(int value) {

  if (value <= 1200) return "LOW";
  if (value <= 2800) return "MEDIUM";

  return "HIGH";
}

unsigned long calculateGreenTime(int value) {

  if (value <= 1200) return 5000;
  if (value <= 2800) return 8000;

  return 12000;
}

// ---------- 74HC165 BUTTON INPUT ----------
byte readButtons() {

  digitalWrite(LOAD_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(LOAD_PIN, HIGH);

  return shiftIn(BUTTON_DATA, CLOCK_PIN, MSBFIRST);
}

void captureEmergencyRequests() {

  byte buttonData = readButtons();

  unsigned long now = millis();

  for (int road = 0; road < 4; road++) {

    bool rawPressed = !(buttonData & (1 << road));
    bool previousRawPressed = !(rawPrevious & (1 << road));

    if (rawPressed != previousRawPressed) {

      lastChange[road] = now;

      if (rawPressed) {
        rawPrevious &= ~(1 << road);
      }
      else {
        rawPrevious |= (1 << road);
      }
    }

    bool stablePressed = !(stableButtons & (1 << road));

    if (rawPressed != stablePressed &&
        now - lastChange[road] >= DEBOUNCE_TIME) {

      if (rawPressed) {

        stableButtons &= ~(1 << road);

        if (road != currentRoad || state == ALL_RED) {

          pendingEmergency |= (1 << road);

          Serial.print("EMERGENCY REQUEST QUEUED: ");
          Serial.println(roadNames[road]);
        }
      }
      else {
        stableButtons |= (1 << road);
      }
    }
  }
}

// ---------- FAIR ROAD SELECTION ----------
int selectNextRoad() {

  for (int offset = 1; offset <= 4; offset++) {

    int road = (currentRoad + offset) % 4;

    if (pendingEmergency & (1 << road)) {

      Serial.print("EMERGENCY PRIORITY: ");
      Serial.println(roadNames[road]);

      return road;
    }
  }

  return (currentRoad + 1) % 4;
}

// ---------- START GREEN ----------
void startGreen(int road) {

  currentRoad = road;

  currentEmergency = pendingEmergency & (1 << road);

  pendingEmergency &= ~(1 << road);

  int density = readDensity(road);

  unsigned long normalDuration = calculateGreenTime(density);

  if (currentEmergency) {

    greenDuration = max(normalDuration, 10000UL);

    Serial.println("EMERGENCY GREEN TIME EXTENDED");
  }
  else {
    greenDuration = normalDuration;
  }

  showRoadSignal(road, GREEN);

  state = GREEN;
  stateStart = millis();

  Serial.println("\n----------------------");

  Serial.print("GREEN ROAD: ");
  Serial.println(roadNames[road]);

  Serial.print("Traffic density: ");
  Serial.println(densityName(density));

  Serial.print("Green duration: ");
  Serial.print(greenDuration / 1000);
  Serial.println(" seconds");

  Serial.print("Pending emergency mask: ");
  Serial.println(pendingEmergency, BIN);

  lastLCDUpdate = 0;
  updateLCD();
}

// ---------- WIFI ----------
void connectWiFi() {

  Serial.print("Connecting to WiFi\n");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi Connected!");
  Serial.println(WiFi.localIP());
}

// ---------- MQTT ----------
void connectMQTT() {

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  while (!mqttClient.connected()) {

    Serial.println("Connecting to ThingsBoard...");

    if (mqttClient.connect(
          "ESP32_ATMS",
          ACCESS_TOKEN,
          NULL
        )) {

      Serial.println("ThingsBoard Connected!");
    }
    else {

      Serial.print("MQTT failed, state: ");
      Serial.println(mqttClient.state());

      delay(2000);
    }
  }
}
// ---------- LIVE TELEMETRY ----------
  void sendTrafficTelemetry() {

  int northDensity = readDensity(NORTH);
  int southDensity = readDensity(SOUTH);
  int eastDensity  = readDensity(EAST);
  int westDensity  = readDensity(WEST);

  const char* signalStatus;

  if (state == GREEN) {
    signalStatus = "GREEN";
  }
  else if (state == YELLOW) {
    signalStatus = "YELLOW";
  }
  else {
    signalStatus = "ALL_RED";
  }

  unsigned long remainingGreen = 0;

  if (state == GREEN) {

    unsigned long elapsed = millis() - stateStart;

    if (elapsed < greenDuration) {
      remainingGreen = (greenDuration - elapsed + 999) / 1000;
    }
  }

  char payload[512];

  snprintf(
    payload,
    sizeof(payload),

    "{\"north_density\":%d,"
    "\"south_density\":%d,"
    "\"east_density\":%d,"
    "\"west_density\":%d,"
    "\"active_road\":\"%s\","
    "\"signal_status\":\"%s\","
    "\"green_duration_sec\":%lu,"
    "\"remaining_green_sec\":%lu,"
    "\"emergency_active\":%s,"
    "\"pending_emergency_mask\":%u,"
    "\"north_emergency_pending\":%s,"
    "\"south_emergency_pending\":%s,"
    "\"east_emergency_pending\":%s,"
    "\"west_emergency_pending\":%s}",

    northDensity,
    southDensity,
    eastDensity,
    westDensity,

    roadNames[currentRoad],
    signalStatus,

    greenDuration / 1000,
    remainingGreen,

    currentEmergency ? "true" : "false",

    pendingEmergency,

    (pendingEmergency & (1 << NORTH)) ? "true" : "false",
    (pendingEmergency & (1 << SOUTH)) ? "true" : "false",
    (pendingEmergency & (1 << EAST)) ? "true" : "false",
    (pendingEmergency & (1 << WEST)) ? "true" : "false"
  );

  bool success = mqttClient.publish(
    "v1/devices/me/telemetry",
    payload
  );

  if (success) {
    Serial.println("ATMS telemetry sent!");
  }
  else {
    Serial.println("Telemetry publish failed!");
  }
}
 
// ---------- SETUP ----------
void setup() {

  Serial.begin(115200);

  connectWiFi();

  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  pinMode(LOAD_PIN, OUTPUT);
  pinMode(BUTTON_DATA, INPUT);

  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(LATCH_PIN, HIGH);
  digitalWrite(LOAD_PIN, HIGH);

  analogReadResolution(12);

  // Initialize LCD
  Wire.begin(SDA_PIN, SCL_PIN);

  lcd.init();
  lcd.backlight();

  printLCDLine(0, "ATMS SYSTEM");
  printLCDLine(1, "INITIALIZING...");

  showAllRed();

  state = ALL_RED;
  stateStart = millis();

  delay(1500);

  printLCDLine(0, "TRAFFIC CONTROL");
  printLCDLine(1, "SYSTEM READY");

  delay(1500);

  lastLCDUpdate = 0;
  updateLCD();

  // MQTT configuration
  mqttClient.setBufferSize(512);
  mqttClient.setKeepAlive(60);

  connectMQTT();

  // Initial telemetry
  sendTrafficTelemetry();

  lastTelemetrySend = millis();
}

// ---------- MAIN LOOP ----------
void loop() {

  captureEmergencyRequests();

  unsigned long now = millis();

  switch (state) {

    case GREEN:

      if (now - stateStart >= greenDuration) {

        showRoadSignal(currentRoad, YELLOW);

        state = YELLOW;
        stateStart = now;

        Serial.print(roadNames[currentRoad]);
        Serial.println(" -> YELLOW");

        lastLCDUpdate = 0;
        updateLCD();
      }

      break;

    case YELLOW:

      if (now - stateStart >= YELLOW_TIME) {

        showAllRed();

        state = ALL_RED;
        stateStart = now;

        Serial.println("ALL RED");

        lastLCDUpdate = 0;
        updateLCD();
      }

      break;

    case ALL_RED:

      if (now - stateStart >= ALL_RED_TIME) {

        int nextRoad = selectNextRoad();

        startGreen(nextRoad);
      }

      break;
  }

  // Maintain WiFi
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  // Maintain MQTT
  if (!mqttClient.connected()) {
    connectMQTT();
  }

  mqttClient.loop();

  // Send telemetry every 2 seconds
  if (millis() - lastTelemetrySend >= TELEMETRY_INTERVAL) {

    lastTelemetrySend = millis();

    sendTrafficTelemetry();
  }

  updateLCD();
}