#include <WiFi.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <EEPROM.h>

// Wi-Fi credentials
const char* ssid = "RRBJ-MIFI";
const char* password = "0099ravindu";

// Time settings (Sri Lanka GMT+5:30)
const char* ntpServer = "pool.ntp.org";  
const long gmtOffset_sec = 19800;  
const int daylightOffset_sec = 0;

// Servo settings
Servo myServo;
const int servoPin = 18;

// Bulb (Relay) settings
const int relayPin = 19;

// Water Temperature Sensor (DS18B20) - I2C optimization not applicable
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Heater control
const int heaterPin = 21;

// Turbidity Sensor (ADC Input)
const int turbidityPin = 32;  
const int acFilterPin = 22;  

// EEPROM Addresses
#define EEPROM_SIZE 10
#define SERVO_DONE_ADDR 0  // EEPROM address for storing servo status

bool servoDone = false;
unsigned long lastCheckTime = 0;

// Function prototypes
void connectToWiFi();
void initNTP();
void printLocalTime();
float readWaterTemperature();
float readTurbidity();
void rotateServo();
void restoreServoState();
void saveServoState(bool state);

void setup() {
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);
    
    connectToWiFi();
    initNTP();

    // Initialize devices
    myServo.attach(servoPin);
    pinMode(relayPin, OUTPUT);
    pinMode(heaterPin, OUTPUT);
    pinMode(acFilterPin, OUTPUT);
    
    digitalWrite(relayPin, LOW);  // Bulb OFF initially
    digitalWrite(heaterPin, LOW); // Heater OFF initially
    digitalWrite(acFilterPin, LOW); // AC Filter OFF initially

    sensors.begin();
    restoreServoState();
}

void loop() {
    unsigned long currentMillis = millis();
    
    // Run every 1 second
    if (currentMillis - lastCheckTime >= 1000) {
        lastCheckTime = currentMillis;

        struct tm timeInfo;
        if (!getLocalTime(&timeInfo)) {
            Serial.println("Failed to obtain time");
            return;
        }

        int currentHour = timeInfo.tm_hour;
        int currentMinute = timeInfo.tm_min;

        // Servo rotation at 11:00 AM (Only once)
        if (currentHour == 20 && currentMinute == 49 && !servoDone) {
            rotateServo();
            servoDone = true;
            saveServoState(true);
        }

        // Bulb ON at 6:00 AM, OFF at 6:00 PM (using hours and minutes)
        if (currentHour == 6 && currentMinute == 00) {
            digitalWrite(relayPin, HIGH);  // Bulb ON
            Serial.println("Bulb ON at 6:00 AM");
        } 
        else if (currentHour == 18 && currentMinute == 00) {
            digitalWrite(relayPin, LOW);  // Bulb OFF
            Serial.println("Bulb OFF at 6:00 PM");
        }

        // Water temperature control
        float waterTemp = readWaterTemperature();
        digitalWrite(heaterPin, (waterTemp < 23.0) ? LOW : HIGH);
        Serial.printf("Heater State: %s\n", (digitalRead(heaterPin) ? "OFF" : "ON"));

        // Turbidity control
        float turbidity = readTurbidity();
        digitalWrite(acFilterPin, (turbidity > 25.0) ? LOW : HIGH);
        Serial.printf("AC Filter State: %s\n", (digitalRead(acFilterPin) ? "OFF" : "ON"));
    }
}

// Wi-Fi Connection
void connectToWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println(WiFi.status() == WL_CONNECTED ? "\nWi-Fi connected!" : "\nFailed to connect to Wi-Fi.");
}

// Initialize NTP
void initNTP() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
}

// Print local time
void printLocalTime() {
    struct tm timeInfo;
    if (!getLocalTime(&timeInfo)) {
        Serial.println("Time not available");
        return;
    }
    Serial.printf("Current Time: %02d:%02d:%02d\n", timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
}

// Read water temperature
float readWaterTemperature() {
    sensors.requestTemperatures();
    float temperature = sensors.getTempCByIndex(0);
    Serial.printf("Water Temperature: %.2f°C\n", temperature);
    return temperature;
}

// Read turbidity (with improved ADC handling)
float readTurbidity() {
    int turbidityRaw = analogRead(turbidityPin);
    float turbidity = map(turbidityRaw, 0, 4095, 100, 0);
    Serial.printf("Turbidity: %.2f NTU\n", turbidity);
    return turbidity;
}

// Rotate servo motor
void rotateServo() {
    Serial.println("Rotating servo..."); 
    for (int i = 0; i < 4; i++) {
        myServo.write(90);
        delay(500);
        myServo.write(0);
        delay(500);
    }
    Serial.println("Rotation complete.");
}

// Restore servo state from EEPROM
void restoreServoState() {
    servoDone = EEPROM.read(SERVO_DONE_ADDR);
}

// Save servo state to EEPROM
void saveServoState(bool state) {
    EEPROM.write(SERVO_DONE_ADDR, state);
    EEPROM.commit();
}

