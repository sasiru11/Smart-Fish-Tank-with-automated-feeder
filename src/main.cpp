#include <WiFi.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>

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

// Water Temperature Sensor (DS18B20)
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Heater control
const int heaterPin = 21;

// Turbidity Sensor
const int turbidityPin = 32;  
const int acFilterPin = 22;  

unsigned long lastCheckTime = 0;

// Function prototypes
void connectToWiFi();
void initNTP();
void printLocalTime();
float readWaterTemperature();
float readTurbidity();
void rotateServo();

void setup() {
    Serial.begin(115200);
    
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
        int currentSecond = timeInfo.tm_sec;

        // Run feeder at 11:00 AM
        if (currentHour == 11 && currentMinute == 00 && currentSecond == 0) {
            Serial.println("Feeder ON 11:00 AM");
            rotateServo();
        }
       
        // Bulb ON from 6:00 AM to 6:00 PM
        if ((currentHour > 6 || (currentHour == 6 && currentMinute >= 00)) &&
            (currentHour < 18 || (currentHour == 18 && currentMinute < 00))) {
            digitalWrite(relayPin, HIGH);  // Bulb ON
            Serial.println("Bulb is ON (6:15 AM - 6:15 PM)");
        } else {
            digitalWrite(relayPin, LOW);  // Bulb OFF
            Serial.println("Bulb is OFF (Night Mode)");
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

// Read turbidity
float readTurbidity() {
    int turbidityRaw = analogRead(turbidityPin);
    float turbidity = map(turbidityRaw, 0, 4095, 100, 0);
    Serial.printf("Turbidity: %.2f NTU\n", turbidity);
    return turbidity;
}

// Rotate servo motor
void rotateServo() {
    Serial.println("Rotating Feeder..."); 
    for (int i = 0; i < 4; i++) {
        myServo.write(135);
        delay(500);
        myServo.write(0);
        delay(500);
    }
    Serial.println("Feeding complete.");
}
