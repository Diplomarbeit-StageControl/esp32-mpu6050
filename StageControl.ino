#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const char* ssid = "A1-6D2E7F";
const char* password = "D6UVKNNPCZ";
const char* mqtt_server = "10.0.0.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
Adafruit_MPU6050 mpu;

// Globale Variablen für Beschleunigungs- und Gyro-Daten
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

void setup() {
  Serial.begin(115200); // Baudrate

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialisieren aller Werte auf 0
  accX = accY = accZ = 0;
  gyroX = gyroY = gyroZ = 0;
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe("esp32/movements");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 1000) { 
    lastMsg = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Update der globalen Variablen mit aktuellen Werten
    accX = a.acceleration.x;
    accY = a.acceleration.y;
    accZ = a.acceleration.z;
    gyroX = g.gyro.x;
    gyroY = g.gyro.y;
    gyroZ = g.gyro.z;

    char accXStr[8], accYStr[8], accZStr[8];
    dtostrf(accX, 1, 2, accXStr);
    dtostrf(accY, 1, 2, accYStr);
    dtostrf(accZ, 1, 2, accZStr);

    char gyroXStr[8], gyroYStr[8], gyroZStr[8];
    dtostrf(gyroX, 1, 2, gyroXStr);
    dtostrf(gyroY, 1, 2, gyroYStr);
    dtostrf(gyroZ, 1, 2, gyroZStr);

    String messagegyro = String(gyroXStr) + "," + String(gyroYStr) + "," + String(gyroZStr);
    client.publish("esp32/movements", messagegyro.c_str());

    // Debug-Ausgabe
    Serial.println("Acceleration: ");
    Serial.print("X: ");
    Serial.println(accX);
    Serial.print("Y: ");
    Serial.println(accY);
    Serial.print("Z: ");
    Serial.println(accZ);

    Serial.println("Gyro Data: ");
    Serial.print("X: ");
    Serial.println(gyroX);
    Serial.print("Y: ");
    Serial.println(gyroY);
    Serial.print("Z: ");
    Serial.println(gyroZ);
  }
}