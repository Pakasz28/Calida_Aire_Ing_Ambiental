#include <WiFi.h>
#include <WiFiClient.h>
#include "Adafruit_PM25AQI.h"

// Configuración de pines y red WiFi
#define RX2_PIN 16          // GPIO 16 como RX2
#define TX2_PIN 17          // GPIO 17 como TX2
#define ANALOG_PIN 34       // Pin analógico para el sensor adicional (por ejemplo, un MQ135)
const char* ssid = "Joe";              // Reemplaza con tu SSID de WiFi
const char* password = "12345678";     // Reemplaza con tu contraseña de WiFi
const char* server = "api.thingspeak.com"; // Servidor de ThingSpeak
const char* apiKey = "C2B8SYRQXTIX5PND";  // Reemplaza con tu API Key de ThingSpeak

HardwareSerial Serial2_PM25(2); 
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Iniciando ESP32 con ThingSpeak y sensor PM2.5 y PM10");

  // Conectar a WiFi
  connectToWiFi();

  // Configurar Serial2 y sensor
  Serial2_PM25.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);

  // Esperar 2 segundos para que el sensor esté listo
  delay(2000);

  // Inicializar el sensor de calidad del aire
  if (!aqi.begin_UART(&Serial2_PM25)) {
    Serial.println("No se pudo encontrar el sensor PM2.5!");
    while (1) { delay(1000); }
  }
  Serial.println("Sensor PM2.5 y PM10 encontrado exitosamente!");
}

void loop() {
  // Verificar la conexión WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconectando a WiFi...");
    connectToWiFi();
  }

  // Intentar leer datos del sensor de calidad del aire
  PM25_AQI_Data data;
  bool success = false;
  for (int attempt = 0; attempt < 5; attempt++) { // Intentar 5 veces
    if (aqi.read(&data)) {
      success = true;
      break;
    } else {
      Serial.println("Intento de lectura fallido, reintentando...");
      delay(1000); // Esperar 1 segundo antes de reintentar
    }
  }

  if (success) {
    Serial.println("Lectura AQI exitosa");
    Serial.print("PM 2.5: ");
    Serial.print(data.pm25_standard);
    Serial.println(" µg/m³");

    Serial.print("PM 10: ");
    Serial.print(data.pm10_standard);
    Serial.println(" µg/m³");

    // Leer el valor del sensor analógico (MQ135 u otro)
    int sensorValue = analogRead(ANALOG_PIN);
    float co2_ppm = calculateCO2(sensorValue);

    Serial.print("Valor del sensor analógico: ");
    Serial.print(sensorValue);
    Serial.print(" - CO2: ");
    Serial.print(co2_ppm);
    Serial.println(" ppm");

    // Enviar datos a ThingSpeak
    sendToThingSpeak(data.pm25_standard, data.pm10_standard, co2_ppm);
  } else {
    Serial.println("No se pudo leer del AQI después de varios intentos");
  }

  delay(60000); // Espera 1 minuto antes de la siguiente lectura
}

float calculateCO2(int sensorValue) {
  const float R0 = 10.0;
  float resistance = (1023.0 / sensorValue - 1) * R0;
  float ppm = (resistance / R0) * 5000;
  if (ppm < 0) ppm = 0;
  return ppm;
}

void sendToThingSpeak(float pm2_5, float pm10, float co2) {
  WiFiClient client;
  if (client.connect(server, 80)) {
    String url = String("/update?api_key=") + apiKey +
                 "&field1=" + String(pm2_5) +
                 "&field2=" + String(pm10) +
                 "&field3=" + String(co2);
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + server + "\r\n" +
                 "Connection: close\r\n\r\n");
    delay(100);

    // Diagnóstico para verificar la respuesta del servidor
    while (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println(line); // Imprime la respuesta del servidor
    }
    
    Serial.println("Datos enviados a ThingSpeak");
  } else {
    Serial.println("Error de conexión a ThingSpeak");
  }
  client.stop();
}

void connectToWiFi() {
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 200) {
    delay(100); 
    attempts++;
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado");
    Serial.print("Dirección IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nError al conectar WiFi");
  }
}


