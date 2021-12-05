// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Colocar nombre de la red Wifi (SSID) y contraseña
const char* ssid = "carlos_covid19";
const char* password = "bunny3101";
// const char* ssid = "WeWork";
// const char* password = "P@ssw0rd";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.1.55";
// const char* mqtt_server = "10.228.6.125";

WiFiClient espClient;
PubSubClient client(espClient);

// Define some constants (datasheet)
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
#define RAD_A_DEG = 57.295779
#define LED_PIN    18 

Adafruit_MPU6050 mpu;

float distancia;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      client.subscribe("guante/dist_lidar");// Subscribe
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  char dist_char[4];
  Serial.print("Callback - ");
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    dist_char[i] = (char)payload[i];
  }
  distancia = atof(dist_char);
  Serial.println(distancia);
}

void setup(void) {
  
  pinMode(LED_PIN, OUTPUT); // set ESP32 pin to output mode

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("MPU6050 - HANDTIC");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println(mpu.begin());
  delay(10);
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  digitalWrite(LED_PIN, LOW);
}

void loop() {

  // Inicializar comunicacion inalambrica
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer data de MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  // Compute the orientation angles ([0]: PITCH, [1]: ROLL, [2]: YAW)
  float orientation[3]={};
  orientation[0] = atan((a.acceleration.y/A_R)/sqrt(pow((a.acceleration.x/A_R),2) + pow((a.acceleration.z/A_R),2)))*RAD_TO_DEG;
  orientation[1] = atan(-1*(a.acceleration.x/A_R)/sqrt(pow((a.acceleration.y/A_R),2) + pow((a.acceleration.z/A_R),2)))*RAD_TO_DEG;

  // Angulos de giroscopio
  float Gy[3]={};
  Gy[0] = g.gyro.x/G_R;
  Gy[1] = g.gyro.y/G_R;
  Gy[2] = g.gyro.x/G_R;

  // Calcula tiempo
  long tiempo_prev=0;
  float dt;
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  // Filtro complementario
  orientation[0] = 0.98 *(orientation[0]+Gy[0]*dt) + 0.02*orientation[0];
  orientation[1] = 0.98 *(orientation[1]+Gy[1]*dt) + 0.02*orientation[1];

  // Imprimir orientacion por consola
  Serial.print("PITCH: ");
  Serial.print(orientation[0]);
  Serial.println("º");
  Serial.print("ROLL: ");
  Serial.print(orientation[1]);
  Serial.println("º");

  // Convertir angulos a string
  char pitch[24];
  dtostrf(orientation[0], 1, 2, pitch);
  char roll[8];
  dtostrf(orientation[1], 1, 2, roll);
  char sep = ',';
  strcat(pitch,&sep);
  strcat(pitch,roll);

  // Publica los valores de orientacion (roll y pitch)
  client.publish("guante/orientacion", pitch);

  // Verificacion para vibrar
  float threshold = 20.0f;
  if(distancia < 0.5 && 
  !((fabs(orientation[0])<threshold && fabs(orientation[1])<threshold)))
  {
    // Activar actuador (LED)
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }

  Serial.println("");
  delay(200);
}