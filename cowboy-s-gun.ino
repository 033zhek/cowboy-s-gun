#include <Wire.h>
#include <ESP8266WiFi.h>

#define GYRO_TRESHOLD 0.11

const char* ssid     = "Keenetic-0335";
const char* password = "zBV4B3Wt";

const char* host = "192.168.1.10";
const uint16_t port = 27015;

bool trigger_is_pulled = false;

uint8_t IMUAddress = 0x68;
int16_t gyroZ;
int16_t gyroX;
int16_t gyroY;
double gyroXangle = 0;
double gyroYangle = 0;
double gyroZangle = 0;
double d_gyroXangle = 0;
double d_gyroYangle = 0;
double d_gyroZangle = 0;
char g_x_str[10];
char g_y_str[10];
char g_z_str[10];
String message = "";
String my_IP;
uint32_t timer;//timer for angle integration
uint32_t cooldown_timer = 0;
uint32_t solenoid_timer;//for turning off the solenoid
double dt;
float gyroXoffset = 0;
float gyroYoffset = 0;
float gyroZoffset = 0;

ICACHE_RAM_ATTR void pull_the_trigger(void) {
  if ((millis() - cooldown_timer > 1000) && (digitalRead(10)))trigger_is_pulled = true;
}


WiFiClient client;

void setup() {
  Wire.begin(5, 4);
  Serial.begin(115200);

  pinMode(12, OUTPUT);//the solenoid
  pinMode(10, INPUT);
  i2cWrite(0x6B, 0x00); // disable sleep mode
  i2cWrite(0x1B, 0x18); // change gyro range to +-2000dps
  i2cWrite(0x1C, 0x00); // change acc range to +-2g

  attachInterrupt(15, pull_the_trigger, RISING);


  //connecting to a wifi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println();
  Serial.println();
  Serial.print("Wait for WiFi... ");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  my_IP = toString(WiFi.localIP());
  Serial.println(my_IP);
  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);

  //calibration
  calib();

  while (!client.connect(host, port)) {
    Serial.println("connection to host failed");
    delay(5000);
  }

  timer = micros();
}

void loop() {

  message = my_IP + " ";

  uint8_t* data = i2cRead(0x43, 6);

  gyroX = ((data[0] << 8) | data[1]);
  gyroY = ((data[2] << 8) | data[3]);
  gyroZ = ((data[4] << 8) | data[5]);

  dt = (micros() - timer) / 10000000.0;

  d_gyroXangle = (double)(((gyroX - gyroXoffset) / 16.4) * dt) * RAD_TO_DEG;
  d_gyroYangle = (double)(((gyroY - gyroYoffset) / 16.4) * dt) * RAD_TO_DEG;
  d_gyroZangle = (double)(((gyroZ - gyroZoffset) / 16.4) * dt) * RAD_TO_DEG;

  timer = micros();

  gyroXangle = 0.8 * gyroXangle + 0.2 * (gyroXangle + ((d_gyroXangle > GYRO_TRESHOLD || d_gyroXangle < -GYRO_TRESHOLD) ? d_gyroXangle : 0));
  gyroYangle = 0.8 * gyroYangle + 0.2 * (gyroYangle + ((d_gyroYangle > GYRO_TRESHOLD || d_gyroYangle < -GYRO_TRESHOLD) ? d_gyroYangle : 0));
  gyroZangle = 0.8 * gyroZangle + 0.2 * (gyroZangle + ((d_gyroZangle > GYRO_TRESHOLD || d_gyroZangle < -GYRO_TRESHOLD) ? d_gyroZangle : 0));

  dtostrf(gyroXangle, 7, 2, g_x_str);
  dtostrf(gyroYangle, 7, 2, g_y_str);
  dtostrf(gyroZangle, 7, 2, g_z_str);

  message += g_x_str;
  message += " ";
  message += g_y_str;
  message += " ";
  message += g_z_str;
  message += '\n';

  if (client.connected() && trigger_is_pulled && digitalRead(10)) { // && gun_is_pulled_out && (millis() - cooldown_timer) > 1000) {
    Serial.println(message);
    client.println(message);//sending data
    trigger_is_pulled = false;
    digitalWrite(12, HIGH);//turning on the solenoid
    solenoid_timer = cooldown_timer = millis();
  }

  if (millis() - solenoid_timer > 40) {
    digitalWrite(12, LOW);//turning off the solenoid
  }

}

void i2cWrite(uint8_t registerAddress, uint8_t data) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++)
    data [i] = Wire.read();
  return data;
}

String toString(const IPAddress& address) {
  return String() + address[0] + "." + address[1] + "." + address[2] + "." + address[3];
}

void calib() {
  int calib_timer = millis();
  int count = 0;
  int16_t sum_gyroX = 0;
  int16_t sum_gyroY = 0;
  int16_t sum_gyroZ = 0;
  while (millis() - calib_timer < 3000) {
    uint8_t* data = i2cRead(0x43, 6);
    sum_gyroX += ((data[0] << 8) | data[1]);
    sum_gyroY += ((data[2] << 8) | data[3]);
    sum_gyroZ += ((data[4] << 8) | data[5]);
    count++;
    yield();
  }
  gyroXoffset = (float)(sum_gyroX / count);
  gyroYoffset = (float)(sum_gyroY / count);
  gyroZoffset = (float)(sum_gyroZ / count);
}
