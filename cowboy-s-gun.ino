#include <Wire.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>

#define GYRO_X_OFFSET_ADDR 0
#define GYRO_Y_OFFSET_ADDR 4
#define GYRO_Z_OFFSET_ADDR 8

const char* ssid     = "";
const char* password = "";

const char* host = "192.168.1.1";
const uint16_t port = 3000;

volatile boolean trigger_is_pulled = false;
volatile boolean gun_is_pulled_out = false;

uint8_t IMUAddress = 0x68;
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
int32_t sum_gyroX = 0;//variables for gyro calib
int32_t sum_gyroY = 0;
int32_t sum_gyroZ = 0;
double gyroXangle = 0;
double gyroYangle = 0;
double gyroZangle = 0;
double accZangle;
double d_gyroXangle = 0;
double d_gyroYangle = 0;
double d_gyroZangle = 0;
char a_z_str[10];
char g_x_str[10];
char g_y_str[10];
char g_z_str[10];
String message = "";
uint32_t timer;//timer for angle integration
uint32_t cooldown_timer = 0;
uint32_t solenoid_timer;//for turning off the solenoid
double dt;
float gyroXoffset = -20.96;
float gyroYoffset = -26.47;
float gyroZoffset = 33.23;

ICACHE_RAM_ATTR void pull_the_trigger(void){
    trigger_is_pulled = true;
  }

ICACHE_RAM_ATTR void pull_out_the_gun(void){
    gun_is_pulled_out = true;
  }
   
ICACHE_RAM_ATTR void pull_in_the_gun(void){
    gun_is_pulled_out = false;
  }

WiFiClient client;

void setup() {
  Wire.begin(4, 5);
  Serial.begin(115200);
  EEPROM.begin(12);
  
  pinMode(12, OUTPUT);//the solenoid
  pinMode(2, INPUT);//calib switch
  attachInterrupt(10, pull_out_the_gun, RISING);//here is hall sensor on D5
  attachInterrupt(10, pull_in_the_gun, FALLING);
  attachInterrupt(15, pull_the_trigger, RISING);//here is the button on D8
  
  i2cWrite(0x6B,0x00); // disable sleep mode  
  i2cWrite(0x1B,0x18);// change gyro range to +-2000dps
  i2cWrite(0x1C,0x00);// change acc range to +-2g
  
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
  Serial.println(WiFi.localIP());

  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);

  while (!client.connect(host, port)) {
    Serial.println("connection to host failed");
    delay(5000);
  }
  
  timer = micros();

  if(digitalRead(2)){//calibration(not done yet)
    delay(3000);
    int i=0;
    while(i < 20000){
        uint8_t* data = i2cRead(0x3B,14);
        sum_gyroX += ((data[8] << 8) | data[9]);
        sum_gyroY += ((data[10] << 8) | data[11]);
        sum_gyroZ += ((data[12] << 8) | data[13]);
      }
      EEPROM_float_write(GYRO_X_OFFSET_ADDR, sum_gyroX/20000.0);
      EEPROM_float_write(GYRO_Y_OFFSET_ADDR, sum_gyroY/20000.0);
      EEPROM_float_write(GYRO_Z_OFFSET_ADDR, sum_gyroZ/20000.0);
    }
}

void loop() {
  uint8_t* data = i2cRead(0x3B,14);

  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]);
  
  gyroX = ((data[8] << 8) | data[9]);
  gyroY = ((data[10] << 8) | data[11]);
  gyroZ = ((data[12] << 8) | data[13]);
  
  dt = (micros() - timer)/10000000.0;

  d_gyroXangle = (double)(((gyroX - gyroXoffset)/16.4)*dt)*RAD_TO_DEG;
  d_gyroYangle = (double)(((gyroY - gyroYoffset)/16.4)*dt)*RAD_TO_DEG;
  d_gyroZangle = (double)(((gyroZ - gyroZoffset)/16.4)*dt)*RAD_TO_DEG;

  timer = micros();

  accZangle = acos(accX/(sqrt(accY*accY+accX*accX)))*RAD_TO_DEG;

  gyroXangle = 0.8*gyroXangle + 0.2*(gyroXangle + ((d_gyroXangle > 0.1 || d_gyroXangle < -0.1) ? d_gyroXangle : 0));
  gyroYangle = 0.8*gyroYangle + 0.2*(gyroYangle + ((d_gyroYangle > 0.1 || d_gyroYangle < -0.1) ? d_gyroYangle : 0));
  gyroZangle = 0.8*gyroZangle + 0.2*(gyroZangle + ((d_gyroZangle > 0.1 || d_gyroZangle < -0.1) ? d_gyroZangle : 0));

  dtostrf(gyroXangle, 7, 2, g_x_str);
  dtostrf(gyroYangle, 7, 2, g_y_str);
  dtostrf(gyroZangle, 7, 2, g_z_str);
  dtostrf(accZangle, 7, 2, a_z_str);
  
  message += g_x_str;
  message += " ";
  message += g_y_str;
  message += " ";
  message += g_z_str;
  message += " ";
  message += a_z_str;
  
  if (client.connected() && trigger_is_pulled && gun_is_pulled_out && (millis() - cooldown_timer) > 1000) {
    client.println(message);//sending data
    digitalWrite(12, HIGH);//turning on the solenoid
    solenoid_timer = cooldown_timer = millis();
  }
  if(micros() - solenoid_timer > 40){
    digitalWrite(12, LOW);//turning off the solenoid
    }
}

void i2cWrite(uint8_t registerAddress, uint8_t data){
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
  for(uint8_t i = 0; i < nbytes; i++)
    data [i]= Wire.read();
  return data;
}

float EEPROM_float_read(int addr) {    
  byte raw[4];
  for(byte i = 0; i < 4; i++) raw[i] = EEPROM.read(addr+i);
  float &num = (float&)raw;
  EEPROM.end();
  return num;
}

void EEPROM_float_write(int addr, float num) {
  byte raw[4];
  (float&)raw = num;
  for(byte i = 0; i < 4; i++) EEPROM.write(addr+i, raw[i]);
  EEPROM.end();
}
