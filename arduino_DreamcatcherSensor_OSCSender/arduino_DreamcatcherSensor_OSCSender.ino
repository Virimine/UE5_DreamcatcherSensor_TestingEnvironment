#include <MPU9250_asukiaaa.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <MadgwickAHRS.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 19
#define SCL_PIN 18
#endif

//// Map movement - now it goes only right
//// Bound within screen
//// Callibrate Threshold
// Connect Locally or bring WIFI
// Fix X / Y axis fusion
// Find Cable

MPU9250_asukiaaa mySensor(MPU9250_ADDRESS_AD0_HIGH);
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

// WiFi Credentials
#define WIFI_NETWORK "moto g32" //Marouli, WiFimodem-CF2B
#define WIFI_PASSWORD "alex0000"    //freeWIFImates, vjyzygmzyj
#define WIFI_TIMEOUT_MS 20000

WiFiUDP Udp;                            // A UDP instance to let us send and receive packets over UDP
// remote IP of your computer. 
// CMD: Use upconfig/all to look for  Wireless LAN adapter Wi-Fi: IPv4 Address. (ex. 172, 20, 10, 2)
const IPAddress outIp(192, 168, 179, 73); //172.20.10.2
const unsigned int outPort = 4445;      // remote port to receive OSC
//const unsigned int localPort = 8001;    // local port to listen for OSC packets (actually not used for sending)

Madgwick filter;
float roll, pitch, yaw;
float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

#ifdef _ESP32_HAL_I2C_H_  // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  filter.begin(500);  // Sample rate in Hz
  connectToWiFi();  // Connect to WiFI first
}

void loop() {
  GetSensorData();
  UpdateGyro();

  Serial.println("X: " + String(yaw) + " Y: " + String(pitch) + " Z: " + String(roll));

  SendOSCBundle();
  delay(100);
}

void connectToWiFi() {
  Serial.print("Connecting to WiFI");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
    Serial.print(".");
    delay(100);
  }
  // TODO: Error Handling
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Failed!");
  } else {
    Serial.print("Connected!");
    Serial.print(WiFi.localIP());
  }

  Serial.println("Starting UDP");
}

void GetSensorData() {
  uint8_t sensorId;
  int result;

  result = mySensor.readId(&sensorId);

  result = mySensor.accelUpdate();
  if (result == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
  } else {
    Serial.println("Cannod read accel values " + String(result));
  }

  result = mySensor.gyroUpdate();
  if (result == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
  } else {
    Serial.println("Cannot read gyro values " + String(result));
  }
}

// void SendOSCMessage() {
//   OSCMessage msg("/gyroData/x");
//   msg.add(gX);

//   // Print the message to Serial
//   // SerialBuffer serialBuffer;
//   // Serial.print("OSC Message: ");
//   // serialBuffer.writeMessage(msg);

//   Udp.beginPacket(outIp, outPort);
//   msg.send(Udp);
//   Udp.endPacket();
//   msg.empty();
// }

void SendOSCBundle() {
  OSCBundle bndl;   //declare the bundle

  //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together
  bndl.add("/gyroData/x").add(yaw);
  bndl.add("/gyroData/y").add(roll);
  bndl.add("/gyroData/z").add(pitch);

  Udp.beginPacket(outIp, outPort);
  bndl.send(Udp);   // send the bytes to the SLIP stream
  Udp.endPacket();  // mark the end of the OSC Packet
  bndl.empty();     // empty the bundle to free room for a new one
}

void UpdateGyro() {

  filter.updateIMU(
    mySensor.gyroX() - gyroBiasX,
    mySensor.gyroY() - gyroBiasY,
    mySensor.gyroZ() - gyroBiasZ,
    aX, aY, aZ);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
}
