#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <Ethernet.h>
#include <SPI.h>
#include <EthernetUdp.h>

Madgwick filter;
float microsPerReading, microsPrevious, lastMicros;
float accelScale, gyroScale;
int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
float microsNow;
boolean stop = false;
float originalHeading;
int stopAngle = 180;
float disposition = 0;
float lastT;
byte* headingSend;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 177);

unsigned int localPort = 8888;      // local port to listen on

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

void setup() {
  Serial.begin(9600);
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);

  filter.begin(25);

  //calibration
  CurieIMU.initialize();
  Serial.print("Starting Gyroscope calibration...");
  CurieIMU.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  Serial.println(" Done");
  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieIMU.setGyroOffsetEnabled(true);
  CurieIMU.setAccelOffsetEnabled(true);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // verify connection
  Serial.println("Testing device connections...");
  if (CurieIMU.testConnection()) {
    Serial.println("CurieIMU connection successful");
  } else {
    Serial.println("CurieIMU connection failed");
  }

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

  // check if it's time to read data and update the filter
  microsNow = micros();

  // read raw data from CurieIMU
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);


  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

  originalHeading = heading;

}

void loop() {
  // check if it's time to read data and update the filter
  microsNow = micros();

  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    
    //    Serial.print("Orientation: ");
    //    Serial.println(heading);
    //    Serial.print(" ");
    //    Serial.print(pitch);
    //    Serial.print(" ");
    //    Serial.println(roll);
    
    //convert float to byte[]
    byte* msg = (byte*) &heading;
    
    //send packet
    int packetSize = Udp.parsePacket();
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(msg, 4);
    Udp.endPacket();
    //    Serial.println( *((float*)msg));

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
    lastMicros = microsNow;
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

//void updateDistance(float distance, float acc, float lastTime, float currentTime)
//
//  float tmp = (currentTime - lastTime) * (currentTime - lastTime) * 0.000000000001;
//  disposition = (acc * 9.8 / 2.0) * tmp + distance;
//
//}

