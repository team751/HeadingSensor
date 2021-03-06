#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Dns.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

#define PIN 4

int val = 0;

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress local(10, 7, 51, 123);
IPAddress ip(10, 7, 51, 109);

unsigned int localPort = 8888;      // local port to listen on

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
DNSClient dnsclient;

// chris's declarations
int last = 0;
int count = 0;
int lastMod = 0;
int sum = 0;
int numMagnet = 6;
float wheelDiameter = 6.25;
//8.7 on the 2016 robot
unsigned long distanceTraveled0 = 0;
unsigned long distanceTraveled1 = 0;

#define INTERVAL 8
const int windowTime = 1000; //milliseconds
const int intervalTime = windowTime / INTERVAL; //milliseconds
int pulses[INTERVAL];
float velocity0 = 0; //inches per second
float velocity1 = 0; //inches per second

// chris's decl
Madgwick filter;
float microsPerReading, microsPrevious, lastMicros; float accelScale, gyroScale;

int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
float microsNow;
float originalHeading;

void setup() {
  //  randomSeed(analogRead(0)); // can't be connected
  int [] pinNums = {4, 8}
                   pinMode(pinNums[0], INPUT_PULLUP);
  pinMode(pinNums[1], INPUT_PULLUP);
  Serial.begin(9600);

  velocity = 0;
  distanceTraveled = 0;
  setupIMU();
}

void loop() {
  // chris's stuff
  // put your main code here, to run repeatedly:
  int x = 1 - digitalRead(4);
  unsigned long timeX = millis() / intervalTime;
  int mod = timeX % INTERVAL;

  sendSensorData(pinNums[0]);
  sendSensorData(pinNums[1]);
}

void loopIMU() {
  microsNow = micros();
  //Heading
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

    //    roll = filter.geftRoll();
    //    pitch = filter.getPitch();
    heading = filter.getYaw();
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
    lastMicros = microsNow;
  }
}

string sendSensorData(int pinNum) {
  int x = 1 - digitalRead(pinNum);
  unsigned long timeX = millis() / intervalTime;
  int mod = timeX % INTERVAL;

  if (lastMod != mod) {
    sum = 0;
    if (timeX >= INTERVAL) {
      for (int i = 0; i < INTERVAL; i++) {
        sum += pulses[i];
      }
      if (pinNum == pinNums[0])
      { velocity0 = sum * wheelDiameter * PI / numMagnet / (windowTime / 1000.0);
      } else {
        velocity1 = sum * wheelDiameter * PI / numMagnet / (windowTime / 1000.0);
      } else {
        //pulses array has not been completely filled in yet
        for (int i = 0; i < mod; i++) {
          sum += pulses[i];
        }
        if (pinNum == pinNums[0])
        { velocity0 = sum * wheelDiameter * PI * INTERVAL / mod / numMagnet / (windowTime / 1000.0);
        } else {
          velocity1 = sum * wheelDiameter * PI * INTERVAL / mod / numMagnet / (windowTime / 1000.0);
        }
      }
      //    Serial.print("Velocity = ");
      //    Serial.println(velocity);
      if(pinNum == pinNums[0]){
      distanceTraveled0 += (velocity0 * intervalTime) / 1000.0;
      }else{
        distanceTraveled1 += (velocity1 * intervalTime) / 1000.0;
      }
      //    Serial.print("distanceTraveled: ");
      //    Serial.println(distanceTraveled);
      pulses[mod] = 0;
    }


    if (last == 0 && x == 1) pulses[mod]++;

    //  for (int i = 0; i < INTERVAL; i++) {
    //      Serial.print(pulses[i]);
    //      Serial.print(" ");
    //  }
    //    Serial.println();

    lastMod = mod;
    last = x;

    loopIMU();

    if(pinNum == pinNums[0]){
    String sendString = "[" + "left" + "," + String(heading) + "," + String(velocity) + "," + String(distanceTraveled) + "]";
    }else{
    String sendString = "[" + "right" + "," + String(heading) + "," + String(velocity1) + "," + String(distanceTraveled1) + "]";
    }
    Serial.println(sendString);
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
  // since we are using 250 degrees/intervalTime range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void setupIMU() {
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  //calibration
  CurieIMU.initialize();
  //Serial.print("Starting Gyroscope calibration...");
  CurieIMU.autoCalibrateGyroOffset();
  //Serial.println(" Done");
  //Serial.print("Starting Acceleration calibration...");
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  //Serial.println(" Done");
  //Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieIMU.setGyroOffsetEnabled(true);
  CurieIMU.setAccelOffsetEnabled(true);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // verify connection
  //Serial.println("Testing device connections...");
  if (CurieIMU.testConnection()) {
    //Serial.println("CurieIMU connection successful");
  } else {
    //Serial.println("CurieIMU connection failed");
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
  //roll = filter.getRoll();
  //pitch = filter.getPitch();
  heading = filter.getYaw();

  originalHeading = heading;
}

/*
 * if (lastMod != mod) {
    sum = 0;
    if (timeX >= INTERVAL) {
      for (int i = 0; i < INTERVAL; i++) {
        sum += pulses[i];
      }
      velocity = sum * wheelDiameter * PI / numMagnet / (windowTime / 1000.0);
    } else {
      //pulses array has not been completely filled in yet
      for (int i = 0; i < mod; i++) {
        sum += pulses[i];
      }
      velocity = sum * wheelDiameter * PI * INTERVAL / mod / numMagnet / (windowTime / 1000.0);
    }
    //    Serial.print("Velocity = ");
    //    Serial.println(velocity);

    distanceTraveled += (velocity * intervalTime) / 1000.0;
    //    Serial.print("distanceTraveled: ");
    //    Serial.println(distanceTraveled);
    pulses[mod] = 0;
  }


  if (last == 0 && x == 1) pulses[mod]++;

  //  for (int i = 0; i < INTERVAL; i++) {
  //      Serial.print(pulses[i]);
  //      Serial.print(" ");
  //  }
  //    Serial.println();

  lastMod = mod;
  last = x;

  loopIMU();

  String sendString = "[" + String(heading) + "," + String(velocity) + "," + String(distanceTraveled) + "]";
  Serial.println(sendString);
 * /
 */
