#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Dns.h>
//#include <CurieIMU.h>
#include <MadgwickAHRS.h>

#define NUM_OF_AXLES 2

int val = 0;

// chris's declarations
int count = 0;
int lastMod = 0;
const int numMagnet = 4;
const float wheelDiameter = 8.7;
//6.25 on the 2017 robot

#define INTERVAL 4
const int windowTime = 2000; //milliseconds
const int intervalTime = windowTime / INTERVAL; //milliseconds

struct Axle{
  String side;
  unsigned short pin;
  unsigned long distanceTraveled;
  float velocity; //inches per second
  int last;
  int pulses[INTERVAL];
} leftAxle, rightAxle;

Axle* axles[] = {&leftAxle , &rightAxle};

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

void setupAxles(){
  leftAxle.pin = 4;
  leftAxle.side = "Left";
  rightAxle.pin = 8;
  rightAxle.side = "Right";
  for(unsigned int i = 0; i < NUM_OF_AXLES; i++){
    pinMode(axles[i]->pin, INPUT_PULLUP);
    axles[i]->distanceTraveled = 0;
    axles[i]->velocity = 0;
    axles[i]->last = 0;
  }
}

void setup() {
  setupAxles();
  randomSeed(analogRead(0)); // can't be connected
  Serial.begin(9600);
  setupIMU();
}
int mod = 0;
unsigned long timeX = millis() / intervalTime;

void loopAxle(Axle* axle){
  // put your main code here, to run repeatedly:
  int x = 1 - digitalRead(axle->pin);
  if  (lastMod != mod) {
    int sum = 0;
    if (timeX >= INTERVAL) {
      for (int i = 0; i < INTERVAL; i++) {
        sum += axle->pulses[i];
      }
      axle->velocity = sum * wheelDiameter * PI / numMagnet / (windowTime / 1000.0);
    } else {
      //pulses array has not been completely filled in yet
      for (int i = 0; i < mod; i++) {
        sum += axle->pulses[i];
      }
      axle->velocity = sum * wheelDiameter * PI * INTERVAL / mod / numMagnet / (windowTime / 1000.0);
    }
    axle->distanceTraveled += (axle->velocity * intervalTime) / 1000.0;
    axle->pulses[mod] = 0;
  }
  Serial.println (axle->side + " " + pulses);
//    Serial.print(axle->)
  if (axle->last == 0 && x == 1){
    axle->pulses[mod]++;
  }
  axle->last = x;
}

unsigned long D;
unsigned long d;
const double width = 25;
double x = 0;
double y = 0;
  
String convertToXY(){
  if(rightAxle.distanceTraveled > leftAxle.distanceTraveled){
    Serial.println("turning right");
    D = rightAxle.distanceTraveled;
    d = leftAxle.distanceTraveled;
    unsigned long radius = d*width/(D-d);
    double angle = d/radius;
    x = (radius+width/2)*cos(angle);
    y = (radius+width/2)*sin(angle);
    return String(x) + "," + String(y);
  }
  else if(leftAxle.distanceTraveled > rightAxle.distanceTraveled){
    Serial.println("turning left");
    D = leftAxle.distanceTraveled;
    d = rightAxle.distanceTraveled;
    unsigned long radius = d*width/(D-d);
    double angle = d/radius;
    x = (radius+width/2)*cos(angle);
    y = (radius+width/2)*sin(angle);
    return String(x) + "," + String(y);
  }
  else{
    Serial.println("driving straight");
    y = leftAxle.distanceTraveled;
    return String(x) + "," + String(y);
  }

}

void loop() {
  Serial.print("right");
  Serial.println(rightAxle.distanceTraveled);
  Serial.print("left");       
  Serial.println(leftAxle.distanceTraveled);
  String send = "";
  loopIMU();
  timeX = millis() / intervalTime;
  mod = timeX % INTERVAL;
  for(unsigned int i = 0; i < NUM_OF_AXLES; i++){
    loopAxle(axles[i]);
  }
  lastMod = mod;
  send.concat(convertToXY());
  send.concat(heading);
  Serial.println(send);
}

void loopIMU() {
/*
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

    //    roll = filter.getRoll();
    //    pitch = filter.getPitch();
    heading = filter.getYaw();
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
    lastMicros = microsNow;
  }
  */
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
  
  /*
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
  */
}
