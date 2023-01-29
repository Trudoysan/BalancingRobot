#include "I2Cdev.h"
#include <Wire.h>
#include <PID_v1.h>
//#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include <NewPing.h>
#include <VL53L1X.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int servoAngle = 90;    // variable to read the value from the analog pin

VL53L1X sensor;
int distance = 0, angleD = 90;
unsigned long dTimer;
int mereni_pohyb = 1;

#define d_speed 1.5
#define d_dir 3

#define IN1 11
#define IN2 10
#define IN3 5
#define IN4 6
#define SERVO_PIN 9

//#define TRIGGER_PIN 9
//#define ECHO_PIN 8
#define MAX_DISTANCE 75
#define MIN_DISTANCE 20
#define BAL_DISTANCE 40

//char content = 'P';
int MotorAspeed, MotorBspeed;
float MOTORSLACK_A = 20;                   // 40Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B = 20;
#define BALANCE_PID_MIN -255              // Define PID limits to match PWM max in reverse and foward
#define BALANCE_PID_MAX 255
MPU6050 mpu;
//const int rxpin = 3;       //Bluetooth serial stuff
//const int txpin = 4;
//SoftwareSerial blue(rxpin, txpin);
//Ultrasonic ultrasonic(A0, A1);

//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
//int distanceCm;
//unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
//unsigned long pingTimer;     // Holds the next ping time.

//int distance
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer</p><p>// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector</p><p>/*********Tune these 4 values for your BOT*********/
double setpoint; //set the value when the bot is perpendicular to ground using serial monitor.
double originalSetpoint;
//Read the project documentation on circuitdigest.com to learn how to set these values
#define Kp  11 //Set this first
#define Kd  0.6//0.6 //Set this secound
#define Ki  160//160 //Finally set this
#define RKp  50 //Set this first
#define RKd 4//Set this secound
#define RKi  300 //Finally set this
/******End of values setting*********/
double ysetpoint;
double yoriginalSetpoint;
double input, yinput, youtput, output, Buffer[3];
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
PID rot(&yinput, &youtput, &ysetpoint, RKp, RKi, RKd, DIRECT);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

double compensate_slack(double yOutput, double Output, bool A);
void motorspeed(int MotorAspeed, int MotorBspeed);


void dmpDataReady()
{
  mpuInterrupt = true;
}


void init_imu() {
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  Wire.begin();
  TWBR = 24;
  mpu.initialize();  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));// load and configure the DMP
  devStatus = mpu.dmpInitialize(); // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-133);
  mpu.setYGyroOffset(54);
  mpu.setZGyroOffset(38);
  mpu.setZAccelOffset(2109); // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);  // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();  // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true; // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();//setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
    rot.SetMode(AUTOMATIC);
    rot.SetSampleTime(10);
    rot.SetOutputLimits(-20, 20);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
void getvalues() {
  // if programming failed, don't try to do anything</p><p>  if (!dmpReady) return;</p><p>  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    new_pid();
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();  // get current FIFO count
  fifoCount = mpu.getFIFOCount();  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();  // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);  // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
    mpu.dmpGetGravity(&gravity, &q); //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
    input = ypr[1] * 180 / M_PI + 180;
    yinput = ypr[0] * 180 / M_PI;
  }
}

void new_pid() {
  //Compute error
  pid.Compute();
  //  rot.Compute();
  // Convert PID output to motor control
  //MotorAspeed = compensate_slack(youtput, output, 1);
  //MotorBspeed = compensate_slack(youtput, output, 0);
  if (input > 100 && input < 260) {
    MotorAspeed = compensate_slack(0, output, 1);
    MotorBspeed = compensate_slack(0, output, 0);
  } else {
    MotorAspeed = 0;
    MotorBspeed = 0;
  }
  motorspeed(MotorAspeed, MotorBspeed);            //change speed
}
//Fast digitalWrite is implemented
/*void Bt_control() {
  if (blue.available()) {
    content = blue.read();
    if (content == 'F')
      setpoint = originalSetpoint - d_speed;//Serial.println(setpoint);}            //forward
    else if (content == 'B')
      setpoint = originalSetpoint + d_speed;//Serial.println(setpoint);}            //backward
    else if (content == 'L')
      ysetpoint = constrain((ysetpoint + yoriginalSetpoint - d_dir), -180, 180); //Serial.println(ysetpoint);}      //left
    else if (content == 'R')
      ysetpoint = constrain(ysetpoint + yoriginalSetpoint + d_dir, -180, 180); //Serial.println(ysetpoint);}        //right
    else if (content == 'S') {
      setpoint = originalSetpoint;
    }
  }
  else content = 'P';
  }*/
void initmot() {
  //Initialise the Motor outpu pins
  pinMode (IN4, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN1, OUTPUT);  //By default turn off both the motor
  analogWrite(IN4, LOW);
  analogWrite(IN3, LOW);
  analogWrite(IN2, LOW);
  analogWrite(IN1, LOW);
}
double compensate_slack(double yOutput, double Output, bool A) {
  // Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
  //yOutput is for left,right control
  if (A)
  {
    if (Output >= 0)
      Output = Output + MOTORSLACK_A - yOutput;
    if (Output < 0)
      Output = Output - MOTORSLACK_A - yOutput;
  }
  else
  {
    if (Output >= 0)
      Output = Output + MOTORSLACK_B + yOutput;
    if (Output < 0)
      Output = Output - MOTORSLACK_B + yOutput;
  }
  Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX);
  return Output;
}
void motorspeed(int MotorAspeed, int MotorBspeed) {
  // Motor A control
  if (MotorAspeed >= 0) {
    analogWrite(IN1, abs(MotorAspeed));
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, abs(MotorAspeed));
  }
  // Motor B control
  if (MotorBspeed >= 0) {
    analogWrite(IN3, abs(MotorBspeed));
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, abs(MotorBspeed));
  }
}
void printval()
{
  Serial.print(yinput); Serial.print("\t");
  Serial.print(yoriginalSetpoint); Serial.print("\t");
  Serial.print(ysetpoint); Serial.print("\t");
  Serial.print(youtput); Serial.print("\t"); Serial.print("\t");
  Serial.print(input); Serial.print("\t");
  Serial.print(originalSetpoint); Serial.print("\t");
  Serial.print(setpoint); Serial.print("\t");
  Serial.print(output); Serial.print("\t"); Serial.print("\t");
  Serial.print(MotorAspeed); Serial.print("\t");
  Serial.print(MotorBspeed); Serial.print("\t");
  //Serial.print(content);
  Serial.println("\t");
}

/*void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    // Here's where you can add code.
    Serial.print("Ping: ");
    Serial.print(sonar.ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
    Serial.println("cm");
  }
  // Don't do anything here!
  }*/
int getDistance() {
  static int dist70 = 5000, dist90 = 5000, dist110 = 5000;

  if (mereni_pohyb) {
    int distanceNow = sensor.read();
    if (servoAngle == 70) {
      dist70 = distanceNow;
    } else if (servoAngle == 90) {
      dist90 = distanceNow;

      if (dist90 >= MIN_DISTANCE) {
        if (dist90 >= MAX_DISTANCE) { //  za MAX v 90
          if (dist70 < MAX_DISTANCE ) {
            distance = dist70;
            angleD = 70;
          } else if (dist110 < MAX_DISTANCE) {
            distance = dist110;
            angleD = 110;
          } else {
            distance = dist90;
            angleD = 90;
          }
        } else { // mezi  MIN a MAX

          int d97 = dist90 - dist70;
          int d911 = dist90 - dist110;
          if (d97 > 0 || d911 > 0 ) {
            if (d97 > d911) {
              distance = dist70;
              angleD = 70;
            } else  {
              distance = dist110;
              angleD = 110;
            }
          }
        }

      } else {                     //back
        distance = dist90;
        angleD = 90;
      }


    } else if (servoAngle == 110) {
      dist110 = distanceNow;
    }
    mereni_pohyb = 0;
    return 50;
  }
  if (servoAngle == 70) {
    servoAngle = 90;
  } else if (servoAngle == 90) {
    servoAngle = 110;
  } else if (servoAngle == 110) {
    servoAngle = 70;
  }
  myservo.write(servoAngle);
  mereni_pohyb = 1;
  return 100;
}


void getMove() {

  setpoint = originalSetpoint;
  if (distance < MIN_DISTANCE) {
    setpoint = originalSetpoint + d_speed;//Serial.println(setpoint);}            //backward
  } else if (distance > BAL_DISTANCE && distance < MAX_DISTANCE) {
    setpoint = originalSetpoint - d_speed;//Serial.println(setpoint);}            //forward
  }
  if (angleD <= 70) {
    ysetpoint = constrain((ysetpoint + yoriginalSetpoint - d_dir), -180, 180); //Serial.println(ysetpoint);}      //left
  } else if (angleD >= 110) {
    ysetpoint = constrain(ysetpoint + yoriginalSetpoint + d_dir, -180, 180); //Serial.println(ysetpoint);}        //right
  }
}

void setup() {
  Serial.begin(115200);
  //pingTimer = millis(); // Start now.
  //  blue.begin(9600);
  //  blue.setTimeout(10);
  init_imu();           //initialiser le MPU6050
  initmot();            //initialiser les moteurs
  originalSetpoint = 176;  //consigne
  yoriginalSetpoint = 0.1;
  setpoint = originalSetpoint ;
  ysetpoint = yoriginalSetpoint ;

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Medium);
  sensor.setMeasurementTimingBudget(33000);
  sensor.startContinuous(50);

  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  delay(500);
  myservo.write(servoAngle);
  mereni_pohyb = 1;
  dTimer = millis();
  delay(500);
}


void loop() {
  if (millis() > dTimer ) {
    dTimer = millis() + getDistance();
  }

  getMove();
  getvalues();
  // Bt_control();
  printval();
}
