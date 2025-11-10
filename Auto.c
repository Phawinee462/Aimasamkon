#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>


//motor pin
#define MOTOR_R_1 6
#define MOTOR_R_2 7
#define MOTOR_R_EN 8
#define MOTOR_L_1 5
#define MOTOR_L_2 4
#define MOTOR_L_EN 3


//servos
Servo liftServo;
Servo gripperServo;
//ultrasonic
const int triggerPin = 11;
const int echoPin = 12;


//--------------------//Doomsday button//----------------------------//
struct DoomsdayOnly {
  bool ignoreFeedback;       //ignore sensors
  int turnTiming;            //soft turn timing
  int turnTiming2;           //hard turn timing
  int speedCompensation[2];  //motor speed compensation
};


DoomsdayOnly hardCode = { true, 250, 70, { 50, -50 } };
DoomsdayOnly* ptrHardCode = &hardCode;
//--------------------//---------------//----------------------------//


MPU6050 mpu(0x68);
#define INTERRUPT_PIN 2
/*IMPORTANT!!! REMEMBER TO WAIT TILL DMP READY IS TRUE BEFORE STARTING*/
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];


Quaternion q;
VectorFloat gravity;
float ypr[3];
float Yaw;


volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}


float getYaw() {
  if (!dmpReady) return;
  if (ptrHardCode->ignoreFeedback) return 0;
  //prevent wrong data, return previous data instead
  uint16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    return Yaw;
  }
  if (mpu.getFIFOCount() >= 1024) {
    mpu.resetFIFO();
    Serial.print("FIFO overflow (WARNING THE GYRO WILL HAVE ERROR FROM NOW), Returning : ");
    Serial.println(Yaw);
    return Yaw;
  }


  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return (ypr[0] * 180 / M_PI);
  }
}


//move function
void moveMotor(int motorR, int motorL) {
  if (motorL > 0) {
    digitalWrite(MOTOR_L_1, HIGH);
    digitalWrite(MOTOR_L_2, LOW);
  } else if (motorL < 0) {
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
  }
  analogWrite(MOTOR_L_EN, abs(motorL));


  if (motorR > 0) {
    digitalWrite(MOTOR_R_1, HIGH);
    digitalWrite(MOTOR_R_2, LOW);
  } else if (motorR < 0) {
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, HIGH);
  }
  analogWrite(MOTOR_R_EN, abs(motorR));
}
//break
void breakMotor() {
  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, HIGH);
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, HIGH);
  analogWrite(MOTOR_L_EN, 255);
  analogWrite(MOTOR_R_EN, 255);
  delay(200);
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, LOW);
}


//turn the car to angle


float acceptableError = 15;


void turnAngle(float target) {
  //if the angle is still off, attempt to adjust it back a total of 5 times
  Yaw = getYaw();
  while (abs(Yaw - target) > acceptableError) {
    if (Yaw > target) {
      moveMotor(-250, 250);
      Serial.print("Left ");
      Serial.println(Yaw);
    }
    if (Yaw < target) {
      moveMotor(250, -250);
      Serial.print("Right ");
      Serial.println(Yaw);
    }
    Yaw = getYaw();
  }


  breakMotor();
  Serial.print("Stop");
}


//move with accuracy for a time
void moveForTime(int direction, float duration) {
  float target = getYaw();
  unsigned long initialTime = millis();
  while (millis() - initialTime <= duration * 1000) {
    Yaw = getYaw();
    if (direction == 1) {
      moveMotor(200 + ptrHardCode->speedCompensation[0], 200 + ptrHardCode->speedCompensation[1]);
    }
    if (direction == -1) {
      moveMotor(-1 * (200 + ptrHardCode->speedCompensation[0]), -1 * (200 + ptrHardCode->speedCompensation[1]));
    }
  }
  breakMotor();
}
////move with accuracy til a distance
void moveTilDistance(int direction, int distance) {
  float target = getYaw();
  if (direction == 1) {
    while (readUltsonic() > distance) {
      Yaw = getYaw();
      moveMotor(200 - ((Yaw - target) / 0.5), 200 + ((Yaw - target) / 0.5));
    }
  }
  if (direction == -1) {
    while (readUltsonic() < distance) {
      Yaw = getYaw();
      moveMotor(-1 * (200 - ((Yaw - target) / 0.5)), -1 * (200 + ((Yaw - target) / 0.5)));
    }
  }
  breakMotor();
}


//servos
void servosObject(int commands) {
  if (commands == 0) {
    //grab
    liftServo.write(0);
    gripperServo.write(180);
    delay(500);
    gripperServo.write(10);
    delay(500);
    liftServo.write(135);
    delay(100);
  }
  if (commands == 1) {
    //release
    liftServo.write(0);
    delay(500);
    gripperServo.write(180);
    delay(500);
    liftServo.write(135);
    gripperServo.write(10);
    delay(100);
  }
}


void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  //gyro pin mode
  pinMode(INTERRUPT_PIN, INPUT);
  //motor pin mode
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(MOTOR_R_EN, OUTPUT);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_L_EN, OUTPUT);
  //servos pin
  liftServo.attach(9);
  gripperServo.attach(10);
  //ultrasonic pinmode
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);


  Serial.println("Testing connection...");
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");


  devStatus = mpu.dmpInitialize();
  Serial.print("DMP init result: ");
  Serial.println(devStatus);


  // Offsets (adjust if needed)
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);


  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();


    mpu.setDMPEnabled(true);


    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();


    Serial.println("DMP ready, waiting for data...");
  } else {
    Serial.println("DMP Initialization failed.");
  }
}


float duration, distance;


int readUltsonic() {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);


  duration = pulseIn(echoPin, HIGH);
  distance = (duration * .0343) / 2;
  if (distance == 0) return (999);
  return (distance);
}


void loop() {
  if (!dmpReady && !ptrHardCode->ignoreFeedback) return;
  liftServo.write(135);
  //the format is : moveForTime(direction(1 forward, -1 backwards), durationSeconds);
  //                turnAngle(angleBasedOnTheInitialPosition);
  //                moveTilDistance(Direction(1 forawrd, -1 backward), Distance to wall);
  //                servosObject(0 for grab, 1 for release)
  //Serial.println(readUltsonic());
  moveForTime(1, 1);
  delay(100);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //first turn
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 1.25);
  delay(100);
  moveMotor(250, -250);
  delay(ptrHardCode->turnTiming);
  breakMotor();
  //grab block
  delay(100);
  moveForTime(1, 1);
  delay(100);
  gripperServo.write(180);
  liftServo.write(0);
  delay(500);
  moveForTime(-1, 1);
  delay(100);
  gripperServo.write(0);
  delay(1000);
  liftServo.write(135);
  delay(1000);
  moveForTime(1, 1);
  delay(100);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming);
  breakMotor();
  //junction
  delay(100);
  moveForTime(1, 1);
  delay(100);
  moveForTime(-1, 0.05);
  moveMotor(250, -250);
  delay(ptrHardCode->turnTiming);
  breakMotor();
  //
  moveForTime(-1, 1);
  delay(100);
  moveForTime(1, 2);
  delay(100);
  moveForTime(-1, 0.05);
  delay(100);
  moveMotor(250, -250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 3);
  delay(100);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 1);
  delay(100);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 0.6);
  delay(100);
  moveMotor(250, -250);
  delay(ptrHardCode->turnTiming);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 1);
  delay(100);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 2);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 1);
  moveMotor(250, -250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 1);
  moveForTime(-1, 0.05);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming);
  breakMotor();
  //release
  moveForTime(1, 1);
  delay(100);
  liftServo.write(0);
  delay(100);
  gripperServo.write(180);
  delay(100);
  moveForTime(-1, 1);
  delay(100);
  moveForTime(1, 1);
  gripperServo.write(0);
  liftServo.write(135);
  delay(100);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //return
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 1);
  delay(200);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 1);
  delay(100);
  moveForTime(-1, 0.05);
  delay(100);
  moveMotor(250, -250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 2);
  delay(100);
  moveForTime(-1, 0.05);
  delay(100);
  moveMotor(250, -250);
  delay(ptrHardCode->turnTiming + ptrHardCode->turnTiming2);
  breakMotor();
  //
  moveForTime(-1, 0.5);
  delay(100);
  moveForTime(1, 1);
  delay(100);
  moveForTime(-1, 0.05);
  delay(100);
  moveMotor(-250, 250);
  delay(ptrHardCode->turnTiming);
  breakMotor();
  //
  moveForTime(-1, 1);


  //moveMotor(250,250);*/
  while (true) {}  //Halt program*/
}
