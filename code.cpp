#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L0X.h"

/************ Pin Definitions ************/
// Right Motor
#define RIGHT_MOTOR_PWM  17   // PWM pin for the right motor
#define RIGHT_MOTOR_IN1  15   // IN1 for the right motor
#define RIGHT_MOTOR_IN2  16   // IN2 for the right motor

// Left Motor
#define LEFT_MOTOR_PWM   23   // PWM pin for the left motor
#define LEFT_MOTOR_IN1   18   // IN3-like pin for the left motor
#define LEFT_MOTOR_IN2   19   // IN4-like pin for the left motor

// Encoders
#define RIGHT_ENCODER_A  26  
#define RIGHT_ENCODER_B  27  
#define LEFT_ENCODER_A   32  
#define LEFT_ENCODER_B   33  

// VL53L0X (Time-of-Flight) Lidar XSHUT Pins
#define SHT_LOX1  13   // Right sensor XSHUT
#define SHT_LOX2  14   // Front sensor XSHUT (you called it "Left", but physically it's front)

// Unique I2C addresses for each sensor
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

/************ Global Variables ************/
// Encoder counts
volatile long right_encoder_count = 0;
volatile long left_encoder_count  = 0;

// Approx pulses per revolution
const int RIGHT_PPR = 155; 
const int LEFT_PPR  = 153;

/** Robot Speed Settings **/
int base_speed  = 60;    
const int MIN_SPEED = 50;

/** PID constants for forward motion (heading-based) **/
const float Kp = 1.0;
const float Ki = 0.005;
const float Kd = 0.25;

/** PID constants for turning **/
const float TURN_Kp = 0.5;  
const float TURN_Ki = 0.0;   
const float TURN_Kd = 0.3;  

Adafruit_MPU6050 mpu;
float gyro_z_offset = 0.0; 

// Heading PID accumulators
float previous_error = 0;
float integral       = 0;

/************ Wheel / Distance Info ************/
const float WHEEL_RADIUS = 2.0f;            // cm
const float WHEEL_CIRCUM = 2.0f * PI * WHEEL_RADIUS;  // ~12.56 cm
const float MOVE_DISTANCE = 18.9f;          // cm per cell or segment

// If drifting to the RIGHT, you might use a negative offset
float driftOffset = 2.6;

/************ VL53L0X Objects ************/
// physically: loxRight is your right sensor, loxLeft is actually your front sensor
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X(); 
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X(); 

VL53L0X_RangingMeasurementData_t measureRight;
VL53L0X_RangingMeasurementData_t measureFront;

/************ Front-Wall Threshold ************/
const int FRONT_WALL_THRESHOLD = 75; // mm

/************ Desired Right-Wall Distance (for alignment) ************/
// If you want to “center” by hugging the right side at ~100 mm from the wall
const float DESIRED_RIGHT_DIST = 40.0;
const float ALIGN_TOLERANCE = 17.0; // acceptable +/- range

/************ Function Prototypes ************/
void setID();
void readSensors();
void moveForwardDistance(float distance_cm);
void moveForward(int pulses);

// Turn routines
void turnRight90();
void turnLeft90();

// Utility
void stopMotors();
float calibrateGyroZ(int numSamples);
void initMPUOnce(); 
// We won't re-init the MPU each time if we want the same reference

// Right-hand rule or your existing logic
void rightHandLogic();

// Align / “Center” to the right wall after a turn
void alignToRightWall();

// Interrupts
void IRAM_ATTR rightEncoderISR();
void IRAM_ATTR leftEncoderISR();


//===================================================================================
//                              SETUP
//===================================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // ========== Setup Lidar Sensors ==========
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  // Both low => reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  setID(); // assigns addresses to each sensor (LOX1_ADDRESS, LOX2_ADDRESS)

  // ========== Setup MPU6050 (only once) ==========
  initMPUOnce();

  // ========== Setup Motors + Encoders ==========
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1,  OUTPUT);
  pinMode(RIGHT_MOTOR_IN2,  OUTPUT);

  pinMode(LEFT_MOTOR_PWM,  OUTPUT);
  pinMode(LEFT_MOTOR_IN1,  OUTPUT);
  pinMode(LEFT_MOTOR_IN2,  OUTPUT);

  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_A,  INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A),  leftEncoderISR,  RISING);

  Serial.println("Setup complete. Using right-hand rule in loop...");
}

//===================================================================================
//                              MAIN LOOP
//===================================================================================
void loop() {
  readSensors(); // read LIDAR continuously
  rightHandLogic();
  delay(100);
}

//===================================================================================
//                           RIGHT-HAND RULE LOGIC
//===================================================================================
void rightHandLogic() {
  float distRight, distFront;

  if (measureRight.RangeStatus != 4) { 
    distRight = measureRight.RangeMilliMeter; 
  } else {
    distRight = 9999.0; 
  }

  if (measureFront.RangeStatus != 4) {
    distFront = measureFront.RangeMilliMeter;
  } else {
    distFront = 9999.0;
  }

  Serial.print("distRight= ");
  Serial.print(distRight);
  Serial.print(" mm, distFront= ");
  Serial.print(distFront);
  Serial.println(" mm.");

  // If there's a big opening on the right, turn right.
  if (distRight > 200.0) { 
    Serial.println("Opening to the RIGHT -> turning right");
    turnRight90();
    moveForwardDistance(MOVE_DISTANCE);
  }
   else if (65.0 < distRight < 150) { // Adjust threshold as needed
      Serial.println("Near wall detected -> aligning to right wall");
      alignToRightWall();
    }
  // If blocked in front, turn left.
  else if (distFront < 80.0 ) {
    Serial.println("Blocked in front -> turning left");
    turnLeft90();
  }
    else {
      // Otherwise, just move forward as per usual.
      Serial.println("No near wall to align to -> moving forward...");
      moveForwardDistance(MOVE_DISTANCE);
    }
  
}


//===================================================================================
//                          ALIGN TO RIGHT WALL
//===================================================================================
// Because you only have a right sensor and a front sensor,
// we can't truly center in a corridor. We'll just align to
// the right wall at ~100 mm if possible.
void alignToRightWall() {
  Serial.println("Aligning to right wall...");
  
  // We'll do a small loop trying to get measureRight ~ DESIRED_RIGHT_DIST
  // If measureRight is far from the setpoint, we nudge left or right.
  // Because we only have the right sensor, we'll do a simple approach.
  
  for (int i=0; i<50; i++) { // up to 50 attempts
    readSensors();
    float distRight = (measureRight.RangeStatus != 4) 
                       ? measureRight.RangeMilliMeter 
                       : 9999.0;

    float error = DESIRED_RIGHT_DIST - distRight; // + if we're too far from wall

    if (fabs(error) < ALIGN_TOLERANCE) {
      // Good enough
      stopMotors();
      Serial.print("Right wall alignment done. distRight= ");
      Serial.println(distRight);
      return;
    }

    // If error>0 too far activate right
    // If error<0 too far activate left
    if (error > 0) {
      // Nudge right: left motor slower, right motor faster
       digitalWrite(RIGHT_MOTOR_IN1, HIGH);
       digitalWrite(RIGHT_MOTOR_IN2, LOW);
       digitalWrite(LEFT_MOTOR_IN1,  HIGH);
        digitalWrite(LEFT_MOTOR_IN2,  LOW);
      analogWrite(LEFT_MOTOR_PWM,  30);
      analogWrite(RIGHT_MOTOR_PWM, 55);
    } else if (error < 0) { //far to get close activate left motor
      digitalWrite(RIGHT_MOTOR_IN1, HIGH);
       digitalWrite(RIGHT_MOTOR_IN2, LOW);
       digitalWrite(LEFT_MOTOR_IN1,  HIGH);
        digitalWrite(LEFT_MOTOR_IN2,  LOW);
      analogWrite(LEFT_MOTOR_PWM,  55);
      analogWrite(RIGHT_MOTOR_PWM, 30 );
    }
    delay(50);
  }
  stopMotors();
  Serial.println("AlignToRightWall: gave up after 50 iterations (couldn't align?).");
}

//===================================================================================
//                         MOVE FORWARD DISTANCE
//===================================================================================
void moveForwardDistance(float distance_cm) {
  float revsRight = distance_cm / WHEEL_CIRCUM;
  int pulsesRight = (int)(revsRight * RIGHT_PPR + 0.5);

  float revsLeft  = distance_cm / WHEEL_CIRCUM;
  int pulsesLeft  = (int)(revsLeft  * LEFT_PPR  + 0.5);

  int pulsesNeeded = (pulsesRight + pulsesLeft) / 2;

  Serial.print("moveForwardDistance: ");
  Serial.print(distance_cm);
  Serial.print(" => pulses ~ ");
  Serial.println(pulsesNeeded);

  moveForward(pulsesNeeded);
}

//===================================================================================
//                      MOVE FORWARD (ENCODER + HEADING PID)
//===================================================================================
void moveForward(int pulses) {
  Serial.print("moveForward -> ");
  Serial.print(pulses);
  Serial.println(" pulses.");

  int originalBaseSpeed = base_speed;

  right_encoder_count = 0;
  left_encoder_count  = 0;
  integral       = 0;
  previous_error = 0;

  // Set motor direction forward
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  digitalWrite(LEFT_MOTOR_IN1,  HIGH);
  digitalWrite(LEFT_MOTOR_IN2,  LOW);

  float currentHeading = 0.0; 
  // If you want to store an absolute heading across the entire run,
  // consider making currentHeading a global variable and not resetting
  // it each time. But we'll keep it local for a "relative" heading.

  unsigned long lastTime = millis();

  // Stall detection
  unsigned long lastEncoderUpdate = millis();
  long prevRightEnc = 0;
  long prevLeftEnc  = 0;
  const unsigned long STALL_TIMEOUT = 4000; 
  const int SPEED_BOOST = 10;
  const int MAX_SPEED   = 120;

  while ((right_encoder_count < pulses) || (left_encoder_count < pulses)) {
    // 1) Always read sensors
    readSensors();

    // 2) Check front sensor
    float distFront = 9999.0;
    if (measureFront.RangeStatus != 4) {
      distFront = measureFront.RangeMilliMeter; 
    }
    if (distFront < FRONT_WALL_THRESHOLD) {
      Serial.println("Wall detected in front -> STOP!");
      stopMotors();
      base_speed = originalBaseSpeed;
      return; 
    }

    // 3) Stall detection
    unsigned long now = millis();
    if ((right_encoder_count != prevRightEnc) || (left_encoder_count != prevLeftEnc)) {
      prevRightEnc = right_encoder_count;
      prevLeftEnc  = left_encoder_count;
      lastEncoderUpdate = now;
    } else {
      if ((now - lastEncoderUpdate) > STALL_TIMEOUT) {
        base_speed += SPEED_BOOST;
        if (base_speed > MAX_SPEED) base_speed = MAX_SPEED;
        Serial.print("No encoder movement >4s => increasing base_speed to ");
        Serial.println(base_speed);
        lastEncoderUpdate = now;
      }
    }

    // 4) Read gyro for heading
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 5) dt for heading PID
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // 6) Integrate heading (relative)
    float z_deg_per_sec = (g.gyro.z - gyro_z_offset) * (180.0 / PI);
    currentHeading += z_deg_per_sec * dt;
    float headingError = currentHeading;

    // 7) PID for heading
    integral += headingError * dt;
    float derivative = (headingError - previous_error) / dt;
    float correction = (Kp * headingError) + (Ki * integral) + (Kd * derivative);
    previous_error = headingError;

    // 8) driftOffset
    float rightSpeed = (base_speed - correction) + driftOffset;
    float leftSpeed  = (base_speed + correction) - driftOffset;

    // 9) Constrain
    rightSpeed = constrain(rightSpeed, MIN_SPEED, base_speed + abs(driftOffset));
    leftSpeed  = constrain(leftSpeed,  MIN_SPEED, base_speed + abs(driftOffset));

    analogWrite(RIGHT_MOTOR_PWM, (int)rightSpeed);
    analogWrite(LEFT_MOTOR_PWM,  (int)leftSpeed);

    // Debug
    Serial.print("HeadingErr=");
    Serial.print(headingError);
    Serial.print(" | R=");
    Serial.print(rightSpeed);
    Serial.print(" | L=");
    Serial.print(leftSpeed);
    Serial.print(" | EncR=");
    Serial.print(right_encoder_count);
    Serial.print(" | EncL=");
    Serial.println(left_encoder_count);

    delay(10);

    if (right_encoder_count >= pulses && left_encoder_count >= pulses) {
      break;
    }
  }

  stopMotors();
  Serial.println("Forward Movement Complete!");

  base_speed = originalBaseSpeed;
  // Removed resetMPU() to preserve heading reference
}

//===================================================================================
//                       TURN RIGHT 90
//===================================================================================
void turnRight90() {
  Serial.println("Turning RIGHT 90...");

  // Right motor reversed
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(RIGHT_MOTOR_PWM, 0);

  // Left motor forward
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);

  integral = 0;
  previous_error = 0;

  float turned_angle = 0.0;
  float target_angle = 68;   // tweak if under/over-turn
  unsigned long lastTime = millis();

  while (true) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    float z_deg_per_sec = (g.gyro.z - gyro_z_offset) * (180.0 / PI);
    turned_angle += z_deg_per_sec * dt;

    float error = target_angle - fabs(turned_angle);

    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    float correction = (TURN_Kp * error) + (TURN_Ki * integral) + (TURN_Kd * derivative);

    float speed = base_speed - correction;
    speed = constrain(speed, MIN_SPEED, base_speed);

    if (fabs(error) < 2.0) {
      break;
    }

    analogWrite(RIGHT_MOTOR_PWM, (int)(speed + 4));
    analogWrite(LEFT_MOTOR_PWM,  (int)(speed + 5));

    previous_error = error;
    Serial.print("Angle=");
    Serial.print(turned_angle);
    Serial.print(" deg | Err=");
    Serial.print(error);
    Serial.print(" | L-speed=");
    Serial.println(speed);

    delay(10);
  }

  stopMotors();
  Serial.println("Right Turn Complete!");
  // Not resetting MPU => keeps heading reference
}

//===================================================================================
//                      TURN LEFT 90
//===================================================================================
void turnLeft90() {
  Serial.println("Turning LEFT 90...");

  // Left motor reversed
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_PWM, 0);

  // Right motor forward
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);

  integral = 0;
  previous_error = 0;

  float turned_angle = 0.0;
  float target_angle = 66; // tweak if needed
  unsigned long lastTime = millis();

  while (true) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    float z_deg_per_sec = (g.gyro.z - gyro_z_offset) * (180.0 / PI);
    turned_angle += z_deg_per_sec * dt;

    float error = target_angle - fabs(turned_angle);

    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    float correction = (TURN_Kp * error) + (TURN_Kd * derivative);

    previous_error = error;

    float speed = base_speed - correction;
    speed = constrain(speed, MIN_SPEED, base_speed);

    if (fabs(error) < 2.0) {
      break;
    }

    analogWrite(RIGHT_MOTOR_PWM, (int)(speed + 20));
    analogWrite(LEFT_MOTOR_PWM,  (int)(speed + 5));

    Serial.print("Angle=");
    Serial.print(turned_angle);
    Serial.print(" deg | Err=");
    Serial.print(error);
    Serial.print(" | R-speed=");
    Serial.println(speed);

    delay(10);
  }

  stopMotors();
  Serial.println("Left Turn Complete!");
  // Not resetting MPU => keeps heading reference
}

//===================================================================================
//                            STOP MOTORS
//===================================================================================
void stopMotors() {
  analogWrite(RIGHT_MOTOR_PWM, 0);
  analogWrite(LEFT_MOTOR_PWM,  0);
}

//===================================================================================
//                       MPU INIT (only once)
//===================================================================================
void initMPUOnce() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1);
  }
  delay(100);

  Serial.println("MPU6050 Initialized");
  gyro_z_offset = calibrateGyroZ(200);
  Serial.print("Gyro Z Offset: ");
  Serial.println(gyro_z_offset);
}

//===================================================================================
//                      ENCODER INTERRUPTS
//===================================================================================
void IRAM_ATTR rightEncoderISR() {
  right_encoder_count++;
}
void IRAM_ATTR leftEncoderISR() {
  left_encoder_count++;
}

//===================================================================================
//                         CALIBRATE GYRO Z
//===================================================================================
float calibrateGyroZ(int numSamples) {
  float sum = 0.0;
  for(int i=0; i<numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(10);
  }
  return sum / numSamples;
}

//===================================================================================
//                      SET ID FOR LIDAR
//===================================================================================
void setID() {
  // Right sensor = LOX1 => SHT_LOX1
  // Front sensor = LOX2 => SHT_LOX2
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  // wake both
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // 1) Right sensor init
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  if (!loxRight.begin(LOX1_ADDRESS)) {
    Serial.println("Failed to boot RIGHT VL53L0X!");
    while(1);
  }
  delay(10);

  // 2) Front sensor init
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!loxFront.begin(LOX2_ADDRESS)) {
    Serial.println("Failed to boot FRONT VL53L0X!");
    while(1);
  }
}

//===================================================================================
//                     READ LIDAR SENSORS
//===================================================================================
void readSensors() {
  loxRight.rangingTest(&measureRight, false); 
  loxFront.rangingTest(&measureFront,  false); 
}
