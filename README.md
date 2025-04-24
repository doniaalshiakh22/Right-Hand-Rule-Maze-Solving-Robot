# Maze-Solving Robot with Right-Hand Rule

## Project Overview  
This project showcases the development of an autonomous maze-solving robot that uses the right-hand rule, a well-known technique for navigating mazes. The robot follows the rule by keeping its right side always close to a wall. Equipped with various sensors like motors, encoders, an MPU6050 IMU (Inertial Measurement Unit), and VL53L0X time-of-flight (ToF) sensors, it is capable of precise navigation and obstacle avoidance.

---

## Hardware Components

- **Microcontroller**: ESP32
- **Motor Drivers**: H-Bridge to control the left and right motors
- **Encoders**: Track wheel rotations to estimate traveled distance
- **IMU Sensor (MPU6050)**: Provides gyroscope data to assist with heading corrections
- **Time-of-Flight Sensors (VL53L0X)**: Measures the distance to walls and obstacles
- **Power Supply**: Battery-powered for independent operation

---

## I2C Communication Protocol  
The robot uses the I2C (Inter-Integrated Circuit) communication protocol to interface with sensors like the MPU6050 IMU and VL53L0X Lidar sensors. I2C utilizes two main lines:

- **SDA (Serial Data Line)**: Transfers data between devices
- **SCL (Serial Clock Line)**: Synchronizes the communication between devices

Each device connected to the I2C bus has a unique address, allowing multiple devices to communicate over the same two lines. To avoid conflicts, the VL53L0X sensors use custom I2C addresses, and the MPU6050 sensor transmits accelerometer and gyroscope data over I2C.

---

## Right-Hand Rule Navigation Logic  
The robot follows the right-hand rule to navigate through its environment. The logic works as follows:

1. **Check Right Wall**: If the right side is clear (open space), the robot will turn right and proceed forward.
2. **Check Front Wall**: If there is an obstacle ahead, the robot turns left.
3. **Align with Right Wall**: If the robot detects a nearby wall, it adjusts its position to maintain a consistent distance from the wall.
4. **Move Forward**: If there are no obstructions, the robot continues straight.

The robot checks its path every 20 cm to decide whether it should turn, align with the wall, or move forward.

### Navigation Algorithm (Code Example)

void rightHandLogic() {
    float distRight, distFront;
    readSensors();
    distRight = measureRight.RangeMilliMeter;
    distFront = measureFront.RangeMilliMeter;

    if (distRight > 200.0) {
        Serial.println("Opening to the RIGHT -> turning right");
        turnRight90();
        moveForwardDistance(MOVE_DISTANCE);
    } else if (65.0 < distRight < 150) {
        Serial.println("Aligning to right wall");
        alignToRightWall();
    } else if (distFront < 80.0) {
        Serial.println("Blocked in front -> turning left");
        turnLeft90();
    } else {
        Serial.println("Moving forward...");
        moveForwardDistance(MOVE_DISTANCE);
    }
}
```

---

## PID-Based Motion Control  
The robot utilizes **PID control** (Proportional-Integral-Derivative) for precise movement, particularly for:

- **Straight-line motion**: Adjustments are made based on IMU heading data.
- **Turn precision**: Corrections are integrated through gyroscope readings.

### What is PID Control?  
PID control is a feedback mechanism designed to improve the stability and accuracy of a system. The formula is as follows:

```
u(t) = Kp * e(t) + Ki * ∫e(t) dt + Kd * de(t) / dt
```

Where:
- **Kp (Proportional)**: Corrects the error based on its current value.
- **Ki (Integral)**: Addresses accumulated past errors.
- **Kd (Derivative)**: Anticipates future errors to apply corrective actions.

### PID Constants Used:

const float Kp = 1.0;
const float Ki = 0.005;
const float Kd = 0.25;
```

---

## Movement Functions  
These functions control the robot's movement:

- **moveForwardDistance(float distance_cm)**: Moves the robot forward by a specified distance.
- **turnRight90()**: Executes a 90-degree right turn.
- **turnLeft90()**: Executes a 90-degree left turn.
- **alignToRightWall()**: Adjusts the robot's position to maintain a consistent distance from the right wall.

---

## Conclusion  
This autonomous robot navigates its environment using the right-hand rule along with PID-based motion control for precise heading adjustments. With the assistance of VL53L0X distance sensors, it efficiently avoids obstacles and solves mazes, making it a robust solution for maze-solving tasks and similar navigation challenges.

---

