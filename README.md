# Maze-Solving Robot with Right-Hand Rule

## Overview

This project showcases an autonomous robot that solves mazes using the Right-Hand Rule. By always keeping its right side adjacent to a wall, the robot systematically explores and finds its way through a maze. Equipped with wheel encoders, an IMU (MPU6050), and VL53L0X time-of-flight sensors, it maintains accurate movement and environmental awareness.

---

## Hardware Components

- **ESP32 Microcontroller**  
- **H-Bridge Motor Drivers**: Controls left and right wheels.  
- **Wheel Encoders**: Track rotations to measure distance traveled.  
- **MPU6050 IMU**: Supplies gyroscope and accelerometer data for heading corrections.  
- **VL53L0X Time-of-Flight Sensors**: Measure distances to walls and obstacles.  
- **Battery Pack**: Provides power for untethered operation.

---

## I2C Communication

Sensors communicate with the ESP32 over the I2C bus, which uses two lines:

- **SDA (Serial Data Line)**: Carries data.  
- **SCL (Serial Clock Line)**: Synchronizes transfers.  

Each device has a unique address. Custom I2C addresses are assigned to the VL53L0X units to prevent conflicts, while the MPU6050 shares motion data via the same bus.

---

## Right-Hand Rule Navigation

The robot’s decision loop runs every 20 cm:

1. **Right Side Check:** If the path on the right is clear (>200 mm), it turns right and advances.  
2. **Wall Alignment:** If a wall on the right is detected between 65 mm and 150 mm, it adjusts its position to hold a steady gap.  
3. **Front Obstacle:** If an object is closer than 80 mm ahead, it turns left.  
4. **Forward Move:** Otherwise, it proceeds forward by the preset distance.

### Example Pseudocode

```cpp
void rightHandLogic() {
    float distR = measureRight.RangeMilliMeter;
    float distF = measureFront.RangeMilliMeter;
    
    if (distR > 200.0) {
        Serial.println("Right open — turning right");
        turnRight90();
        moveForwardDistance(MOVE_DISTANCE);
    }
    else if (distR > 65.0 && distR < 150.0) {
        Serial.println("Aligning to right wall");
        alignToRightWall();
    }
    else if (distF < 80.0) {
        Serial.println("Front blocked — turning left");
        turnLeft90();
    }
    else {
        Serial.println("Advancing forward");
        moveForwardDistance(MOVE_DISTANCE);
    }
}
```

---

## Motion Control via PID

To maintain stability and precise turns, the robot uses a PID controller:

- **Proportional (K<sub>p</sub>)**: Reacts to the current error.  
- **Integral (K<sub>i</sub>)**: Accounts for accumulated past errors.  
- **Derivative (K<sub>d</sub>)**: Predicts future error trends.

#### PID Formula

\[ u(t) = K_p \cdot e(t) + K_i \int e(t) \,dt + K_d \frac{d e(t)}{d t} \]

#### Tuning Constants

```cpp
const float Kp = 1.0;
const float Ki = 0.005;
const float Kd = 0.25;
```

---

## Core Functions

- **moveForwardDistance(float cm):** Drives forward a specified distance.  
- **turnRight90():** Executes a right-angle turn to the right.  
- **turnLeft90():** Executes a right-angle turn to the left.  
- **alignToRightWall():** Adjusts lateral position to keep constant spacing from the wall.

---

## Conclusion

By combining the Right-Hand Rule logic, real-time sensor data, and PID-based control, this robot navigates and solves mazes reliably. Its modular design allows for upgrades or adaptations to more complex environments.

