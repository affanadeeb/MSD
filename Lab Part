# **Lab Experiment: Sensor Integration and Data Analysis Using a Remote-Controlled Toy**

This lab will focus on the practical application of various sensors, including the wheel encoder, magnetometer, IMU (accelerometer and gyroscope), and range-measuring sensors (LIDAR, ultrasonic, and camera), to help students gain hands-on experience in robotics and sensor data analysis.

---

## **Objective**
To demonstrate the integration and utilization of multiple sensors for:
1. Calculating distance and speed using a wheel encoder.
2. Determining orientation and direction using a magnetometer.
3. Measuring acceleration, angular velocity, and tilt using an IMU.
4. Performing obstacle detection and avoidance using LIDAR, ultrasonic, and camera sensors.

---

## **Materials Required**
1. Remote-controlled toy car (with integrated wheel encoder).
2. ESP32 microcontroller.
3. Magnetometer (e.g., HMC5883L).
4. IMU sensor (e.g., MPU6050).
5. LIDAR sensor (e.g., LIDAR Lite or RPLIDAR).
6. Ultrasonic sensor (e.g., HC-SR04).
7. Camera module (e.g., ESP32-CAM or Raspberry Pi Camera).
8. Breadboard and jumper wires.
9. Laptop with Arduino IDE or other coding platform.
10. Measuring tape.

---

## **Step-by-Step Procedure**

### **Part 1: Setting Up the Sensors**

#### **1. Wheel Encoder Setup**
**Purpose**: Calculate distance traveled and speed.

1. Connect the encoder output pins to the ESP32 GPIO pins.
2. Use the following constants for calculations:
   - Wheel radius (‘r’): Measure and note the radius of the toy car’s wheel (in meters).
   - Motor gear ratio (‘N’): Note the gear ratio of the toy’s motor.
   - Number of magnetic poles (‘P’): Check the encoder’s specifications.
3. Upload the code below to the ESP32:

```cpp
#define ENCODER_PIN 25  // Replace with your GPIO pin
volatile unsigned long pulseCount = 0;  // Stores the pulse count

// Wheel encoder constants
const float wheelRadius = 0.05;  // Radius in meters (e.g., 5 cm)
const int gearRatio = 30;        // Example motor gear ratio
const int poleCount = 20;        // Number of magnetic poles

// Distance per pulse (D)
const float distancePerPulse = (2 * PI * wheelRadius) / (gearRatio * poleCount);

void IRAM_ATTR countPulse() {
  pulseCount++;  // Increment pulse count
}

void setup() {
  Serial.begin(115200);

  // Setup GPIO pin as input with interrupt
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(ENCODER_PIN, countPulse, RISING);  // Interrupt on rising edge
}

void loop() {
  // Calculate total distance
  float totalDistance = pulseCount * distancePerPulse;  // Total distance in meters

  // Print the distance
  Serial.print("Pulse Count: ");
  Serial.println(pulseCount);
  Serial.print("Total Distance: ");
  Serial.print(totalDistance);
  Serial.println(" meters");

  delay(1000);  // Update every second
}
```

**Task for Students**:
   - **Activity**: Driving the toy along a straight path (e.g., 1 meter) and recording the encoder readings.  
   - **What to Show**:  
   - Distance traveled per pulse \( D \) is given by:
     \[
        D = \frac{2\pi r}{NP}
     \]
     - Compare with actual measured distance using a tape.  

---

#### **2. Magnetometer Setup**
**Purpose**: Determine orientation and heading.

1. Connect the magnetometer (e.g., HMC5883L) to the ESP32 via I2C.
   - SDA to GPIO21, SCL to GPIO22.
2. Use the following code to measure the heading:

```cpp
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  Serial.begin(115200);
  if (!mag.begin()) {
    Serial.println("Magnetometer not detected.");
    while (1);
  }
}

void loop() {
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate heading
  float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  if (heading < 0) {
    heading += 360;
  }

  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.println(" degrees");
  delay(1000);
}
```

**Task for Students**:
   - **Activity**: Rotate the toy on a flat surface.  
   - **What to Show**:  
     - Display the heading (in degrees) relative to magnetic north on the screen.  
     - Explain how this helps with navigation.  

---

#### **3. IMU Setup**
**Purpose**: Measure acceleration, angular velocity, and tilt.

1. Connect the IMU (e.g., MPU6050) to the ESP32 via I2C.
2. Use the following code to read acceleration and gyroscope data:

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("IMU not connected.");
    while (1);
  }
}

void loop() {
  // Read acceleration and gyro data
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" Y: "); Serial.print(ay);
  Serial.print(" Z: "); Serial.println(az);

  Serial.print("Gyro X: "); Serial.print(gx);
  Serial.print(" Y: "); Serial.print(gy);
  Serial.print(" Z: "); Serial.println(gz);

  delay(1000);
}
```

**Task for Students**:
- Move the toy and observe the changes in acceleration and angular velocity.

---

#### **4. Range Sensors Setup**
**Purpose**: Perform obstacle detection and avoidance.

   - **LIDAR**: Rotate in place to create a 2D map of surrounding objects.  
   - **Ultrasonic**: Place obstacles at different distances (e.g., 10 cm, 20 cm) and record readings.  
   - **Camera**: Capture live video and identify objects like markers or colors.  

##### **Ultrasonic Sensor (HC-SR04)**
1. Connect the ultrasonic sensor to the ESP32.
2. Use the following code to measure distance:

```cpp
#define TRIG_PIN 5
#define ECHO_PIN 18

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(500);
}
```

**Task for Students**:
- Place an obstacle at different distances and observe the readings.
- Write a program to stop the toy when an obstacle is detected within 10 cm.

---

### **Part 2: Integrating Sensors**

1. **Task 1: Path Navigation Using Wheel Encoder and IMU**:  
   - **Objective**: Drive the toy in a square pattern (1-meter sides).  
   - **How-To**:  
     - Use the wheel encoder to measure distance and turn 90° at each corner using the IMU.  
   - **Observation**:  
     - Check if the toy completes the square and returns to the starting point.  
     - Discuss errors like drift and how to correct them.  

2. **Task 2: Obstacle Avoidance Using Range Sensors**:  
   - **Objective**: Navigate a predefined path with obstacles.  
   - **How-To**:  
     - Use ultrasonic or LIDAR data to detect obstacles within a threshold distance (e.g., 20 cm) and stop or reroute.  
   - **Observation**:  
     - Test how accurately the sensors detect and avoid obstacles.  

3. **Task 3: Object Detection Using Camera**:  
   - **Objective**: Recognize and move toward a colored marker (e.g., red box).  
   - **How-To**:  
     - Process the camera feed to identify the marker using color thresholds (e.g., OpenCV in Python).  
     - Guide the toy to align with and approach the marker.  
   - **Observation**:  
     - Students observe the toy's ability to identify and navigate based on visual input.  

---

### **Part 4: Analysis and Interaction**

1. **Data Visualization**:  
   - Plot sensor readings in real time (e.g., distance vs. time, heading vs. time).  
   - Display the map generated by LIDAR.  

2. **Discussion Points**:  
   - Challenges faced by each sensor (e.g., noise, drift, environmental interference).  
   - How combining sensors improves accuracy (sensor fusion).  

3. **Student Tasks**:  
   - Allowing students to modify parameters like threshold distances or color detection logic.  
   - Asking them to analyze errors and propose improvements.  

---
