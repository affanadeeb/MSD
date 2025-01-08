### **Lab Experiment: Understanding Sensors in Motion**

#### **Objective**
To understand the practical applications of wheel encoders, magnetometers, IMUs, and range-measuring sensors by integrating them with a remote-controlled toy for navigation and obstacle avoidance.

---

### **Part 1: Preparation**

#### **Materials Needed**:
1. **Sensors**:  
   - Wheel encoder (already integrated with the toy).  
   - Magnetometer (e.g., HMC5883L).  
   - IMU (e.g., MPU6050 or MPU9250 for accelerometer and gyroscope).  
   - LIDAR (e.g., RPLIDAR).  
   - Ultrasonic sensor (e.g., HC-SR04).  
   - Camera module (e.g., Raspberry Pi Camera).  

2. **Hardware**:  
   - Remote-controlled toy.  
   - Microcontroller (e.g., Arduino, Raspberry Pi, or ESP32).  
   - Power supply.  

3. **Software Tools**:  
   - Arduino IDE or Python (for sensor data processing).  
   - Visualization tools (e.g., MATLAB, Python libraries like matplotlib).  

4. **Miscellaneous**:  
   - Laptop/PC for coding and data visualization.  
   - Measuring tape.  
   - Obstacles (boxes, cones).  
   - Markers or colored objects for camera-based tasks.  

---

### **Part 2: Experimental Setup**

1. **Assembling Sensors on the Toy**:
   - Mounting the IMU at the center of the toy for balanced measurements.
   - Attaching the magnetometer in an open area (away from metal parts to avoid interference).
   - Place the ultrasonic sensor at the front for obstacle detection.
   - Attach the camera for forward-facing visual data.
   - Connecting LIDAR to provide a 360° mapping capability.

2. **Wiring and Power**:
   - We should connect all sensors to the microcontroller following their respective pin configurations should ensure all components share a common ground.

3. **Calibrate Sensors**:
   - **Wheel Encoder**: Roll the toy forward and back while counting pulses to ensure accurate readings.
   - **Magnetometer**: Rotate 360° to calibrate and offset magnetic field interference.
   - **IMU**: Place the toy on a flat surface to set baseline readings for accelerometer and gyroscope.

4. **Code Upload**:
   - We should upload a basic script to the microcontroller to log sensor data. For instance:
     - Log wheel encoder data to measure distance.
     - Display magnetometer readings for orientation.
     - Visualize IMU readings for tilt and movement.
     - Show range sensor distances.

---

### **Part 3: Conducting the Experiment**

#### **Step 1: Demonstrating Each Sensor**  

1. **Wheel Encoder**:  
   - **Activity**: Driving the toy along a straight path (e.g., 1 meter) and recording the encoder readings.  
   - **What to Show**:  
   - Distance traveled per pulse \( D \) is given by:
     \[
        D = \frac{2\pi r}{NP}
     \]
     - Compare with actual measured distance using a tape.  

2. **Magnetometer**:  
   - **Activity**: Rotate the toy on a flat surface.  
   - **What to Show**:  
     - Display the heading (in degrees) relative to magnetic north on the screen.  
     - Explain how this helps with navigation.  

3. **IMU**:  
   - **Activity**: Tilt the toy in various directions and shake it lightly.  
   - **What to Show**:  
     - Real-time accelerometer readings showing movement in x, y, and z axes.  
     - Gyroscope readings showing angular velocity.  

4. **Range Sensors**:  
   - **LIDAR**: Rotate in place to create a 2D map of surrounding objects.  
   - **Ultrasonic**: Place obstacles at different distances (e.g., 10 cm, 20 cm) and record readings.  
   - **Camera**: Capture live video and identify objects like markers or colors.  

---

#### **Step 2: Integrating Sensors for Tasks**

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
   - Allow students to modify parameters like threshold distances or color detection logic.  
   - Ask them to analyze errors and propose improvements.  

---

### **Expected Learning Outcomes**

- Hands-on understanding of sensor principles and their limitations.  
- Practical application of navigation and obstacle detection techniques.  
- Basics of sensor data integration and visualization.  



Got it! We can incorporate these formulas for distance and speed calculation directly into the experimental procedure to make it more aligned with what your professor provided. Here’s the updated approach:

---

### **Updated Lab Experiment: Understanding Sensors in Motion with Wheel Encoder**

---

#### **Part 3: Conducting the Experiment**

---

### **Step 1: Demonstrating the Wheel Encoder**

#### **Objective**  
Measure the distance traveled and calculate the speed of the toy using the given formulas.

#### **Procedure**  

1. **Understanding the Formula**:  
   - The distance moved per pulse \( D \) is given by:  
     \[
     D = \frac{2\pi r}{NP}
     \]  
     
