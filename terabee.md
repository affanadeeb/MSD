## Running the teraranger_evo Node

To run the Lidar node and visualize the data:

1. **Connect the ToF sensor**: Ensure your sensor is connected to your system.

2. **Open a Terminal in VSCode**:
   - Navigate to the terminal within VSCode.

3. **Source the ROS 2 Environment**:
   - Run: `source /opt/ros/humble/setup.bash`

4. **Launch the teraranger_evo Node**:
   - For TR-EVO-60M-USB, execute:
        ```bash
        ros2 launch teraranger_evo teraranger_evo.launch.py
        ```
   - This command will start the Lidar node and publish data on the rostopic .

5. **Publishing the data**
    - Echo the rostopic to see the data in real time
        ```
        ros2 topic echo /teraranger_evo/range 
        ```
