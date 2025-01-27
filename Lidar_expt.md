## Running the Lidar Node

To run the Lidar node and visualize the data:

1. **Connect the Lidar Device**: Ensure your Lidar device is connected to your system.

2. **Open a Terminal in VSCode**:
   - Navigate to the terminal within VSCode.

3. **Source the ROS 2 Environment**:
   - Run: `source /opt/ros/humble/setup.bash`

4. **Launch the Lidar Node**:
   - For RPLIDAR A2M12, execute:
     ```bash
     ros2 launch sllidar_ros2 view_sllidar_a2m12_launch.py
     ```
   - This command will start the Lidar node and open RViz for visualization.

**Note**: Replace `a2m12` with your specific Lidar model if different. For other models, refer to the [sllidar_ros2 GitHub repository](https://github.com/Slamtec/sllidar_ros2) for the appropriate launch file.
