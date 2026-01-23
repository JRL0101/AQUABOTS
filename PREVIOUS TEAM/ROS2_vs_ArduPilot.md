# ROS 2 vs. ArduPilot for Navigation and Mission Planning

| Feature | ROS 2 | ArduPilot |
| --- | --- | --- |
| Navigation stack | Nav2 provides path planning and obstacle avoidance for robots, with plugins for marine vessels. | Built-in navigation tailored for UAVs, Rovers, and Boats with mature waypoint navigation. |
| State estimation | Supports `robot_localization` and many filter packages for sensor fusion. | Uses EKF/UKF engines integrating GPS, IMU, and barometer data. |
| Mission planning | Flexible mission scripting via Python/C++ nodes; integration with RViz. | Mission Planner and MAVProxy offer GUI and scripting for complex missions. |
| Hardware support | Requires custom drivers or ROS 2 wrappers for sensors/actuators. | Extensive hardware abstraction through HAL with many supported boards. |
| Community & maturity | Large robotics community, active development. | Long-term autonomy focus, proven in field with extensive documentation. |

**Summary:** ROS 2 excels when integrating diverse software components and advanced autonomy algorithms, especially if a flexible middleware is desired. ArduPilot offers a more turnkey solution with robust navigation and mission capabilities out of the box for small autonomous vehicles. For Aquabot, ArduPilot may reduce development overhead for navigation and mission execution, while ROS 2 is preferable if deep customization or integration with other robotics frameworks is required.
