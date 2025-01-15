# ROS-Based-Multi-Robot-Interaction-Framework


This project focuses on creating a robust and efficient framework for interaction and coordination between a Mobile Robot and an Industrial Robot, leveraging the capabilities of the Robot Operating System (ROS). The framework enables synchronized task execution and improves operational efficiency in dynamic environments.

### Behaviour Tree (Mobile Robot):

Root (Mobile Robot)
├── Execution
│   ├── Sequence (Avoid Obstacle)
│   │   ├── Detect Obstacle (Sensor Input)  & Signal -> IR ( Sensor Fusion )
│   │   ├── Stop
│   │   ├── Turn Left/Right
│   │   └── Resume Exploration
│   ├── Sequence (Return to Base)
│   │   ├── Check Battery
│   │   ├── Navigate to Charging Station
│   │   └── Recharge
│   └── Sequence (Exploration)
│       ├── Move Forward
│       ├── Scan for Obstacles
│       └── No Obstacle -> Continue Moving

### Behaviour Tree (Industrial Robot):

Root: Industrial Robot
├── Execution
│   ├── Sequence: Destination Reached (Signal Received from Mobile Robot)
│   │   ├── Detect Obstacle (Camera Input)
│   │   ├── Overcome the Obstacle and Execute the Task
│   │   ├── Done Task -> Signal to Mobile Robot (Move) 
│   │   └── Failed Task -> Signal to Mobile Robot (Change or Move Location)
│   ├── Sequence: Obstacle Signal from Mobile Robot
│   │   ├── Detect Obstacle (Camera Input)
│   │   ├── Decision Making to Avoid Obstacle (Change the IR Position)
│   │   └── Send Signal to Navigate the Mobile Robot
│   └── Sequence: Exploration
│       ├── No Obstacle
│       ├── Execute the Task
│       └── Done Task -> Signal to Mobile Robot

## Key Contributions:
### Communication Framework Development:

Designed and implemented a seamless communication mechanism between the mobile and industrial robots using ROS topics, services, and actions.
Ensured real-time data exchange for task coordination, enhancing system responsiveness.

### Robot Integration:

Successfully mounted and integrated the Industrial Robot on the Mobile Robot using ROS nodes.
Established bi-directional communication between robots to facilitate collaborative operations.

![image](https://github.com/user-attachments/assets/a85f03ef-8eee-4877-9f6d-dbfb27e68d86)

### Simulation and Visualization:

Utilized tools like Gazebo and Rviz to simulate and visualize the robots' interaction in a virtual environment.
Conducted virtual testing of robot behaviors to minimize errors during physical deployment.

#### RViz Image :

![image](https://github.com/user-attachments/assets/9dd51b84-a455-4a75-beac-d05b7b2a8330)

#### Gazebo Simulation:

![image](https://github.com/user-attachments/assets/c9dcc06a-9f11-4974-a880-445e85edbb1a)

![image](https://github.com/user-attachments/assets/a0298f99-1862-46a8-94f0-8753d3ce3281)


### Motion and Path Planning:

Developed Motion Planning algorithms for precise manipulation tasks performed by the Industrial Robot.
Implemented Path Planning for the Mobile Robot to navigate complex environments using sensors such as LIDAR, Realsense Camera.

#### Note:
This framework demonstrates a comprehensive approach to multi-robot interaction, showcasing proficiency in ROS, robotics simulation, and autonomous navigation. It holds significant potential for industrial automation, logistics, and collaborative robotics.
