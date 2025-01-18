# ROS-Based-Multi-Robot-Interaction-Framework


This project focuses on creating a Robust and Efficient framework for interaction and coordination between a Mobile Robot and an Industrial Robot, leveraging the capabilities of the Robot Operating System (ROS). The framework enables synchronized task execution and improves operational efficiency in dynamic environments.

#### Download Package:
Download Universal Robot Package : https://github.com/ros-industrial/universal_robot/tree/noetic-devel/ur_description

Download Turtlebot Gazebo Model : https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_gazebo

### Behaviour Tree (Mobile Robot):


![Mobile_Robot_Flowchart](https://github.com/user-attachments/assets/a216eb42-4453-4e4f-beb1-cbee1c8640e8)


### Behaviour Tree (Industrial Robot):

![Industrial_Robot_Flowchart](https://github.com/user-attachments/assets/fc32b5b7-c0eb-4ad6-b281-fc8449667bae)


## Key Contributions:
### Communication Framework Development:

Designed and implemented a seamless communication mechanism between the Mobile and Industrial Robots using ROS topics, services, and actions.
Ensured real-time data exchange for task coordination, enhancing system responsiveness.

### Robot Integration:

Successfully mounted and integrated the Industrial Robot on the Mobile Robot using ROS nodes.
Established bi-directional communication between robots to facilitate collaborative operations.

![image](https://github.com/user-attachments/assets/a85f03ef-8eee-4877-9f6d-dbfb27e68d86)

### Simulation and Visualization:

Utilized tools like Gazebo and Rviz to Simulate and Visualize the robots interaction in a virtual environment.
Conducted virtual testing of robot behaviors to minimize errors during physical deployment.

#### RViz Image :

![image](https://github.com/user-attachments/assets/9dd51b84-a455-4a75-beac-d05b7b2a8330)

#### Gazebo Simulation:

![image](https://github.com/user-attachments/assets/c9dcc06a-9f11-4974-a880-445e85edbb1a)

![image](https://github.com/user-attachments/assets/a0298f99-1862-46a8-94f0-8753d3ce3281)


#### Motion and Path Planning:

Developed Motion Planning Algorithm for Precise Manipulation tasks performed by the Industrial Robot.
Implemented Obstacle Avoidance for the Mobile Robot to Navigate complex environments using sensors such as LIDAR, Realsense Camera.

##### Path Planning
![image](https://github.com/user-attachments/assets/943892b9-4f69-4819-b90f-c8077c704fc1)

![combined_nav](https://github.com/user-attachments/assets/a821c342-a0fd-40d6-8123-d53106318cab)

##### Motion Planning
![combined_IR](https://github.com/user-attachments/assets/db3de559-bbde-44c1-91f4-5cd53646abb6)





#### Note:
This framework demonstrates a comprehensive approach to Multi-Robot Interaction, showcasing proficiency in ROS, Robotics Simulation, and Autonomous Navigation. It holds significant potential for Industrial Automation, Logistics, and Collaborative Robotics.
