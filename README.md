# A Path Planning and Navigation Controls System


AutoBot is a an advanced path planning and SLAM system capable of navigating to any destination. It was built with the Robotic Operating System and is currently being tested on the CARLA simulator for better accuracy. The path planning system was developed from the integration of many algorithms working side by side with each other. 

For global path planning, Djikstra's algorithm was utilized which converted the grid-like representation of the environment map produced by the gmapping package into a a node graph like sturcutre to perform the algorithm on. The system then would utilize the Dynamic Window Approach as a local path planning mechanism to avoid dynamic obstacles and plan paths to each coordinate point along the path. This version of the DWA local path planner included an additional cost function which prioritized the smoothness of the trajectories when comparing possible linear and angular velocities. Finally,the system employed an optimization algorithm called Particle Swarm Optimization which asynchronously optimized the weights/constants used in the DWA approach after every iteration of the robot's movement. 

After each linear and angular velocity is calculated, the output is carefully tuned and conrolled through the use of a PID closed control loop system. To indicate start and stop, the ros workspace also includes a vision package that uses OpenCV and MediaPipe to analyze the users hand signals and determine whether the user has indicated for the car to start or stop. 


# Dependencies

This repository was built with Cmake and the catkin build system using ROS1 Noetic. The operating system requirements for this particular project is Linux Ubuntu 20.04. Anyone wishing to test this project must also clone and download the GMapping and AMCL Package from their respective online github repos.
