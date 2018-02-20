# Autonomous-Rover
This is a LEGO Technic Autonomous Rover. This vehicle utilised my knowledge and understanding gained from the Udacity AI and Robotics courses and the Coursera Robotics Specialization (in progress). The chassis was made out of LEGO Technic pieces. The main onboard computing was done with a Raspberry Pi 3 with associated IMU and Pi Camera components. This hardware facilitated robot perception, robotic search/motion planning and obstacle avoidance. This toy-scale autonomous rover would represent AI used in a robotic car.

Hardware
- Raspberry Pi 3
- IMU (SparkFun IMU Breakout - MPU-9250)
- Pi Camera (Raspberry Pi Camera Board v2 - 8MP)
- L298N Dual H-bridge DC Motor Controller
- 2 x micro DC motors

Aims
- Computer vision to detect obstacles/path
- Implement robot motion planning
- Carry out Python and OpenCV image processing on the Raspberry Pi
- Create custom laser-cut/3D-printed parts to connect the hardware to LEGO

Task CheckList:
- HSV thresholding (DONE - need to remove noise)
- Move in a square (Done)
- Motion with keyboard/Xbox functionality (Done)
- Motion with control feedback loop
- Track ball with openCV
- Rover program (autonomous) (perception + motion)
- Path Planning (A*)
