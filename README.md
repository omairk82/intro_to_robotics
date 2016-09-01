# intro_to_robotics
"Intro to Robotics" course projects from USC M.S. in intelligent robotics

file allocation:
* PID_loop.C - Simple PID loop to keep rover driving along wall at goal delta = 30 inches. Calls to NEMO library are to send commands to microcomputer on the mobile robot.
* wheel_sensor_drive.C - Control loop to drive wheels and sensors on mobile robot so that it can move forward in-between two walls with goal of staying equidistant from each wall face. Calls to NEMO library are to send commands to microcomputer on the mobile robot.
* vision_navigation.C - Computer vision-based navigation of a mobile robot by focusing on following a pre-defined color (e.g. RGB) pattern. Mobile robot command/telemetry is handled by a web interface. Calls to NEMO library are to send commands to microcomputer on the mobile robot.
* part_filter.C, Particle.H, Util.H - Implemented markov decision process (MDP)-based navigation of a mobile robot utilizing a particle filter algorithm for localization and computer vision based sensing for navigation. Mobile robot command/telemetry is handled by a web interface. Calls to NEMO library are to send commands to microcomputer on the mobile robot.
* soccer_robot.C - State-machine based code architecture to control PID-based mobility robot with soccer-playing end effectors (e.g. ball rotation rollers and ball-shooting mechanism). Sensor input is based on pressure sensors, computer vision (camera-based tracking of pre-defined RGB patterns), and sonars. States include search/retrieve ball, dribble, defend, shoot goal, and obstacle avoidance.
