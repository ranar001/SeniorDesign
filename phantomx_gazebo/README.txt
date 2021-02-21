Original: https://github.com/RobotnikAutomation/phantomx_reactor_arm
Gazebo adaptation: https://github.com/KTigerFIre/phantomx_gazebo

To download, go to your catkin_ws/src file and type:
git clone https://github.com/ranar001/SeniorDesign/phantomx_gazebo

To run, type each roslaunch in a terminal:
roslaunch phantomx_gazebo phantom_gazebo_camera.launch
roslaunch phantomx_reactor_arm_moveit_config demo_camera.launch

For the camera to work, in RVIZ, click the camera check and change the image_topic to the drop down option that appears when you click it.
Move the slides around in the ? window called joint_state_publisher

What's happening:
phantomx_gazebo/launch/phantom_gazebo_camera.launch opens an empty world(/phantomx_gazebo/launch/empty_world.launch), the urdf(/phantomx_gazebo/robots/phantomx_robot_camera.urdf.xacro), controller utils which set up the joint_state_controller and the robot_state_publisher which and yaml's which set up PID, damping, etc for joints. 

The relevant file there is the URDF (/phantomx_description/robots/phantomx_robot_camera.urdf.xacro). It originally had a stand, which I edited out. It grounds the base_link to the world frame and then links to the next URDF (/phantomx_description/urdf/phantomx_reactor_wrist_camera.urdf.xacro), which contains all the joints and links of the arm, along with links to gazebo info (/phantomx_description/urdf/phantomx.gazebo.xacro) and transmission info(/phantomx_description/urdf/phantomx.transmission.xacro). At the bottom, a camera link and a camera joint were added, and a camera optical link and joint. They were added at the wrist2 link, which is the 2nd rotating motor near the end of the arm, slightly above the center.  

Info about the camera changes came from here: http://gazebosim.org/tutorials?tut=ros_gzplugins. I used the code from there, along with variable names and the addition of two camera links in the gazebo file (/phantomx_description/urdf/phantomx.gazebo.xacro). 

Now for RVIZ, we start with: (/phantomx_reactor_arm_moveit_config/launch/demo_camera.launch). The planning context file (/phantomx_reactor_arm_moveit_config/launch/planning_context_camera.launch) contains the URDF (/phantomx_description/robots/phantomx_reactor_arm_wrist_camera.urdf.xacro) with the base/world definition and the URDF from earlier (phantomx_description/urdf/phantomx_reactor_wrist_camera.urdf.xacro). Back to planing context, there's a semantic description of the URDF called SRDF. I didn't change it so idk what it does. Then we have joint config yaml files for kinematics and joint limits. 

Back in the demo_camera.launch file, we have a joint_state_publisher, which has a slider bar but that bar doesn't exist when we have actual controllers I think. We have a robot_state_publisher that communicates tf frames, which are little XYZ frames for each relevant point in the arm. 

And we have moveit_rviz.launch which launches the RVIZ file and move_group.launch which handles groups of the arm. I didn't really touch these. 

Next I'm gonna write python files similar to take_photo.py from here: (https://github.com/markwsilliman/turtlebot/), add a range sensor from here: (http://gazebosim.org/tutorials?tut=ros_gzplugins), attach this arm to a basic cart. and go over some 144 labs for sending python commands to the cart. 
