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

The relevant file there is the URDF (/phantomx_description/robots/phantomx_robot_camera.urdf.xacro). It originally had a stand, which I edited out. It grounds the base_link to the world frame and then links to the next URDF (phantomx_description/urdf/phantomx_reactor_wrist_camera.urdf.xacro), which contains all the joints and links of the arm, along with links to gazebo info (phantomx_description/urdf/phantomx_reactor_wrist_camera.urdf.xacro) and transmission info(/urdf/phantomx.transmission.xacro)
