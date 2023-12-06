# EE204A_Team5
Team 11 Autonomous Racing



##### Team Members
Elena
Hooman
Haard
Akhdan
Saikhanbileg

## How to use: 
1. Launch the sensors on board the car using: 
    `roslaunch realsense2_camera rs_rgbd.launch`

2. Launch the ROS Serial communication for the Arduino: 
    `rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600`

3. Launch the rc car controller: 
    `roslaunch rc_car_control rc_car_controller.launch`

4. Lauch the ORB SLAM for localisation and odometry tracking:
    `roslaunch orb3_ros_interface rgbd.launch`

5. For preception systems: [Recommend to use a new tab]
    `roslaunch darkent_ros custom.launch`

6. Motion Planning systems: [Recommend to use a new tab]
    a. First launch the depth2pointcloud package: 
     `roslaunch pointcloud_projection depth_image_projection.launch`

    b. Next launch the cost map generator so the motion planner can use it to plan the paths: 
     `roslaunch local_costmap_generator run.launch`

    c. Next launch the motion primitive planner: 
     `roslaunch motion_primitives_planner run.launch`

7. Preception system: [Recommend to use a new tab]
    a. Launch the Yolo v4 model 
     ``

    b. Launch the image saver node
     ``
    



##### Test Commit
Test Commit
