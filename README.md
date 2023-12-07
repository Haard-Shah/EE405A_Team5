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

2. Lauch the ORB SLAM for localisation and odometry tracking:  
    `roslaunch orb3_ros_interface rgbd.launch`  

3. For preception systems: [Recommend to use a new tab]  
    `roslaunch darkent_ros custom.launch`  

4. Motion Planning systems: [Recommend to use a new tab]  
    a. First launch the depth2pointcloud package:  
     `roslaunch pointcloud_projection depth_image_projection.launch`  

    b. Next launch the cost map generator so the motion planner can use it to plan the paths:  
     `roslaunch local_costmap_generator run.launch`  

    c. Next launch the motion primitive planner:  
     `roslaunch motion_primitives_planner run.launch`  

    d. Launch the ROS Serial communication for the Arduino:  
     `sudo chmod 777 /dev/tty*`  
     `rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600`  

    e. Launch the rc car controller:  
     `roslaunch rc_car_control rc_car_controller.launch`  

5. Preception system: [Recommend to use a new tab]  
    a. Launch the Yolo v4 model  
     `roslaunch darknet_ros custom.launch`  

    b. Launch post processing node  
      `rosrun post_process process.py`  

    c. Launch qr detection node  
      `rosrun nodelet nodelet manager __name:=nodelet_manager`  
      in other terminal,  
      `rosrun nodelet nodelet load qr_detector/qr_detector_nodelet nodelet_manager`  

    d. Launch chat GPT  
      `rosrun post_process chat.py`  
      or  
      `rosrun post_process chat_new.py`  

    e. Launch the image saver node  
     `rosrun final_result final_result`  
    

### NOTES: 
* ~/.bashrc has been updated with the source script.
* For Rviz visualisation, there is a config file save in the EE405A_Team5 directory and can be used by calling `rosrun rviz rviz -d <filename>`
* currntly the auto_mode is off to toggle it on for autonomous driving run `rostopic pub /auto_mode std_msgs/Bool 1`
* If the car speed is to slow or fast, adjust the upper and lower bounds of the PWM in the `rc_car_controller.py` [ONLY IF NEEDED] 
