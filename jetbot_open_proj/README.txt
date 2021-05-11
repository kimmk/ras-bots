STEPS TO SETUP JETBOT FOR USAGE

NOTE: we run the master on the jetbot's computer and there we activate the rplidar and jetbot ros cmd_vel topics. On local machine we get the odometry and do the hector slam
 
 1) connect with USB cable and setup JETBOT to the same network as you with command:sudo nmcli device wifi connect <MY_WIFI_AP> password <MY_WIFI_PASSWORD>
 2) connect cu wlan0 after disconnecting cable(you should be able to connect through both ssh and wlan0 ip in the same network if cable is connected)
 3) ALWAYS do this in Jupyter Lab's EVERY TERMINAL:
 -export ROS_MASTER_URI=http://<jetbot's_ip>:11311
 -export ROS_IP=<jetbot's_ip> 
 4) ALWAYS do this in local computer in EVERY TERMINAL:
 -export ROS_MASTER_URI=http://<jetbot's_ip>:11311
 -export ROS_IP=<localhost>  (we do this so we can publish from laptop to jetbot)
 P.S: some say that they can receive the messages even if you dont write all the exports in every computer, but in my case it didn't work otherwise

 5) TO ACTIVATE RPLIDAR:
    - ls /dev  (search for ttyUSB0 or ttyUSB1)  (optional)
    - ls -l /dev | grep ttyUSB   (optional)
    - sudo chmod 666 /dev/ttyUSB0 (always before using lidar)
    - IN JUPYTER's LAB: roslaunch rplidar_ros rplidar.launch
    - search for /scan topic and see if you receive messages on jetbot and pc
 6) TO GET ODOMETRY FROM LIDAR:
    - We use rf2o_laser_odometry package to get odometry from laser_scan data
    - in own PC after adding package and building in your workspace run: roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch

 P.S: CAREFUL TO CLONE THIS repository: https://github.com/artivis/rf2o_laser_odometry. The master one is garbage
 7) Make HECTOR_SLAM after getting odometry and lidar data:
    - get hector_slam package in your workspace
    - roslaunch hector_slam_launch tutorial.launch
 8) CONTROL MOVEMENT of jetbot:  
    - On the jetbot's computer in jupyter's lab there is an inference, deep_learning and jetbot_ros movement control package already added by me.
    - With the Adafruit libraries we control the motors of the jetbot and create ros topics for movement control: cmd_str, cmd_dir, cmd_raw
    - we run rosrun jetbot_ros jetbor_motors.py
    - if we want to control the jetbot we can publish to the /cmd_str topic from our pc. (for exemple: rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "forward")
    - in jetbot_motors.py we can modify the speed of the motors
    - for autonomous navigation: create a file where the robot moves autonomously by publishing to /cmd_str 
 9) Jetbot OLED:
    - rosrun jetbot_ros jetbot_oled.py
    - rostopic pub /jetbot_oled/user_text std_msgs/String --once "HELLO!"
 10) Jetbot CAMERA:
    - rosrun jetbot_ros jetbot_camera
    - rosrun image_view image_view image:=/jetbot_camera/raw

for tf graph:

rosrun rqt_tf_tree rqt_tf_tree

reference:

https://github.com/dusty-nv/jetbot_ros
https://github.com/tu-darmstadt-ros-pkg/hector_slam_launch
http://wiki.ros.org/hector_slam
https://github.com/artivis/rf2o_laser_odometry
