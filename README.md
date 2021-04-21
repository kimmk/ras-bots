# Instructions - WIP
Necessary installations:

Drone related drivers and dependencies.
Installation of TIERS' ROS Driver and Python Driver.

```
cd ~/ras-bots/src/tello-driver-ros
git clone --recursive https://github.com/TIERS/tello-driver-ros.git
sudo apt install ros-melodic-camera-info-manager-py ros-melodic-codec-image-transport python-catkin-tools python3-dev python3-pip python-dev python-pip
sudo -H pip3 install --upgrade pip
sudo -H pip3 install https://github.com/damiafuentes/DJITelloPy/archive/master.zip
```
Building the project

```
cd ~/ras-bots
catkin init
catkin build
```
# Open Project Plan: Aircraft Carrier & Discovery

## 1. Team name and team members (up to 4 persons/team)
Sebastian Icking - [sealic@utu.fi](mailto:sealic@utu.fi)

Kimmo Korpelin – [kikkor@utu.fi](mailto:kikkor@utu.fi)

Daniel Montero – [daanmh@utu.fi](mailto:daanmh@utu.fi)

Gabriel Pirlogeanu - [gapirl@utu.fi](mailto:gapirl@utu.fi)

## 2. Application / Use-case
We want to combine navigating through an unknown environment with a combination of drone and Jetbot movement, where part of the environment might be inaccessible to either of the robots.

## 3. The system
Landing the drone on top of the Jetbot also requires synchronized positioning.
-  **Robots:**
1 Jetbots and 1 Drone
-  **Computing Platform:**
Due to the Jetbot processing power not being good enough, we might add one of our laptops as a data processing is desirable, however, it would not act as the master.
-  **Sensors:**
LIDAR sensor and Cameras for the Jetbot
-  **Communication:**
Most communication should happen through ROS topics, as it&#39;s the most stable way to send camera information.

-  **Algorithms:**
	-  Lidar:
		- SLAM
		- Potential Fields (Jetbot avoidance)
	-  Visual:
		- Landing algorithm
		- Fiducial recognition
		- Drone search/navigation algorithm

 
- **Data flow:**
Communication happens through ROS and there should be a constant flow of information between Drone and Jetbot. The Jetbot makes decisions and commands the Drone to move around.

	- Jetbot-LIDAR
		- Sensor data is processed to allow Jetbot to move around
		- Jetbot decides on resting place
		- Jetbot commands the Drone for take off
	- Drone-Camera
		- Drone streams camera data to Jetbot
		- Jetbot instructs the Drone to move around for Discovery
		- Jetbot recognizes Fiducial and instructs Drone to come back
	- Jetbot-Camera
		- Once the Drone is in the field of view, send commands to correct positioning
		- Jetbot commands Drone to land

![enter image description here](https://github.com/kimmk/ras-bots/blob/master/images/dataflow_RAS_01.png?raw=true)  

## 4. GitHub repo link
[https://github.com/kimmk/ras-bots](https://github.com/kimmk/ras-bots)
  
## 5. Background
The team already has some experience receiving data from a robot to a laptop using ROS topics, so we are confident that the communication part will part, however, we have not done communication between robots in the past.

We&#39;re also somewhat confident of the LIDAR-SLAM Potential Fields driving and basic navigation for the Jetbot to avoid obstacles, but we have not yet built a combination of both systems at the same time.

Finally, the drone &quot;discovery&quot; path is one of the riskiest parts of the project. While we have successfully received and transformed data from a drone in the past, we have yet to make a &quot;search &amp; find&quot; functionality.

## 6. Expected challenges and wishes to learn
Main expected challenge is the &quot;search &amp; find&quot; algorithm.
Other important challenges are: communication between robots, managing the processing capabilities of the Jetbot and not crashing the Drone.

We would like to know more about path-planning and obstacle avoidance if possible. If we were to add mapping to our project, then some information regarding this would be nice, otherwise we would need to investigate how to combine visual images with sensor data.

Another known challenge is getting quality imagery from the drone, since the data tends to be poor.

## 7. Team roles
- Sebastian
	- Physical modification of the Jetbot
	- Landing Algorithm

- Kimmo
	- Drone Data Interpretation
	- Drone Search&amp;Find Algorithm

- Daniel
	- Drone Commands
	- Jetbone Commands
	- Fiducial Interpretation

- Gabriel
	- SLAM (Mapping room with LIDAR)
	- Jetbot Autonomous Movement

## 8. Work packages (how is the work going to be divided among team members and in time), with tentative project schedule.

  Tentative Project Schedule: [Drive GANTT](https://docs.google.com/spreadsheets/d/1XahKZe0qL9Kq-8Cc7oUSd-BmwO9jYJjiQDrlhUxS5nU/edit?usp=sharing)

## 9. Description of final experiment or demonstration.
We let the Jetbot with the docked Drone into a space. The Jetbot drives around and eventually settles. Jetbot commands the Drone around to do discovery, and once the goal is reached (fiducial finding), the Drone is autonomously landed on the Jetbot, and the Jetbot can leave.
