**Open Project Plan Template**

Return the project plan in PDF format in the course&#39;s Moodle page.

Upload also a copy to GitHub in Markdown format.

1. Team name and team members (up to 4 persons/team)

Sebastian Icking - [sealic@utu.fi](mailto:sealic@utu.fi)

Kimmo Korpelin – [kikkor@utu.fi](mailto:kikkor@utu.fi)

Daniel Montero – [daanmh@utu.fi](mailto:daanmh@utu.fi)

Gabriel Pirlogeanu - [gapirl@utu.fi](mailto:gapirl@utu.fi)

2. Application / Use-case

We want to combine navigating through an unknown environment with a combination of drone and Jetbot movement, where part of the environment might be inaccessible to either of the robots.

3. The system

Landing the drone on top of the Jetbot also requires synchronized positioning.

- **Robots:**

1 Jetbots and 1 Drone

- **Computing Platform:**

Due to the Jetbot processing power not being good enough, we might add one of our laptops as a data processing is desirable, however, it would not act as the master.

- **Sensors:**

LIDAR sensor and Cameras for the Jetbot

- **Communication:**

Most communication should happen through ROS topics, as it&#39;s the most stable way to send camera information.

  - **Algorithms:**
    - **Lidar:**
      - SLAM
      - Potential Fields (Jetbot avoidance)
    - **Visual:**
      - Landing algorithm
      - Fiducial recognition
      - Drone search/navigation algorithm

- Data flow:

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

![](RackMultipart20210318-4-kzcb9b_html_7883d5d9a911491d.png)

4. GitHub repo link

[https://github.com/kimmk/ras-bots](https://github.com/kimmk/ras-bots)

5. Background

The team already has some experience receiving data from a robot to a laptop using ROS topics, so we are confident that the communication part will part, however, we have not done communication between robots in the past.

We&#39;re also somewhat confident of the LIDAR-SLAM Potential Fields driving and basic navigation for the Jetbot to avoid obstacles, but we have not yet built a combination of both systems at the same time.

Finally, the drone &quot;discovery&quot; path is one of the riskiest parts of the project. While we have successfully received and transformed data from a drone in the past, we have yet to make a &quot;search &amp; find&quot; functionality.

6. Expected challenges and wishes to learn

Main expected challenge is the &quot;search &amp; find&quot; algorithm.

Other important challenges are: communication between robots, managing the processing capabilities of the Jetbot and not crashing the Drone.

We would like to know more about path-planning and obstacle avoidance if possible. If we were to add mapping to our project, then some information regarding this would be nice, otherwise we would need to investigate how to combine visual images with sensor data.

One known challenge is getting quality imagery from the drone, since the data tends to be poor.

7. Team roles

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

8. Work packages (how is the work going to be divided among team members and in time), with tentative project schedule.

- SLAM (Mapping room with LIDAR)
- Jetbot Command
- Jetbot Autonomous Movement
- Fiducial Interpretations
- Drone Command
- Drone Data Interpretation
- Drone Search&amp;Find Algorithm
- Landing Algorithm (Jetbot Camera)

9. Description of final experiment or demonstration.

We let the Jetbot with the docked Drone into a space. The Jetbot drives around and eventually settles. Jetbot commands the Drone around to do discovery, and once the goal is reached (fiducial finding), the Drone is autonomously landed on the Jetbot, and the Jetbot can leave.