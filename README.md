ERRORS:
1. Inside the dockerfile, is specifying the version of ROS to be downloaded. This threw error while creating the image. I removed the version entirely.
2. For some reason (didn't had time to troubleshoot), Gazebo did't installed directly from the dockerfile (I included the commands for Gazebo installation inside the Dockerfile). So, Gazebo needs manual installation from the docker container, using the following command:
$curl -sSL http://get.gazebosim.org | sh

HOW TO RUN THE SIMULATION:
1. Enable access to xhost from the container:
$xhost +local:root

2. From within the folder that includes the Dockerfile and the entrypoint.sh, create the docker image with the following command:
$docker build -t ros2_px4_image .

2. Open a terminal and go inside the container. 
$ docker run -it --rm --privileged \
--entrypoint bash \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=:0 \
--network host \
--name=ros2-px4 ros2_px4_image

3. For some reason (didn't had time to troubleshoot), Gazebo did't installed directly from the dockerfile (I included the commands for Gazebo installation inside the Dockerfile but Gazebo has not been installed). So, Gazebo needs manual installation from within the docker container, using the following command:
$curl -sSL http://get.gazebosim.org | sh

4. On the terminal within the container,run the PX4 along with the Gazebo. Run the following commands (the second command will need some time to complete):
$ cd /PX4-Autopilot
$ make px4_sitl_default gazebo-classic

5. Open a new terminal and get inside the docker container:
$ docker exec -it <container_id> bash
From the terminal within the container, run the MicroXRCEAgent:
$cd /Micro-XRCE-DDS-Agent/
$MicroXRCEAgent udp4 -p 8888 

6. Initiate the ROS2 workspace to run the ROS2 nodes for the assignment. e.g.:
$cd ~/px4_ros_com_ros2
$source /opt/ros/humble/setup.bash
$source /root/px4_ros_com_ros2/install/setup.bash
$ros2 run px4_ros_com offboard_control => arms and lands the drone.

more examples (offroad mission):https://docs.px4.io/v1.12/en/ros/ros2_offboard_control.html 
