# TII_assignment
ERRORS:
1. Inside the dockerfile, is specifying the version of ROS to be downloaded. This threw error while creating the image. I removed the version entirely.
2. For some reason (didn't had time to troubleshoot), Gazebo did't installed directly from the dockerfile (I included the commands for Gazebo installation inside the Dockerfile). So, Gazebo needs manual installation from the docker container, using the following command:
$curl -sSL http://get.gazebosim.org | sh

HOW TO RUN THE SIMULATION:
1. Enable access to xhost from the container:
$xhost +local:root

2. Open a terminal and go inside the container. 
$ docker run -it --rm --privileged \
--entrypoint bash \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=:0 \
--network host \
--name=ros2-px4 ros2_px4_image

3. On the terminal within the container,run the PX4 along with the Gazebo. Run the following commands (from within the docker container):
$ cd /PX4-Autopilot
$ make px4_sitl_default gazebo-classic
If the command "make px4_sitl_default gazebo-classic" throws an error, most propably gazebo has not been installed properly. Install it with: $curl -sSL http://get.gazebosim.org | sh

4. Open a new terminal and get inside the docker container:
$ docker exec -it <container_id> bash
From the terminal within the container, run the MicroXRCEAgent:
$cd /Micro-XRCE-DDS-Agent/
$MicroXRCEAgent udp4 -p 8888 
