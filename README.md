HOW TO RUN THE SIMULATION:
1. Enable access to xhost from the container:
$ xhost +local:root

2. From within the folder that includes the Dockerfile and the entrypoint.sh ("docker_folder", the folder is inside the github repository that has been created for the assignment: https://github.com/demetris97/interview_assignment/tree/main), run the following command to build the docker image:
$ docker build -t ros2_px4_image .

2. Open a terminal and go inside the container. 
$ docker run -it --rm --privileged \
--entrypoint bash \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=:0 \
--network host \
--name=ros2-px4 ros2_px4_image

3. For some reason (didn't had time to troubleshoot), Gazebo did't installed directly from the dockerfile (I included the commands for Gazebo installation inside the Dockerfile but Gazebo has not been installed). So, Gazebo needs manual installation from within the docker container, using the following command:
$ curl -sSL http://get.gazebosim.org | sh

4. On the terminal within the container,run the PX4 along with the Gazebo. Run the following commands (the second command will need some time to complete):
$ cd /PX4-Autopilot
$ make px4_sitl_default gazebo-classic_standard_vtol

5. Open a new terminal and get inside the docker container:
$ docker exec -it <container_id> bash
From the terminal within the container, run the MicroXRCEAgent:
$ cd /Micro-XRCE-DDS-Agent/
$ MicroXRCEAgent udp4 -p 8888 

6. A ROS2 package has been created for the assignment. The package must be downloaded from the github folder that also has been created for the assignment (https://github.com/demetris97/interview_assignment). Copy the ROS2 package ("assignment_ros2_package") into the ROS2 workspace inside the docker container: /root/px4_ros_com_ros2 (the ROS package must have the name: "assignment_ros2_package"). Build the ROS2 workspace to run the ROS2 nodes for the assignment.
#Run from host machine. Command to use to find the docker container id:
$ docker ps      
#Run from host machine. Command to copy the ROS package from the host machine to the ROS workspace inside the docker container
$ docker cp /host_machine_path_to_the_downloaded_ROS2_package/assignment_ros2_package <docker_container_id>:/root/px4_ros_com_ros2/src/assignment_ros2_package           
#Run the following command from within the docker container to build the ROS workspace and run the autonomous missions for the assignment.
$ cd /root/px4_ros_com_ros2/src/px4_ros_com/scripts
$ ./build_ros2_workspace.bash
$ source /opt/ros/humble/setup.bash
$ source /root/px4_ros_com_ros2/install/setup.bash
$ ros2 run assignment_ros2_package ros2_node_function1_position_control       #to run the position control node (autonomous mission) 
$ ros2 run assignment_ros2_package ros2_node_function2_velocity_control       #to run the velocity control node (autonomous mission)

7. For the plots:
$ sudo apt install ros-humble-plotjuggler
$ sudo apt install ros-humble-plotjuggler-ros
Run the PlotJuggler with:
$ source /opt/ros/humble/setup.bash
$ source /root/px4_ros_com_ros2/install/setup.bash
$ ros2 run plotjuggler plotjuggler

