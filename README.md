# ROS_Autonomous_Robot_Real_World

## Configurations

1. Setup up raspberry PI 4B

    1. Install Ubuntu-server
		- Install Ubuntu 20.04 from Raspberry Pi Imager to a SD card

    2. Install Ubuntu-desktop
		- Insert SD card to Raspberry Pi and monitor. Then;
            ```
                $ sudo apt update
                $ sudo apt upgrade
                $ sudo apt install ubuntu-desktop
                $ reboot
            ```

	3. If you have any problem with connection to wifi try this:
	    - https://raspberrypi.stackexchange.com/questions/111722/rpi-4-running-ubuntu-server-20-04-cant-connect-to-wifi
		- Only add new network to /etc/netplan/50-cloud-init.yaml

	4. Add swap if you don't have
		- https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04

	5. Set ssh key:
		- If the keys are indeed missing, you can create new ones using the ssh-keygen utility.Please follow the steps below:
			- First, you should make sure you're in the correct directory. Use the cd command to go to the /etc/ssh directory:
                ```
                    $ cd /etc/ssh
                ```
			- Now you can use the ssh-keygen tool to generate new host keys. You'll need to do this once for each type of key you want to generate. For example, to generate an RSA key, use the following command:
                ```
                    $ sudo ssh-keygen -t rsa -f ssh_host_rsa_key
                ```
			- This will create a new RSA key and save it in a file called ssh_host_rsa_key. The -t option specifies the type of key to create, and the -f option specifies the filename.
			You should do the same for DSA, ECDSA and ED25519 keys:
                ```
                    $ sudo ssh-keygen -t dsa -f ssh_host_dsa_key
                    $ sudo ssh-keygen -t ecdsa -f ssh_host_ecdsa_key
                    $ sudo ssh-keygen -t ed25519 -f ssh_host_ed25519_key
                ```
			- Please note that when you run these commands, you will be asked for a passphrase. You should just press Enter each time to leave the passphrase empty.
			Once you have generated the keys, you should change their permissions to make sure they are secure:
                ```
                    $ sudo chmod 600 ssh_host_*
                ```
			- This command sets the permissions so that only the owner (root, in this case) can read and write to the key files.
			- Finally, try restarting the SSH service:
                ```
                    $ sudo systemctl restart ssh
                ```

	6. Do camera settings:
		- Add "start_x=1" line and gpu_mem=256 line to end of /boot/firmware/config.txt
		then reboot

2. Software installation

	1. Install [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
        - Also install these packages
        ```
            $ sudo apt install ros-noetic-move-base* ros-noetic-dwa-local-planner* ros-noetic-joy ros-noetic-joy-dbgsym 
        ```

	2. Install ydlidar sdk
        ```
            $ cd <PATH> # PATH=/your/desired/path
            $ git clone https://github.com/YDLIDAR/YDLidar-SDK.git
            $ cd YDLidar-SDK/build
            $ cmake ..
            $ make
            $ sudo make install
        ```
    3. Clone this repo and create your ros workspace:
		
        ```
            $ cd <PATH>
            $ mkdir ydlidar_ws
            $ git clone <REPO>
            $ cp -r ./ROS_Autonomous_Robot_Real_World/src ./ydlidar_ws
            $ cd ydlidar_ws
            $ source /opt/ros/noetic/setup.bash
            $ catkin_make
            $ source ./devel/setup.bash
        ```
        - Do this on both devices

    4. Install hector_slam package to your workspace:

        ```
            $ cd src/
            $ git clone git@github.com:tu-darmstadt-ros-pkg/hector_slam.git
            $ cd ..
            $ catkin_make
            $ source ./devel/setup.bash
        ```
		- Modify x4.launch in hector_slam
        ```
            <param name="frame_id"     type="string" value="laser_frame_temp"/>
            <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser4"
        args="0.0 0.0 0.0 3.14 0.0 0.0 laser_frame laser_frame_temp 100" />
        ```
		- Modify tutorial.launch in hector_slam
        ```
    		<param name="/use_sim_time" value="false"/>
    		mapping_default.launch
        ```

		- update_factor_free: Increase the value of this parameter to make the map updates for free cells more aggressive. This will allow the map to adapt more quickly to changes in the environment and capture fine details of free space. You can try values higher than the default of 0.4, such as 0.5 or 0.6.

		- update_factor_occupied: Similarly, you can increase the value of this parameter to make the map updates for occupied cells more aggressive. This will make the map more sensitive to newly observed obstacles. You can try values higher than the default of 0.9, such as 0.95 or 1.0.

		- map_update_distance_thresh: Decrease the value of this parameter to make the map updates occur over shorter distances. This means that the algorithm will update the map more frequently as the robot moves, resulting in a faster refreshing map. You can try values lower than the default of 0.4, such as 0.3 or 0.2.

		- map_update_angle_thresh: Decrease the value of this parameter to make the map updates occur over smaller angular changes. This means that the algorithm will update the map more frequently as the robot rotates, allowing it to adapt quickly to changes in the environment. You can try values lower than the default of 0.06, such as 0.05 or 0.04.

3. Communication:
	- Set shh settings : https://phoenixnap.com/kb/ssh-permission-denied-publickey
	- Raspberry PI and host computer must be on the same network.
    - Connect the raspberry PI with
        ```
            $ ssh -Y Raspberry_PI_name@Raspberry_PI_IP
        ```
        - raspberry PI IP can be detected with
            ```
                $ sudo nmap -sn 172.19.0.0/24
            ```
    - Edit bashrc files each device
        - On your computer
            ```
                export ROS_MASTER_URI=http://<YOUR IP>:11311/
                export ROS_IP=<YOUR IP>
            ```
        - On raspberry PI
            ```
                export ROS_MASTER_URI=http://<YOUR IP>:11311/
                export ROS_IP=<Raspberry_PI IP>
            ```
    - Test communication. Run roscore on host and maybe some test publishers.

4. Assemble adept awr robot kit
	https://www.youtube.com/watch?v=-6Tur98TvYA&t=1s

5. Manuel test drive:
	- Test if everything is fine by running joystick codes. You can find how at below.

	- If any - python env source errors
		- change first line of code to "#!/usr/bin/python3.8" or the path to your python
	- If you get the error of: RuntimeError: No access to /dev/mem.  Try running as root!
		```
            $ sudo usermod -aG gpio $USER
            $ sudo chmod 666 /dev/mem
        ```
		- run these two commands and reboot



## How to Execute

1. Open raspberry PI and connect the same network with rosmaster.
2. Connect to raspberry PI with ssh:
    ```
        $ ssh -Y raspberry_name@raspberry_IP
    ```
3. Edit bashrc files as mentioned Configurations -> Communication.
4. Connect joystick to your computer if you will use manuel drive codes.
5. Open 4 terminals for raspberry and 4 terminals on the rosmaster.
    - Source and run catkin_make each.
    - Host computer:
    1. Terminal:
        ```
            $ roscore
        ```
    2. Terminal:
        - For manuel drive
        ```
            $ rosrun joy joy_node
        ```
    3. Terminal:
        - For manuel drive
        ```
            $ rosrun drive2 joy_to_cmd_vel.py
        ```
    4. Terminal:
        - For interface. We send move_base goals from here. Generated map and processed camera image are also showned.
        ```
            $ rosrun interface qt_interface.py
        ```
    - Raspberry Pi:
    1. Terminal:
        - Drive code
        ```
            $ rosrun drive2 cmd_vel.py
        ```
    2. Terminal:
        - Mapping
        ```
            $ roslaunch drive2 slam.launch
        ```
    3. Terminal:
        - Autonomy
        ```
            $ roslaunch drive2 move_base.launch
        ```
    4. Terminal:
        - Opens camera, do image processing and publish data to interface
        ```
            $ rosrun interface interface_pub.py
        ```

### Challenging Area Mapping Result
![mapping](https://github.com/Alperenlcr/ROS_Navigation_Over_the_Maze/assets/75525649/7b9e0e29-0f05-43bf-8619-33e2aa2c2ec9)

### Costmap
![imgonline-com-ua-twotoone-vOiBy8Cm9hHK6xK](https://github.com/Alperenlcr/ROS_Navigation_Over_the_Maze/assets/75525649/04760aad-e3cf-44fd-8b5d-f859a35455db)

## 🙌 Final Automation
📽️ Watch these video on <a href="https://youtu.be/eaIH36Q41yU" target="_blank">YouTube</a> for a comprehensive real-world test involving all aspects.

### Collaboration
Collaborated with [Mert Yürekli](https://github.com/mertyurekli)
Collaborated with [Ahmet Uğuz](https://github.com/ahmtuguz)