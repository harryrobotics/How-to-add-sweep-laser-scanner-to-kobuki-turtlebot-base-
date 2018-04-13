# How to add sweep laser scanner to kobuki turtlebot base

Modify based on http://wiki.ros.org/turtlebot/Tutorials/hydro/Adding%20a%20Hokuyo%20laser%20to%20your%20Turtlebot . I try to look for the tutorial to add sweep laser scanner but fail to do so. Therefore I refered to this article and do some tweeks to make it work. The article is created quite long time ago, hence some folders and filename have changed. You should be aware of that. Have fun !

Firstly, we need to do some system configuration so that the system can read data from Lidar:
Setup udev:
```
$ sudo echo 'KERNEL=="ttyUSB[0-9]*", MODE="0777"' >> /etc/udev/rules.d/50-laser.rules
```
Now plug in your laser. Check it's file permissions. They should be read/write/execute for all. The ls output should look similar to this:
```
$ ls -al /dev/ttyUSB0 
crw-rw-rw- 1 root dialout 166, 0 May 11 23:36 /dev/ttyUSB0
```

## 1 .Edit your turtlebot description
First, ```roscd``` to ```turtlebot_description``` folder. *Note: Depends on your base, you might need to edit the robot model urdf file at a difference folder.

```
roscd turtlebot_description
```
Edit ```./urdf/turtlebot_library.urdf.xacro```

*Note: Depends on your base, you might need to edit the robot model urdf file at difference folder. For example, my base is kobuki so I go to:
```
roscd kobuki_description
```
And then modify the file : ```./urdf/turtlebot_library.urdf.xacro```

You need to add the laser to the robot model. Add the following right above the </robot> tag

```
  <joint name="laser" type="fixed">
    <origin xyz="-0.005 0.00 0.43" rpy="3.14159 0 0" />
    <parent link="base_link" />
    <child link="base_laser_link" />
  </joint>

  <link name="base_laser_link">
    <visual>
      <geometry>
        <box size="0.00 0.06 0.06" />
      </geometry>
      <material name="Green" />
    </visual>
    <inertial>
      <mass value="0.000001" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
        iyy="0.0001" iyz="0.0"
        izz="0.0001" />
    </inertial>
  </link>
 
 ```
## 2. Edit ```3dsensor.launch``` - If you dont have a kinect sensor - skip this step
The stock files will have your kinect's virutal laser scan publishing on the same topic that other tutorials want to use, and I would rather the laser data would come from the hokuyo laser. This can be prevented by changing the topic that the Kinect laser publishes on with the following:
```
roscd to turtlebot_bringup/launch/
```
roscd turtlebot_bringup/launch/
edit 3dsensor.launch

Look for the line that looks like this:

```
  <arg name="scan_topic" default="scan"/>
```  
Remove it and replace it with:
```
  <arg name="scan_topic" default="kinect_scan"/>
``` 
Save that file.

## 3 .Edit ```minimal.launch``` 
We now need to add the hokuyo node to it.

At the bottom of the file, right before </launch>, add the following:
```
    <!-- run sweep_node node -->
    <node name="sweep_node"          pkg="sweep_ros"  type="sweep_node" output="screen">
            <param name="serial_port"         type="string" value="/dev/ttyUSB0"/>
            <param name="serial_baudrate"     type="int"    value="115200"/>
            <param name="frame_id"            type="string" value="base_laser_link"/>
            <param name="rotation_speed"      type="int" value="5"/>
    </node>

    <!-- run pointcloud_to_laserscan node -->
    <node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">

        <remap from="cloud_in" to="pc2"/>
        <rosparam>
            target_frame: base_laser_link
            transform_tolerance: 0.001
            min_height: -1.0
            max_height: 1.0

            angle_min: -3.14 # -M_PI/2
            angle_max: 3.14 # M_PI/2
            angle_increment: 0.01745 # 2*M_PI/360.0
            scan_time: 0.1
            range_min: 0.0
            range_max: 40.0
            use_inf: true

            # Concurrency level, affects number of pointclouds queued for processing and number of threads used
            # 0 : Detect number of cores
            # 1 : Single threaded
            # 2->inf : Parallelism level
            concurrency_level: 1
        </rosparam>

    </node>
```
The edit to minimal.launch should bring up the sweep node when you launch it.

## 4. Install ```sweep-ros wrapper```, ```sweep-sdk```
The sweep node can only come up if it's on your system. Make sure you have it installed by running:

###Todo : Add how to install sweep-node here

## 5. Bringup your new config
Assuming that it all worked, you should be able to bring up your new enviroment.

First, ensure that your ROS_MASTER_URI and ROS_HOSTNAME are correct.

If you are unsure what your master URI and ros hostname are suposed to be, please see network configuration

Bringup the new turtlebot config. In one terminal, run:

```
roslaunch turtlebot_bringup minimal.launch
```
Note that you should see messages about your laser driver starting up with messages like:
```
process[sweep-node]: started with pid [8288]
```
In another terminal, you can bring up the kinect sensor by running:

```
roslaunch turtlebot_bringup 3dsensor.launch
```
## 6.
Check your results with rviz
Now, run rviz : ```oslaunch turtlebot_rviz_launchers view_robot.launch```
You should now have a block that appears on your turtlebot model that looks something like this:

![alt text](http://wiki.ros.org/turtlebot/Tutorials/hydro/Adding%20a%20Hokuyo%20laser%20to%20your%20Turtlebot?action=AttachFile&do=get&target=scanner-in-model.png)

Add two LaserScan visuilizations. Subscribe one of them to the /kinect_scan topic and set it's Color to "2; 255; 255"

Like this:

![alt text](http://wiki.ros.org/turtlebot/Tutorials/hydro/Adding%20a%20Hokuyo%20laser%20to%20your%20Turtlebot?action=AttachFile&do=get&target=Kinect_scan_topic.png)

Subscribe the other topic to /scan. Leave it's Color "255; 255; 255"

Like this:

![alt text](http://wiki.ros.org/turtlebot/Tutorials/hydro/Adding%20a%20Hokuyo%20laser%20to%20your%20Turtlebot?action=AttachFile&do=get&target=scan_topic.png)
