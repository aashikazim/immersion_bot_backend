udp receiver

-> commands: 
go to goal (input: x, y) | on complete, send acknowledgement
manual control (input: linearx, angularz)
stop (no input)
pursuit task (input: x, y, r, speed)

config (this function can set robot parameters --> max velocity, max angular velocity)


# Go To Goal
{
    command: "GO_TO_GOAL",
    argument: {
        x: x_val,
        y: y_val
    }
}

# Pursuit
{
    command: "PURSUIT",
    argument: {
        x: x_val,
        y: y_val,
        r: r_val,
        speed: speed_val
    }
}


+++++++++++++++
commands:

roslaunch husarion_ros rosbot_drivers.launch           

To create a map:
rosrun gmapping slam_gmapping
roslaunch als_ros mcl.launch

OR ==> rosrun amcl amcl


To save the map:
rosrun map_server map_saver -f /home/husarion/samplemap.pgm

To load previously created map:
rosrun map_server map_server /home/husarion/samplemap.yaml
roslaunch als_ros mcl.launch




rosrun immersion_bot pose_sync.py

rosrun teleop_twist_keyboard teleop_twist_keyboard.py            



roslaunch husarion_ros rosbot_drivers.launch  
rosrun map_server map_server /home/husarion/samplemap.yaml
roslaunch als_ros mcl.launch
rosrun immersion_bot pose_sync.py


<launch>
    <!-- Launch ROSBOT drivers -->
    <include file="$(find husarion_ros)/launch/rosbot_drivers.launch"/>

    <!-- Wait for the drivers to be up and running -->
    <group if="$(arg wait_for_drivers)">
        <param name="/use_sim_time" value="true"/>
        <rosparam param="/use_sim_time">true</rosparam>
        <node pkg="roscpp" type="wait_for_time" name="wait_for_time" />
    </group>

    <!-- Run the map server -->
    <node name="map_server" pkg="map_server" type="map_server" args="/home/husarion/samplemap.yaml"/>

    <!-- Launch MCL -->
    <include file="$(find als_ros)/launch/mcl.launch"/>

    <!-- Run custom pose synchronization script -->
    <node name="pose_sync" pkg="immersion_bot" type="pose_sync.py"/>
</launch>


roslaunch [your_package_name] combined_launch.launch wait_for_drivers:=true
