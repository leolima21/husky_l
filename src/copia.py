<?xml version="1.0"?>

<launch>

  <param name="use_sim_time" value="true"/>
  <arg name="scan_topic" default="scan" />
  
  <!--- Run gmapping -->
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping">
    <rosparam>
      odom_frame: odom
      base_frame: base_link
      map_frame: map

      map_update_interval: 0.5 # Publish new map

      maxUrange: 30 # Should be just less than sensor range
      maxRange: 60 # Should be just greater than sensor range
      particles: 40 # Increased from 80

      # Update frequencies
      linearUpdate: 0.1 #0.3
      angularUpdate: 0.1 #0.5
      temporalUpdate: 2.0
      resampleThreshold: 0.5

      # Initial Map Size
      xmin: -80.0
      ymin: -80.0
      xmax: 80.0
      ymax: 80.0
      delta: 0.05

      # All default
      sigma: 0.05
      kernelSize: 1
      lstep: 0.05
      astep: 0.05
      iterations: 5
      lsigma: 0.075
      ogain: 3.0
      lskip: 0 #10
      llsamplerange: 0.01
      llsamplestep: 0.01
      lasamplerange: 0.005
      lasamplestep: 0.005

    </rosparam>
    <remap from="scan" to="$(arg scan_topic)"/>
  </node>


  <!--- Run Move Base -->
  <include file="$(find husky_navigation)/launch/move_base.launch" />

  <!-- <node pkg="explore_lite" type="explore" respawn="false" name="explore" output="screen">
    <param name="robot_base_frame" value="base_link"/>
    <param name="costmap_topic" value="map"/>
    <param name="costmap_updates_topic" value="map_updates"/>
    <param name="visualize" value="true"/>
    <param name="planner_frequency" value="0.33"/> <! #0.33 > 
    <param name="progress_timeout" value="1"/> <! 30 >
    <param name="potential_scale" value="3.0"/> <! 3 >
    <param name="orientation_scale" value="0.0"/> <!0>
    <param name="gain_scale" value="1.0"/> <!1.0>
    <param name="transform_tolerance" value="0.3"/> <!0.3>
    <param name="min_frontier_size" value="5"/> <!0.5>
  </node> -->

  <rosparam file="$(find nav2d_tutorials)/param/ros.yaml"/>

  <node name="Operator" pkg="nav2d_operator" type="operator" >
		<remap from="scan" to="scan"/>
		<rosparam file="$(find nav2d_tutorials)/param/operator.yaml"/>
		<rosparam file="$(find nav2d_tutorials)/param/costmap.yaml" ns="local_map" />
	</node> -->

  <!-- <node name="Mapper" pkg="nav2d_karto" type="mapper">
		<remap from="scan" to="scan"/>
		<rosparam file="$(find nav2d_tutorials)/param/mapper.yaml"/>
	</node> -->

  <node name="Navigator" pkg="nav2d_navigator" type="navigator">
		<rosparam file="$(find nav2d_tutorials)/param/navigator.yaml"/>
	</node>
 


  

  <node name="GetMap" pkg="nav2d_navigator" type="get_map_client" />
  <node name="Explore" pkg="nav2d_navigator" type="explore_client" />
  <node name="SetGoal" pkg="nav2d_navigator" type="set_goal_client" /> 

</launch>