#!/usr/bin/env python

import sys
import math

def createLaunchFile():
    sideLength = int(math.sqrt(nBots)/2)+1
    botPoses = [(i,j) for i in range(-sideLength,sideLength) for j in range(-sideLength,sideLength)]
    fileContent = '''<?xml version="1.0"?>

<launch>
 <arg name="paused" default="false"/>
 <arg name="use_sim_time" default="true"/>
 <arg name="gui" default="true"/>
 <arg name="headless" default="false"/>
 <arg name="debug" default="false"/>
 <arg name="model" default="$(find hector_quadrotor_description)/urdf/quadrotor.gazebo.xacro" />
 <include file="$(find gazebo_ros)/launch/empty_world.launch">
  <arg name="paused" value="$(arg paused)"/>
  <arg name="use_sim_time" value="$(arg use_sim_time)"/>
  <arg name="gui" value="$(arg gui)"/>
  <arg name="headless" value="$(arg headless)"/>
  <arg name="debug" value="$(arg debug)"/>
 </include>
  \n
  '''
    for i in range(1,nBots+1):
        robot = '''<!-- BEGIN ROBOT {0}-->
      <group ns="robot{0}">
       <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch">
       <arg name="name" value="Robot{0}" />
       <arg name="tf_prefix" value="robot{0}" />
       <arg name="model" value="$(arg model)" />
       <arg name="x" value="{1}"/>
       <arg name="y" value="{2}" />
       </include>
      </group>
      \n'''.format(str(i),str(botPoses[i][0]),str(botPoses[i][1]))
        fileContent = fileContent + robot
    fileContent = fileContent+'''</launch>'''
    launchFile = open('multi_quadrotor_world{}.launch'.format(str(nBots)),'w')
    launchFile.write(fileContent)
    launchFile.close()

if __name__ == '__main__':
    try:
	nBots = int(sys.argv[1])
    except ValueError:
	print "The 'nBots' parameteter is required. \n Try '***.py nBots'"
	sys.exit()
    createLaunchFile()
