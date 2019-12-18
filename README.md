# Sparki Tower Defense
Our final project for CSCI 3302 - Intoduction to Robotics.

### Setup Project
##### Unity
1) Install [Unity3D](https://unity.com/) Editor version 2018.4 LTS
2) Add "Android Build Support" and "Vuforia Augmented Reality Support" modules
3) Open the "Unity Project" folder in Unity
##### ROS
1) Install [ROS Melodic](http://wiki.ros.org/melodic)
2) Install [Rosbridge](http://wiki.ros.org/rosbridge_suite)
3) Start Roscore (`roscore`)
4) Start Rosbridge (`roslaunch rosbridge_server rosbridge_websocket.launch`)
5) Start Sparki-ros (`python2 ROSWorkspace/src/sparki-ros.py /dev/ttyACM0`)

### Run Game
1) Press the play button in Unity
1) Enter the IP address of the machine running Roscore and press "Connect"
2) Run main.py (`python2 ROSWorkspace/src/sparki-ros.py`)

### Run Unit Tests
1) Run the command `python2 -m ROSWorkspace.test.<testname>` to run unit tests.
   - For example, to unit test `world.py`, run `python2 -m ROSWorkspace.test.test-world`
