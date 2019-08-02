# jelly_core updates

I have implemented the mqtt_bridge package to send mode/status/command/calibrate messages to and from the webpage and tf messages to the webserver. The webserver hosts a mosquitto mqtt broker which the jelly robot and webpage use to talk.

I turned jelly_core into a catkin workspace so that I could easily build all the packages inside and usee roscd, roslaunch etc

I cloned in https://github.com/groove-x/mqtt_bridge and added as a package with the rest. It's use requires 
`pip -r requirements` to install the python dependancies in your environment as well as running the setup.py script to prepare the the python modules fo the catkin build system. http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html

But I've updated the requirements file on account of the following error 'BSON installation does not support all necessary features. Please use the MongoDB BSON implementation'.

Then I have run catkin_make again in jelly_core to install the python files at the package level. You will need to do the same.

I edited the jelly_gui.launch file to swap the rosbridge with the mqtt-ros bridge. Btw the rosbridge package is a dependancy of the mqtt_bridge so keep it on your system.

I altered the mqtt_params.yaml file with the correct topics and server details.

Use the same launch files to bringup the jelly robot. I deleted some duplicate code in these.














