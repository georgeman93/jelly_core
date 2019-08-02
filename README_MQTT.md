# jelly_core

I turned jelly_core into a catkin workspace so that I could easily build all the packages inside and usee roscd, roslaunch etc

I cloned in https://github.com/groove-x/mqtt_bridge and added as a package with the rest. It's use requires 
`pip -r requirements` to install the python dependancies in your environment as well as running the setup.py script to prepare the the python modules fo the catkin build system. http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html

But I've updated the requirements file on account of this error 'BSON installation does not support all necessary features. Please use the MongoDB BSON implementation'.

Then I have run catkin_make again in jelly_core to install the python files at the package level. You will need to do the same.

I edited the jelly_gui.launch file to swap the rosbridge with the mqtt-ros bridge. Btw the rosbridge package is a dependancy of the mqtt_bridge so keep it on your system.

I altered the mqtt_params.yaml file with the correct topics and server details.


# jelly_web

I added the front end paho mqtt client (using mqtt websockets back to a mosquitto broker running on the server) script reference into the index file. The license requires it to be externally referenced otherwise you must copyleft your project.

I created a script mqtt_client.js to deal with the web_page mqtt initialization.

I stripped out the rosbridge parts of script.js that handle command/mode/status/calibration and replaced them with mqtt code.

I have added mosquitto_setup.bash to detail the mosquitto broker setup on the server. 

The jelly robot and web page will communicate (command/mode/status/calibration messages) with the mosquitto broker in the middle.


# tf viewer

My Ubuntu updated Firefox to 68.0.1 causing errors with openGL preventing the 3D viewer from appearing.
Red hat people confirm that WebGL works ok after downgrade to firefox-59.0.2-1.fc28.x86_64 (no other packages changed). Thought you should know

I have implemented a very hacky workaround to get the 3D viewer working.

I would have liked to pass the /tf messages straight through the mqtt_bridge to the paho-mqtt client to an adapted ROSLIB.TFClient that takes in /tf data taken from a js object. But this is potentially a lot of work. 

My solution is to install ROS on the web server and implement another ros-mqtt bridge to get /tf messages from the jelly robot over mqtt but pass them onto the webpage via rosbridge websockets. 

The webpage sets up a ROS websocket connection back to the server (with its static hostname/ipaddress, also needed for the mqtt_bridge config) which is running a mqtt_bridge node as well as a tf2_web_publisher node 

To test this, I used 1 ubuntu VM for the webserver another for the Jelly robot and viewed the webpage from my MacOS. It worked well.

You might notice with this implementation that I could have left script.js almost intact with all messages coming in via a ros websocket connection back to the server. But hopefully in the future someone makes an mqtt adapter for ROSLIB.TFClient and it gets completely untethered from rosbridge.

I












