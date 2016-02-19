# baxter_socket_server

This repository contains files that allow developers to interact and control the Baxter humanoid robot in any programming language (that can access sockets). This removes the need for the programmer to be familiar with ROS or Python.

This is done by hosting a socket server on Baxter, which can receive requests in a language agnostic manner. It converts those requests to ROSPy commands, acting as a bridge between the developer's application and Baxter.

Additionally, a script abstracting control of the arm to use inverse kinematics is also included. Example scripts of how to access the server are also included.

##To use:

ssh into Baxter, and copy the socketserver.py file onto Baxter. If you want to use the limbs or sonar, you should also copy baxter_arm_control.py and sonar_interface.py

Run the socket_server using python socketserver.py. Some initialisation sequence is executed to ensure that the arms and cameras are in a usable state. The server is then started on port 10111.

Use Ctrl+\ to crash out of the server.

##Supported Objects:

Analog: left_hand_range, right_hand_range, left_itb_wheel, right_itb_wheel, torso_left_itb_wheel, torso_right_itb_wheel, torso_fan

Digital: left_itb_light_inner, right_itb_light_inner, torso_left_itb_light_inner, torso_right_itb_light_inner, left_itb_light_outer, right_itb_light_outer, torso_left_itb_light_outer, torso_right_itb_light_outer

Arms: left_limb, right_limb

Cameras: left_camera, right_camera, head_camera

Others: sonar, head

##Functions:

####analog_state(object_name):

  returns state or error code
  
####analog_setoutput(object_name, value, timeout (optional))

  returns "Successful" or error code
  
  
####digital_state(object_name):

  returns state or error code
  
####digital_setoutput(object_name, value, timeout (optional)):

  returns "Successful" or error code
  
  
####limb_calibrategripper(object_name):

  Closes and opens the gripper
  
  returns "Successful" or error code
  
####limb_moveto(object_name, px, py, pz, pr, pp, pya):

  Moves the limb to the specified endpoint in the format {x, y, z, roll, pitch, yaw}
  
  returns "Successful" or error code
  
####limb_getpose(object_name):

  returns current endpoint position in {x, y, z, roll, pitch, yaw} format or error code
  
  
  
####sonar_getdata(object_name):

  returns current sonar array values or error code
  
  
  
####head_getpan(object_name):

  returns current head angle or error code
  
####head_setpan(object_name, angle, speed (optional)):

  returns "Successful" or error code
  
####head_nod(object_name, timeout(optional)):

  Makes Baxter's head move down and back up
  
  returns "Successful" or error code
  
  
  
####set_camera(object_name, exposure, gain, blue, green, red, width, height):

  All arguments other than the object_name are optional
  
  returns "Successful" or error code
  
####camera_open(object_name):

  Attempts to enable the specified camera
  
  returns "Successful" or error code
  
####camera_getimage(object_name):

  returns the current frame from that camera as a string or error code
  
####camera_close(object_name):

  Attempts to disable the specified camera
  
  returns "Successful" or error code
  
  
  
##Error codes:

  -1: "Successful"
  
  0 : "Invalid component Type"
  
  1 : "Invalid io_component Type"
  
  2 : "Invalid name"
  
  3 : "Invalid value"
  
  4 : "No valid path to arm position"
  
  9 : "ROS Master is unavailable"
  
