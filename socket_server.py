#!/usr/bin/python
""" socket_server.py

    Author: Andrew Chen (andrew.chen@auckland.ac.nz)
    Last Edited: 03/07/2015

    This webserver acts as a bridge between the Python baxter-interface
    and a program written in any language, interfacing via Sockets
    
    DEPENDENCIES:
    ROS Indigo
    Baxter SDK (http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)
    baxter_arm_control and sonar_interface
    moveit-ikfast (preferably indigo) (sudo apt-get install ros-indigo-moveit-ikfast)
    Set up the Raspberry Pi using https://wiki.ubuntu.com/ARM/RaspberryPi
"""

import SocketServer
import socket
import sys
import threading
import rospy
import baxter_interface
import baxter_arm_control #Could be in baxter_interface
import sonar_interface #Could be in baxter_interface
from sensor_msgs.msg import Image

errorCodes = {  -1: "Successful",
                0 : "Invalid component Type",
                1 : "Invalid io_component Type",
                2 : "Invalid name",
                3 : "Invalid value",
                4 : "No valid path to arm position",
                9 : "ROS Master is unavailable"}

def getError(num):
    try:
        return errorCodes[num]
    except KeyError:
        return "Other Error"

#Global Object Dictionary
#key:value = name:object (pointer)
_OBJECT_DICTIONARY = {}

def checkDictionary(name):
    try:
        x = _OBJECT_DICTIONARY[name]
        return True
    except KeyError:
        return False

########  REGISTRATION OF ALL OBJECTS  ########
def registerObjects():
    """io_component Types:
    -analog_io
        -left_hand_range
        -right_hand_range
        -left_itb_wheel
        -right_itb_wheel
        -torso_left_itb_wheel
        -torso_right_itb_wheel
        -left_vacuum_sensor_analog
        -right_vacuum_sensor_analog
        -torso_fan
    -digital_io
        -left_itb_light_inner
        -right_itb_light_inner
        -torso_left_itb_light_inner
        -torso_right_itb_light_inner
        -left_itb_light_outer
        -right_itb_light_outer
        -torso_left_itb_light_outer
        -torso_right_itb_light_outer
    -limb
        -left_limb
        -right_limb
    -sonar
    -head
    -camera
        -left_camera
        -right_camera
        -head_camera
    """
    try:
        #Register all components in the dictionary
        _OBJECT_DICTIONARY["left_hand_range"] = baxter_interface.analog_io.AnalogIO("left_hand_range")
        _OBJECT_DICTIONARY["right_hand_range"] = baxter_interface.analog_io.AnalogIO("right_hand_range")
        _OBJECT_DICTIONARY["left_itb_wheel"] = baxter_interface.analog_io.AnalogIO("left_itb_wheel")
        _OBJECT_DICTIONARY["right_itb_wheel"] = baxter_interface.analog_io.AnalogIO("right_itb_wheel")    
        _OBJECT_DICTIONARY["torso_left_itb_wheel"] = baxter_interface.analog_io.AnalogIO("torso_left_itb_wheel")
        _OBJECT_DICTIONARY["torso_right_itb_wheel"] = baxter_interface.analog_io.AnalogIO("torso_right_itb_wheel")
        _OBJECT_DICTIONARY["torso_fan"] = baxter_interface.analog_io.AnalogIO("torso_fan")
        _OBJECT_DICTIONARY["left_itb_light_inner"] = baxter_interface.digital_io.DigitalIO("left_itb_light_inner")
        _OBJECT_DICTIONARY["right_itb_light_inner"] = baxter_interface.digital_io.DigitalIO("right_itb_light_inner")
        _OBJECT_DICTIONARY["torso_left_itb_light_inner"] = baxter_interface.digital_io.DigitalIO("torso_left_itb_light_inner")
        _OBJECT_DICTIONARY["torso_right_itb_light_inner"] = baxter_interface.digital_io.DigitalIO("torso_right_itb_light_inner")
        _OBJECT_DICTIONARY["left_itb_light_outer"] = baxter_interface.digital_io.DigitalIO("left_itb_light_outer")
        _OBJECT_DICTIONARY["right_itb_light_outer"] = baxter_interface.digital_io.DigitalIO("right_itb_light_outer")
        _OBJECT_DICTIONARY["torso_left_itb_light_outer"] = baxter_interface.digital_io.DigitalIO("torso_left_itb_light_outer")
        _OBJECT_DICTIONARY["torso_right_itb_light_outer"] = baxter_interface.digital_io.DigitalIO("torso_right_itb_light_outer")
        _OBJECT_DICTIONARY["left_limb"] = baxter_arm_control.arm("left")
        _OBJECT_DICTIONARY["right_limb"] = baxter_arm_control.arm("right")    
        _OBJECT_DICTIONARY["sonar"] = sonar_interface.sonar()
        _OBJECT_DICTIONARY["head"] = baxter_interface.Head()        
        #Only two cameras can be only at any time, so we need to close the cameras (and open again when we need them)
        try:
            _OBJECT_DICTIONARY["left_camera"] = baxter_interface.camera.CameraController("left_hand_camera")
            _OBJECT_DICTIONARY["left_camera"].close()
            _OBJECT_DICTIONARY["right_camera"] = baxter_interface.camera.CameraController("right_hand_camera")
            _OBJECT_DICTIONARY["right_camera"].close()
        except AttributeError:
            _OBJECT_DICTIONARY["right_camera"] = baxter_interface.camera.CameraController("right_hand_camera")
            _OBJECT_DICTIONARY["right_camera"].close()
            _OBJECT_DICTIONARY["left_camera"] = baxter_interface.camera.CameraController("left_hand_camera")
            _OBJECT_DICTIONARY["left_camera"].close()
        _OBJECT_DICTIONARY["head_camera"] = baxter_interface.camera.CameraController("head_camera")
        _OBJECT_DICTIONARY["head_camera"].close()
                    
    except ValueError,e:
        if e.args[0] == "signal only works in main thread":
            print "ERROR: "+getError(9)
        else:
            print "ERROR: "+getError(999)

########  ANALOG I/O FUNCTIONS  ########
def analog_state(name):
    if checkDictionary(name):
        return unicode(_OBJECT_DICTIONARY[name].state())
    else:
        return "ERROR: "+getError(2)

def analog_setoutput(name, value, timeout=2.0):
    if checkDictionary(name):    
        try:
            value = int(value)
        except ValueError:
            return "ERROR: "+getError(3)
        if value in range(0,101):
            _OBJECT_DICTIONARY[name].set_output(value, float(timeout))
            return getError(-1)
        else:
            return "ERROR: "+getError(3)
    else:
        return "ERROR: "+getError(2)

########  DIGITAL I/O FUNCTIONS  ######## 
def digital_state(name):
    if checkDictionary(name):
        return unicode(_OBJECT_DICTIONARY[name].state)
    else:
        return "ERROR: "+getError(2)
    
def digital_setoutput(name, value, timeout=2.0):
    if checkDictionary(name):
        value = value.lower()
        if value in ["true", "false", True, False]:
            _OBJECT_DICTIONARY[name].set_output(True if value=="true" else False, float(timeout))
            return getError(-1)
        else:
            return "ERROR: "+getError(3)
    else:
        return "ERROR: "+getError(2)    
                
########  LIMB FUNCTIONS  ########
def limb_calibrategripper(name):
    if checkDictionary(name):    
        _OBJECT_DICTIONARY[name].calibrateGripper()
        return getError(-1)
    else:
        return "ERROR: "+getError(2)

#Open or Close the gripper    
def limb_gripper(name, value):
    if checkDictionary(name):
        value = value.lower()
        if value in ["open", "1"]:
            _OBJECT_DICTIONARY[name].gripper.open()
        elif value in ["close", "0"]:
            _OBJECT_DICTIONARY[name].gripper.close()
        else:
            return "ERROR: "+getError(3)
        return getError(-1)
    else:
        return "ERROR: "+getError(2)
        
def limb_getownstate(name):
    if checkDictionary(name):
        return unicode(_OBJECT_DICTIONARY[name].get_ownstate())
    else:
        return "ERROR: "+getError(2)
    
#Send the state number to the other arm
#Blocking synchro must be implemented in control program
#(i.e. poll and check the value of getSynchro)
def limb_sendSynchro(name, state):
    if checkDictionary(name):
        _OBJECT_DICTIONARY[name].sendSynchro(state)
        return getError(-1)
    else:
        return "ERROR: "+getError(2)
        
#Get the current state number reported by the other arm
def limb_getSynchro(name):
    if checkDictionary(name):
        return unicode(_OBJECT_DICTIONARY[name].get_sub())
    else:
        return "ERROR: "+getError(2)        

#Set the arm pose
#px = x co-ordinate px = y co-ordinate, pz = z co-ordinate
#pr = roll, pp = pitch, pya = yaw (all in radians)
def limb_moveto(name, px, py, pz, pr, pp, pya):    
    if checkDictionary(name):
        try:
            result = _OBJECT_DICTIONARY[name].baxter_ik_move([float(px), float(py), float(pz), float(pr), float(pp), float(pya)])
            if result>0:
                return "ERROR: "+getError(4)
            else:
                return getError(-1)
        except ValueError:
            return "ERROR: "+getError(3)
    else:
        return "ERROR: "+getError(2)
        
#Get arm pose
def limb_getpose(name):
    if checkDictionary(name):
        return str(_OBJECT_DICTIONARY[name].getPose())
    else:
        return "ERROR: "+getError(2)

########  SONAR FUNCTIONS  ########
def sonar_getdata(name):
    if checkDictionary(name):
        return unicode(_OBJECT_DICTIONARY[name].get_sonararray())
    else:
        return "ERROR: "+getError(2)
        
########  HEAD FUNCTIONS ########
def head_getpan(name):
    if checkDictionary(name):
        return unicode(_OBJECT_DICTIONARY[name].pan())
    else:
        return "ERROR: "+getError(2)    
        
def head_setpan(name, angle, speed=100.0):
    if checkDictionary(name):
        try:
            _OBJECT_DICTIONARY[name].set_pan(float(angle), float(speed))
            return getError(-1)
        except ValueError:
            return "ERROR: "+getError(3)
    else:
        return "ERROR: "+getError(2)  
        
def head_nod(name, timeout=0.0):
    if checkDictionary(name):
        _OBJECT_DICTIONARY[name].command_nod()
        return getError(-1)
    else:
        return "ERROR: "+getError(2)     
        
########  CAMERA FUNCTIONS ########
def set_camera(name, exposure=-1, gain=-1, blue=-1, green=-1, red=-1, width=320, height=200):
    if checkDictionary(name):
        _OBJECT_DICTIONARY[name].exposure = exposure
        _OBJECT_DICTIONARY[name].gain = gain
        _OBJECT_DICTIONARY[name].white_balance_blue = blue
        _OBJECT_DICTIONARY[name].white_balance_green = green
        _OBJECT_DICTIONARY[name].white_balance_red = red
        _OBJECT_DICTIONARY[name].resolution = (width, height)
        return getError(-1)
    else:
        return "ERROR: "+getError(2)  

def camera_open(name):
    if checkDictionary(name):
        _OBJECT_DICTIONARY[name].open()
        if name == "left_camera":
            left_img_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, left_cam_callback)
        elif name == "right_camera":
            right_img_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image, right_cam_callback)        
        else:
            head_img_sub = rospy.Subscriber("/cameras/torso_camera/image", Image, head_cam_callback)
        return getError(-1)
    else:
        return "ERROR: "+getError(2)  

camera_images = {"left_camera": None, "right_camera": None, "head_camera": None}        
def left_cam_callback(data):
    camera_images['left_camera'] = data.data
def right_cam_callback(data):
    camera_images['right_camera'] = data.data
def head_cam_callback(data):
    camera_images['head_camera'] = data.data
            
def camera_close(name):
    if checkDictionary(name):
        _OBJECT_DICTIONARY[name].close()
        return getError(-1)
    else:
        return "ERROR: "+getError(2)
        
def camera_getimage(name):
    if checkDictionary(name):
        return str(camera_images[name])
    else:
        return "ERROR: "+getError(2)


########  SOCKET SERVER SETUP  ########
class SocketHandler(SocketServer.BaseRequestHandler):     

    def handle(self):
        #self.request is the TCP socket connected to the client
        self.data = self.request.recv(4096).strip()
        print unicode(self.client_address[0]) + ": " + self.data
        
        #Determine what the command actually was
        command = self.data.encode('ascii','ignore') #Get rid of weird unicode characters coming from Java
        command = command.split(' ')
        
        result = globals()[command[0]](*command[1:]) #Using * unpacks the list into *args
        self.request.sendall(result)
        
class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass
        
if __name__ == "__main__":       
    print "========================="
    print "University of Auckland"
    print "Baxter Interface Bridge Server (Sockets Edition)"
    print "========================================"    

    #Check if another instance is already runnning
    try:
        # Start the web server
        HOST, PORT = "0.0.0.0", 10111
        server = ThreadedTCPServer((HOST, PORT), SocketHandler)    
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
        server_thread.start()
        print "Socket Server Created at " + server.server_address[0] + ":" + str(PORT)
    except socket.error:
        sys.exit(-1)                   
    
    #Initialise the System
    import subprocess
    subprocess.call(["rosrun", "baxter_tools", "enable_robot.py", "-e"])
    subprocess.call(["rosrun", "baxter_tools", "tuck_arms.py", "-u"])
    
    #Register a node with the ROS Master
    rospy.init_node('socket_server')  
    
    registerObjects()
    limb_calibrategripper("left_limb")
    limb_calibrategripper("right_limb")
    
    print "System ready"
    
    while(True):
        pass
