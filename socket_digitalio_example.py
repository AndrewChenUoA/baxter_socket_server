from sockaccess import sendSocket
import time
import sys

if len(sys.argv)==2:
    io_component = sys.argv[1]
else:
    io_component = "torso_right_itb_light_outer" #default
        
name = io_component
print "Initial state: " + sendSocket("digital_state "+name)

while True:
    # turn on light
    sendSocket("digital_setoutput "+name+" True")
    time.sleep(1)
    print "New state: " + sendSocket("digital_state "+name)

    # turn off light
    sendSocket("digital_setoutput "+name+" False")
    time.sleep(1)
    print "New state: " + sendSocket("digital_state "+name)
