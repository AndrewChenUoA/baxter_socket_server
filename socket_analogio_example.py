from sockaccess import sendSocket
import time
import sys

if len(sys.argv)==2:
    io_component = sys.argv[1]
else:
    io_component = "torso_fan" #default
    
name = io_component
print sendSocket("analog_state "+name)

while True:
    for i in range(0, 101, 1):
        sendSocket("analog_setoutput "+name+" "+str(i))
        print i
        time.sleep(0.1)
    time.sleep(2)
    for i in range(100, -1, -1):
        sendSocket("analog_setoutput "+name+" "+str(i))
        print i
        time.sleep(0.1)
    time.sleep(2)
