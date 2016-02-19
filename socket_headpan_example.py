from sockaccess import sendSocket
import time
from math import pi as PI
       
name = "head"

while(True):
    sendSocket("head_setpan "+name+" "+str(0)+" "+str(10.0))
    sendSocket("head_nod "+name)
    time.sleep(1)
    sendSocket("head_setpan "+name+" "+str(-PI/2)+" "+str(10.0))
    time.sleep(1)
    sendSocket("head_setpan "+name+" "+str(0)+" "+str(10.0))
    sendSocket("head_nod "+name)
    time.sleep(1)
    sendSocket("head_setpan "+name+" "+str(PI/2)+" "+str(10.0))
    time.sleep(1)
    
