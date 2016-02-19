#!/usr/bin/env python
__author__ = 'Andrew Chen'
#Requires ROS Indigo and Baxter SDK 1.1+

import rospy
from sensor_msgs.msg import PointCloud

_NUM_SENSORS = 12

class sonar():
    def __init__(self):

        # subscribe to sonar data
        self.sub = rospy.Subscriber('/robot/sonar/head_sonar/state', PointCloud, self.store_sub)
        print "Channel Initialised"
        self.sub_sensors = [] #Initial State
        self.sub_distances = [] #Initial State
        self.sonar_array = [None]*_NUM_SENSORS #Initial State
        self.rate = rospy.Rate(100)

    #Callback for storing received data
    def store_sub(self, data):
        channels = data.channels
        #Convert channels into a list of active sensors and a list of distances seen by those sensors
        sensors = str(channels[0])[:-1].split("[")[1].split(",")
        self.sub_sensors = [r for r in (int(float(i)) if i!="" else None for i in sensors) if r is not None]
        #print self.sub_sensors
        distances = str(channels[1])[:-1].split("[")[1].split(",")
        #The original float is actually very "precise" but the extra precision is unnecessary so just save 2dp
        self.sub_distances = [r for r in (round(float(i),2) if i != "" else None for i in distances) if r is not None]
        #print self.sub_distances
        for idx, prev_dist in enumerate(self.sonar_array):
            if idx in self.sub_sensors:
                self.sonar_array[idx] = self.sub_distances[self.sub_sensors.index(idx)]
            else:
                #Ensure that we reset and don't hold onto old values indefinitely
                self.sonar_array[idx] = None  

    def get_subsensors(self):
        return self.sub_sensors
        
    def get_subdistances(self):
        return self.sub_distances
        
    def get_sonararray(self):
        return self.sonar_array
        
    def sleep(self):
        self.rate.sleep()
