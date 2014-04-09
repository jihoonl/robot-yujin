#!/usr/bin/env python

import roslib
roslib.load_manifest('turtlebot_teleop')
import rospy
import time

import sys, select, termios, tty

from std_msgs.msg import String

import unique_id
import scheduler_msgs.msg as scheduler_msgs
from geometry_msgs.msg import Twist
import BPEL_Adapter.msg as adapter_msgs


resource_alloc_pub_topic = '/services/adapter/resources_alloc_request'
resource_alloc_sub_topic = '/services/adapter/resources_alloc_reply'

command_pub_topic = '/services/adapter/command_request'
command_sub_topic = '/services/adapter/command_reply'

class AdapterUtility:
    def __init__(self):

        rospy.init_node('ssel_node', anonymous=True)

        self.resource_alloc_pub = rospy.Publisher(resource_alloc_pub_topic, adapter_msgs.Adapter)
        self.resource_alloc_sub = rospy.Subscriber(resource_alloc_sub_topic, String, self.resource_alloc_callback)
        self.resource_alloc_flag = False

        self.command_pub = rospy.Publisher(command_pub_topic, adapter_msgs.Adapter)
        self.command_sub = rospy.Subscriber(command_sub_topic, String, self.command_callback)
        self.command_flag = False

    def rapp_parse(self, uri):
        rapp = uri.split('#')[1]
        return rapp

    '''
    Called by Publisher to allocate a resource
    '''
    def pub_resource_alloc(self, uri, options):
        self.resource_alloc_flag = False
        data = adapter_msgs.Adapter()
        data.resource.id = unique_id.toMsg(unique_id.fromRandom())
        data.resource.uri = uri
        data.resource.rapp = self.rapp_parse(uri)
        data.command = options

        time.sleep(1)
        self.resource_alloc_pub.publish(data)

    '''
    Called by Subscriber when the resource infomation arrived from the adapter
    '''
    def resource_alloc_callback(self, msg):
        if msg.data=="resource_alloc_success":
            self.resource_alloc_flag= True
        else:
            self.resource_alloc_flag= False
        return


    '''
    Called by Publisher to control the robot with command during 'duration'
    '''
    def pub_command(self, duration, options):
        self.command_flag = False
        data = adapter_msgs.Adapter()
        data.resource.id = unique_id.toMsg(unique_id.fromRandom())
        data.command_duration = int(duration)
        data.command = options

        print data

        time.sleep(1)
        self.command_pub.publish(data)

    '''
    Called by Subscriber when the command arrived from the adapter
    '''
    def command_callback(self, msg):
        print "==============publisher.py (command_callback)=============="
        print "msg.data: ", msg.data
        if msg.data=="command_success":
            self.command_flag= True
        else:
            self.command_flag= False
        return
