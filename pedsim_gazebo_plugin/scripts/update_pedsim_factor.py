#!/usr/bin/env python
"""
Created on Wed Mar  18 12:53:00 2021

@author: fagner pimentel
"""

import rospy
import dynamic_reconfigure.client

from std_msgs.msg import Float32

def factor_callback(msg):

    params = { 'simulation_factor' : msg.data}
    config = client.update_configuration(params)

if __name__ == '__main__':

    rospy.init_node("update_pedsim_factor")

    node = rospy.get_param('~node_to_reconfigure')
    topic = rospy.get_param('~topic_with_new_factor')
    
    client = dynamic_reconfigure.client.Client(node)
    
    rospy.Subscriber(topic, Float32, factor_callback)

    rospy.spin()
