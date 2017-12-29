#!/usr/bin/env python
import rospy

from aws_mqtt_bridge.app import aws_mqtt_node

if __name__ == '__main__':
    try:
        aws_mqtt_node()
    except rospy.ROSInterruptException:
        pass