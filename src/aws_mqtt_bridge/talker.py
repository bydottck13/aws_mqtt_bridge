#!/usr/bin/env python
import rospy
import json
from aws_mqtt_bridge.msg import MQTT_publish

def talker():
    pub = rospy.Publisher('publish_to_aws', MQTT_publish, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 0.5hz
    msg = MQTT_publish()
    msg.topic = "test1/test2"
    
    while not rospy.is_shutdown():
        message = {}
        message['message'] = "hello world %s" % rospy.get_time()
        msg.payload = json.dumps(message)
        rospy.loginfo(rospy.get_caller_id() + " Send %s to %s" % (msg.payload, msg.topic))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
