#!/usr/bin/env python
from __future__ import absolute_import

import rospy
import inject
import json

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from aws_mqtt_bridge.msg import MQTT_publish
from .utils import lookup_object, convert_json_to_ros_message

def create_publish_bridge():
    return RosToAwsBridge(topic_from="publish_to_aws")

def create_subscribe_bridge(msg_type, topic_from, topic_to):
    if isinstance(msg_type, basestring):
        msg_type_obj = lookup_object(msg_type)
    if not issubclass(msg_type_obj, rospy.Message):
        raise TypeError(
            "msg_type should be rospy.Message instance or its string"
            "representation")
    return AwsToRosBridge(msg_type_obj=msg_type_obj, msg_type=msg_type, topic_from=topic_from, topic_to=topic_to)

class Bridge(object):
    _aws_mqtt_client = inject.attr(AWSIoTMQTTClient)

class RosToAwsBridge(Bridge):
    def __init__(self, topic_from):
        self._topic_from = topic_from
        rospy.Subscriber(topic_from, MQTT_publish, self._callback_ros)

    def _callback_ros(self, msg):
        rospy.logdebug(rospy.get_caller_id() + " Publish to the AWS IoT topic '%s' with the payload '%s'" % (msg.topic, msg.payload))
        # Publish to AWS IoT
        self._aws_mqtt_client.publish(msg.topic, msg.payload, 1)

    def on_shutdown(self):
        rospy.logdebug("shutdown RosToAwsBridge...")

class AwsToRosBridge(Bridge):
    def __init__(self, msg_type_obj, msg_type, topic_from, topic_to):
        self._msg_type_obj = msg_type_obj
        self._msg_type = msg_type
        self._topic_from = topic_from
        self._topic_to = topic_to
        # Subscribe to AWS IoT
        self._aws_mqtt_client.subscribe(topic_from, 1, self._callback_aws_iot)
        rospy.loginfo(rospy.get_caller_id() + " Subscribe to the topic %s", topic_from)
        self._publisher = rospy.Publisher(self._topic_to, self._msg_type_obj, queue_size=10)

    def _callback_aws_iot(self, client, userdata, message):
        rospy.logdebug("Received a MQTT message: topic is '%s', and payload is '%s'" % (message.topic, message.payload))
        decodeMQTTJson = json.loads(message.payload)
        ros_msg = None
        try:
            ros_msg = convert_json_to_ros_message(self._msg_type, decodeMQTTJson)
        except Exception, e:
            rospy.logerr(str(e))
        else:
            rospy.logdebug(rospy.get_caller_id() + " Publish to the ROS topic '%s' with the payload:", self._topic_to)
            rospy.logdebug(json.dumps(message.payload))
            self._publisher.publish(ros_msg)

    def on_shutdown(self):
        rospy.logdebug("unsubscribe %s..." %(self._topic_from))
        self._aws_mqtt_client.unsubscribe(self._topic_from)

__all__ = ['create_publish_bridge', 'create_subscribe_bridge', 'Bridge', 'RosToAwsBridge', 'AwsToRosBridge']