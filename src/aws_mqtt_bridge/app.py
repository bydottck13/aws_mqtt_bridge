#!/usr/bin/env python
from __future__ import absolute_import

import rospy
import inject

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

from .bridge import create_publish_bridge
from .bridge import create_subscribe_bridge

def create_config(aws_mqtt_client):
    def config(binder):
        binder.bind(AWSIoTMQTTClient, aws_mqtt_client)
    return config

def aws_mqtt_node():
    # init node
    rospy.init_node('aws_mqtt_node', anonymous=True, log_level=rospy.DEBUG)

    # load parameters
    params = rospy.get_param("~", {})
    mqtt_params = params.pop("mqtt", {})
    bridge_params = params.pop("bridge", {})

    host = mqtt_params.pop("host", "")
    rootCAPath = mqtt_params.pop("rootCAPath", "")
    certificatePath = mqtt_params.pop("certificatePath", "")
    privateKeyPath = mqtt_params.pop("privateKeyPath", "")
    clientId = mqtt_params.pop("clientId", "")

    # Init AWSIoTMQTTClient
    myAWSIoTMQTTClient = AWSIoTMQTTClient(clientId)
    myAWSIoTMQTTClient.configureEndpoint(host, 8883)
    try:
        myAWSIoTMQTTClient.configureCredentials(rootCAPath, privateKeyPath, certificatePath)
    except:
        raise IOError("Cannot load certificates...")
    else:
        # AWSIoTMQTTClient connection configuration
        myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
        myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
        myAWSIoTMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
        myAWSIoTMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
        myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

        # dependency injection
        config = create_config(myAWSIoTMQTTClient)
        inject.configure(config)

        # Connect to AWS IoT
        try:
            myAWSIoTMQTTClient.connect()
        except:
            raise IOError("Cannot connect to AWS IoT...")
        else:
            # configure bridges
            bridges = []
            # setup publishing bridge
            bridges.append(create_publish_bridge())
            # setup subscribing bridges
            for bridge_args in bridge_params:
                try:
                    bridges.append(create_subscribe_bridge(**bridge_args))
                except Exception, e:
                    rospy.logerr(str(e))
                    rospy.logerr("Cannot subscribe to the topic %s" %(bridge_args.pop("topic_from", "")))

            rospy.loginfo(rospy.get_caller_id()+" All settings are ready!")

            # spin() simply keeps python from exiting until this node is stopped
            for bridge in bridges:
                rospy.on_shutdown(bridge.on_shutdown)
            rospy.on_shutdown(myAWSIoTMQTTClient.disconnect)
            rospy.spin()

__all__ = ['aws_mqtt_node']