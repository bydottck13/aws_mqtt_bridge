# ***** NOTICE *****
This is a ROS package for bridging aws-iot-mqtt services. All coding structures are referenced from [ROS mqtt_bridge](http://wiki.ros.org/mqtt_bridge).

------

## REQUIREMENTS
* Ubuntu 16.04
* ROS Kinetic Kame
* [aws-iot-device-sdk-python](https://github.com/aws/aws-iot-device-sdk-python)

## DESCRIPTIONS
* From AWS to ROS device:  
From AWS-IOT cloud, publishing to `aws_to_device/chatter` with the payload in the JSON format. This would convert to a ROS topic with the ***String*** format, e.g.
> - ROS :
>> - topic: chatter
>> - type: std_msgs/String
>> - data: "Hello, this is the message from AWS IoT"
> - AWS IOT MQTT:
>> - topic: aws_to_device/chatter
>> - payload: {
>>  "data": "Hello, this is the message from AWS IoT"
>> }
* From ROS device to AWS:  
From ROS device, publish to `publish_to_aws` with the payload in `MQTT_publish.msg` format. This would convert to a MQTT topic, e.g.
> - AWS IOT MQTT:
>> - topic: test1/test2
>> - payload: {"message": "hello world"}
> - ROS :
>> - topic: publish_to_aws
>> - payload: "{topic: test1/test2, payload: '{\"message\": \"hello world\"}'}"

## INSTALLATIONS
* aws-iot-device-sdk-python
```
$ git clone https://github.com/aws/aws-iot-device-sdk-python.git
$ cd aws-iot-device-sdk-python
$ python setup.py install
```
* aws_mqtt_bridge
> - Create the `catkin_ws` directory
```
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/catkin_ws
$ cd ~/catkin_ws
$ catkin_make
```
> - Make the `aws_mqtt_bridge`
```
$ source devel/setup.bash
$ cd src
$ git clone https://github.com/bydottck13/aws_mqtt_bridge.git
$ cd ..
$ catkin_make install --pkg aws_mqtt_bridge
```

## USAGES
Configurate the parameter file (demo_params.yaml):
```
mqtt:
  host: {YOUR-AWS-MQTT-HOST}
  rootCAPath: {YOUR-AWS-MQTT-ROOT-CA}
  certificatePath: {YOUR-AWS-MQTT-CERTIFICATE}
  privateKeyPath: {YOUR-AWS-MQTT-PRIVATE-KEY}
  clientId: {CLIENT-ID}
```
And launch the aws_mqtt_bridge package:
```
$ roslaunch aws_mqtt_bridge demo.launch
```

## TESTING
* Test from ros to aws-iot-mqtt:
> - Using `rostopic`: 
```
$ rostopic pub -1 /publish_to_aws aws_mqtt_bridge/MQTT_publish "{topic: test1/test2, payload: '{\"message\": \"hello world\"}'}"
```
> - Using the `talker node`, please modify the `demo.launch`:
```
<!-- For testing ROS to AWS IoT MQTT, uncomment this -->
<node name="talker" pkg="aws_mqtt_bridge" type="talker.py" output="screen" />
```
And subscribe to the topic `test1/test2` on aws-iot-mqtt.
* Test from aws-iot-mqtt to ros:
> - Using the `listener node`, please modify the `demo.launch`:
```
<!-- For testing AWS IoT MQTT to ROS, uncomment this -->
<node name="listener" pkg="aws_mqtt_bridge" type="listener.py" output="screen" />
```
And publish to the topic `aws_to_device/chatter` on aws-iot-mqtt.
>> - {
>>  "data": "Hello, this is the message from AWS IoT"
>> }
> - Using the `turtlesim node`
```
$ rosrun turtlesim turtlesim_node
```
And publish to the topic `aws_to_device/turtlesim`
>> - {
>>  "linear": {
>>    "x": 2.0,
>>    "y": 0.0,
>>    "z": 0.0 },
>>  "angular": {
>>    "x": 0.0,
>>    "y": 0.0,
>>    "z": 2.0 }
>> }

## REFERENCE
* [ROS mqtt_bridge](http://wiki.ros.org/mqtt_bridge)
* [AWS IoT sdk setup](http://docs.aws.amazon.com/iot/latest/developerguide/iot-sdk-setup.html)
