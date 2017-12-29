#!/usr/bin/env python
from __future__ import absolute_import
from importlib import import_module
import roslib.message

ros_primitive_types = ['bool', 'byte', 'char', 'int8', 'int16',
						'uint16', 'int32', 'uint32', 'int64', 'uint64',
						'float32', 'float64', 'string']

def lookup_object(object_path, package='aws_mqtt_bridge'):
	# object_path, e.g. std_msgs.msg:String
    module_name, obj_name = object_path.split(":")
    module = import_module(module_name, package)
    obj = getattr(module, obj_name)
    return obj

def convert_json_to_ros_message(object_path, json_data):
	if ":" in object_path:
		module_name, obj_name = object_path.split(":")
		message_name, message_postfix = module_name.split(".")
		# object_path, e.g. std_msgs/String
		message_type = message_name + "/" + obj_name
	else:
		message_type = object_path
	message_class = roslib.message.get_message_class(message_type)
	message = message_class()
	message_fields = dict(_get_message_field(message))

	"""
	# for debuging
	for field in message_fields.items():
		print "field is " +str(tuple(field))
	"""

	for field_name, field_value in json_data.items():
		if field_name in message_fields:
			field_type = message_fields[field_name]
			field_value = _convert_to_ros_type(field_type, field_value)
			setattr(message, field_name, field_value)
		else:
			error_message = 'ROS message type "{0}" has no field named "{1}"'\
				.format(message_type, field_name)
			raise ValueError(error_message)
	return message

def _convert_to_ros_type(field_type, field_value):
	if field_type in ros_primitive_types:
		field_value = _convert_to_ros_primitive(field_type, field_value)
	else:
		field_value = convert_json_to_ros_message(field_type, field_value)
	return field_value

def _convert_to_ros_primitive(field_type, field_value):
	return field_value

def _get_message_field(message):
	return zip(message.__slots__, message._slot_types)


__all__ = ['lookup_object', 'convert_json_to_ros_message']