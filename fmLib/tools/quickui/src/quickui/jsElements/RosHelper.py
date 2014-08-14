
import roslib.message
import rospy

def create_publisher_from_type(topic,type):
    base_message_type = roslib.message.get_message_class(type)
    return rospy.Publisher(topic,base_message_type)

def create_subscriber_from_type(topic_name, topic_type, cb):
    base_message_type = roslib.message.get_message_class(topic_type)
    return rospy.Subscriber(topic_name,base_message_type,cb)

def create_msg_from_type(topic_type):
    base_message_type = roslib.message.get_message_class(topic_type)
    return base_message_type()

def get_all_msg_fields(topic_type):
    return None