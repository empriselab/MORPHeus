#!/usr/bin/env python
import rospy
from curtsies import Input
from std_msgs.msg import String, Header #String message 
from data_collection.msg import StringStamped


################################
# created by yuvaram
#yuvaramsingh94@gmail.com
################################

# TODO: Add /key_1 to synced data

string = "r"

def keys():
    global string

    sub = rospy.Subscriber('key_input', StringStamped, key_callback, queue_size=10) # "key" is the publisher name
    pub = rospy.Publisher('key_continuous', StringStamped,queue_size=10) # "key" is the publisher name

    # Setting anonymous to False so only one instance can run at a time
    rospy.init_node('keypublisher',anonymous=False)


    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():

        # Create message and publish
        h = Header()
        h.stamp = rospy.Time.now()
        msg = StringStamped(h, String(string))
        pub.publish(msg)#to publish
        rate.sleep()

def key_callback(key_msg):
    global string
    print("here")
    string = key_msg.data.data
    print(string)

if __name__=='__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        pass