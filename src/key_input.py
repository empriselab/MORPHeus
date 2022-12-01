#!/usr/bin/env python
import rospy
from curtsies import Input
from std_msgs.msg import String, Header #String message 
from data_collection.msg import StringStamped
from std_srvs.srv import Empty


################################
# created by yuvaram
#yuvaramsingh94@gmail.com
################################

key_dict = {
    "p": "[p] - Peeling over peeled area", 
    "u": "[u] - Peeling over unpeeled area", 
    "r": "[r] - Not peeling"
    }

print(f"Use the following keys to annotate the associated actions"\
    f"\n\t {key_dict['p']}"\
    f"\n\t {key_dict['u']}"\
    f"\n\t {key_dict['r']}"\
    f"\n\n To EXIT press <CTRL+C> and then <ESC>")

def keys():

    pub = rospy.Publisher('key_input', StringStamped,queue_size=10) # "key" is the publisher name

    # Setting anonymous to False so only one instance can run at a time
    rospy.init_node('keypress',anonymous=False)

    # Service Proxy to Tare (zero out FT sensor)
    reset_bias = rospy.ServiceProxy(
        'wireless_ft/reset_bias', Empty)

    with Input(keynames='curses') as input_generator:

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            # for every input in buffer (?)
            for e in input_generator:

                # check if valid input
                if e in key_dict:
                    print(key_dict[e])

                    # Tare
                    resp = reset_bias()

                    # Create message and publish
                    h = Header()
                    h.stamp = rospy.Time.now()
                    msg = StringStamped(h, String(e))
                    pub.publish(msg)#to publish
                    rate.sleep()

                # exit loop with <ESC> (\x1b) or <CTRL + D> (\x04)
                if e in ('\x1b', '\x04'):
                    break



if __name__=='__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        pass