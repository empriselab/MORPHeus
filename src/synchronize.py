#!/usr/bin/env python3

import rospy
from audio_common_msgs.msg import AudioData, AudioDataStamped
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Image
from data_collection.msg import SyncedDataStamped, StringStamped
import message_filters
from std_msgs.msg import Float64MultiArray, Bool, String, Float32

import threading 
# Declraing a lock
lock = threading.Lock()

from std_srvs.srv import SetBool, SetBoolResponse

class DataCollection:
    def __init__(self) -> None:

        # Initiate node
        rospy.init_node('data_collection', anonymous=True)

        # subscribers
        # self.audio_sub = message_filters.Subscriber("/audio/audio", AudioData)
        self.audio_stamped_sub = message_filters.Subscriber("/audio/audio_stamped", AudioDataStamped)
        self.img_color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.img_depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        self.img_aligned_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self.force_sub = message_filters.Subscriber("force", Float32)
        self.key_sub = message_filters.Subscriber("/key_continuous", StringStamped)
        self.sync_sub = message_filters.ApproximateTimeSynchronizer([self.img_color_sub, 
                                                                    self.img_depth_sub, 
                                                                    self.img_aligned_sub,
                                                                    # self.audio_sub,
                                                                    self.audio_stamped_sub,
                                                                    self.force_sub,
                                                                    self.key_sub,
                                                                    ],
                                                               queue_size=10, slop=.1)
        self.sync_sub.registerCallback(self.sync_callback)

        # publisher
        self.pub = rospy.Publisher("/synced_data", SyncedDataStamped, queue_size=10)

    # TODO: Add ft_msg back
    def sync_callback(self, img_color_msg, img_depth_msg, img_aligned_msg, audio_stamped_msg, force_msg, key_msg):

        # Publish synced data 
        #  
        synced_msg = SyncedDataStamped(img_color_msg, img_depth_msg, img_aligned_msg, audio_stamped_msg, force_msg, key_msg)
        self.pub.publish(synced_msg)
    

if __name__ == "__main__":
    data_collector = DataCollection()
    lock.acquire()
    try:
        rospy.loginfo("Starting synchronized data collection")
        rospy.spin()
    finally:
        data_collector.save_data()