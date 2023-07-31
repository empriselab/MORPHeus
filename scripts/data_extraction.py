import os, sys
import cv2
import numpy as np
import time
import math
import argparse

import signal
import pickle

import matplotlib.pyplot as plt
plt.switch_backend('agg')

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import struct
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped, WrenchStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float64MultiArray, Bool, String, Float32
from data_collection.msg import StringStamped
from audio_common_msgs.msg import AudioData
from types import SimpleNamespace

from threading import Lock
from copy import deepcopy


class DataExtractor:
    def __init__(self):
        rospy.init_node('DataExtractor')
        
        self.force_sensor_lock = Lock()
        self.force_sensor_data_buffer = []

        self.audio_lock = Lock()
        self.audio_data_buffer = []

        self.dataset_lock = Lock()
        self.dataset = []

        queue_size = 1

        self.force_sensor_sub = rospy.Subscriber('force', Float32, self.force_sensor_callback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.audio_sub = rospy.Subscriber('/audio/audio', AudioData, self.audio_callback, queue_size = queue_size, buff_size = 65536*queue_size)

        self.last_key_continuous_data = None
        self.key_continuous_sub = rospy.Subscriber('/key_continuous', StringStamped, self.key_continuous_callback, queue_size = queue_size, buff_size = 65536*queue_size)

    def force_sensor_callback(self, force_sensor_msg):

        data = [force_sensor_msg.data]
   
        with self.force_sensor_lock:
            self.force_sensor_data_buffer.append(data)

            # if len(self.ft_sensor_data_buffer) > 125:
            #     self.ft_sensor_data_buffer.pop(0)

    def audio_callback(self, audio_msg):
            
        with self.audio_lock:
            self.audio_data_buffer.append(audio_msg.data)

            # if len(self.audio_data_buffer) > 30:
            #     self.audio_data_buffer.pop(0)

    def key_continuous_callback(self, key_continuous_msg):
            
        if self.last_key_continuous_data is not None and self.last_key_continuous_data != key_continuous_msg.data.data:
            if key_continuous_msg.data.data == "0":
                self.store_data(self.last_key_continuous_data)
                self.clear_buffers()
            else:
                print("---")
                self.clear_buffers()
                print("Listening to data with key: ", key_continuous_msg.data.data)
        
        self.last_key_continuous_data = key_continuous_msg.data.data

    def clear_buffers(self):

        print("Clearing buffers...")

        with self.force_sensor_lock:
            self.force_sensor_data_buffer.clear()
        
        with self.audio_lock:
            self.audio_data_buffer.clear()

    def store_data(self, key):

        print("Storing data with key: ", key)

        force_sensor_data_buffer = None
        audio_data_buffer = None

        with self.force_sensor_lock:
            force_sensor_data_buffer = deepcopy(self.force_sensor_data_buffer)

        with self.audio_lock:
            audio_data_buffer = deepcopy(self.audio_data_buffer)

        if force_sensor_data_buffer is not None and audio_data_buffer is not None:

            print("Number of force_sensor frames: ", len(force_sensor_data_buffer))
            print("Number of audio frames: ", len(audio_data_buffer))

            with self.dataset_lock:
                self.dataset.append({'haptic':force_sensor_data_buffer, 'audio': audio_data_buffer, 'key': key})

        else:
            print("No data to save")

    def signal_handler(self, signal, frame):

        print("\nSaving dataset...")

        with self.dataset_lock:
            print(f"Number of datapoints: {len(self.dataset)}")
            with open('../data/peeling_data.pickle', 'wb') as handle:
                pickle.dump(self.dataset, handle, protocol=pickle.HIGHEST_PROTOCOL)

        print("\nprogram exiting gracefully ...")
        sys.exit(0)
        
if __name__ == '__main__':

    data_extractor = DataExtractor()
    signal.signal(signal.SIGINT, data_extractor.signal_handler) # ctrl+c

    print("spinning")
    rospy.spin()