#!/usr/bin/env python3

import rosbag
import rospy
import numpy as np
import os
from array import array
import struct

"""
Header header
uint8 num_rx
uint8 num_tx
int8[] tx_order
uint32 num_adc_samples_per_chirp
uint32 num_chirps_per_frame
uint32 adc_sample_frequency # in Hz
float32 start_frequency # in Hz
float32 idle_time # in seconds
float32 adc_start_time # in seconds
float32 ramp_end_time # in seconds
float32 frequency_slope # in Hz/s
int16[] samples
"""
index = 0
basedir = '/home/ajay/catkin_ws_rmap/coloradar'

bag_file = 'ec_hallways_run0.bag'

radar_bag = rosbag.Bag(os.path.join(basedir,bag_file))
# Iterate over the lang_full messages
tx_order = []
samples = []

bagdir = os.path.join(basedir, bag_file.split('.bag')[0])
if not os.path.exists(bagdir):
    os.makedirs(bagdir)
radar_dir = os.path.join(bagdir, 'radar')
if not os.path.exists(radar_dir):
    os.makedirs(radar_dir)

for topic, msg, time in radar_bag.read_messages(topics=["/dca_node/data_cube"]):
    # Start a new output file
    num_rx = msg.num_rx
    num_tx = msg.num_tx
    num_adc_samples_per_chirp = msg.num_adc_samples_per_chirp
    num_chirps_per_frame = msg.num_chirps_per_frame
    adc_sample_frequency = msg.adc_sample_frequency
    start_frequency = msg.start_frequency
    idle_time = msg.idle_time
    adc_start_time = msg.adc_start_time
    ramp_end_time = msg.ramp_end_time
    frequency_slope = msg.frequency_slope
    print("\nRx: ",num_rx, "\nTx: ", num_tx, "\nnum_adc_samples_per_chirp: ", num_adc_samples_per_chirp, "\nnum_chirps_per_frame:", num_chirps_per_frame, "\nadc_sample_frequency:", adc_sample_frequency, "\nstart_frequency:", start_frequency, "\nidle_time:", idle_time, "\nadc_start_time:", adc_start_time, "\nramp_end_time:", ramp_end_time, "\nfrequency_slope:", frequency_slope)
    msg_tx_order = np.array(msg.tx_order, dtype=np.int8)
    tx_order.append(list(msg_tx_order))
    
    samples = np.array(msg.samples, dtype=float)
    # frame_vals = frame_vals[:-1:2] + 1j * frame_vals[1::2]
    data_filename = os.path.join(radar_dir, str(index)+".npy")
    samples = samples[:-1:2]
    with open(data_filename, "wb") as f:
       np.save(f, np.array(samples))
    # print(msg_samples.shape, msg_tx_order, index)
    index += 1
    
        
# Close the bag files
radar_bag.close()

