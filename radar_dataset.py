#!/usr/bin/env python3

import rosbag
import rospy
import numpy as np
import os
from array import array
import struct

index = 0
basedir = '/home/ajay/catkin_ws_rmap/coloradar'

bag_file = 'ec_hallways_run0.bag'

radar_bagfile = os.path.join(basedir,bag_file)

bagdir = os.path.join(basedir, bag_file.split('.bag')[0])
if not os.path.exists(bagdir):
    os.makedirs(bagdir)
radar_dir = os.path.join(bagdir, 'radar')
if not os.path.exists(radar_dir):
    os.makedirs(radar_dir)



# returns the interpolated index of the value in the input array 
# where the input value would be
def get_array_idx(arr, val):

  lower_idx = min(range(len(arr)), key = lambda i: abs(arr[i]-val))
  if arr[lower_idx] > val:
    lower_idx -= 1

  if lower_idx < 0:
    return float(lower_idx)
  elif lower_idx == len(arr) - 1:
    if val > arr[lower_idx]:
      return float(lower_idx + 1)
    else:
      return float(lower_idx)

  upper_idx = lower_idx + 1
  lower_val = arr[lower_idx]
  upper_val = arr[upper_idx]

  c = (val - lower_val) / (upper_val - lower_val)
  idx = lower_idx * (1.0 - c) + upper_idx * c

  return idx

# transforms polar image with min range, azimuth, and elevation at (0,0,0)
# to cartesian image with min x, y and z at (0,0,0)
def resample_to_cartesian(polar_img, dim, vox_width, bin_widths):
  global sampler
  global im_points
  global verbose_debug

  range_bin_width = bin_widths[0] # scalar
  azimuth_bins = bin_widths[1]    # array
  elevation_bins = bin_widths[2]  # array

  x_dim = dim[0]
  y_dim = dim[1]
  z_dim = dim[2]

  device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

  if im_points is None:
    im_points = torch.zeros(x_dim,y_dim,z_dim,3,device=device)
    for i in range(x_dim):
      x = vox_width * float(i)
      for j in range(y_dim):
        y = vox_width * (float(j) - (float(y_dim)-1.) / 2.0)
        for k in range(z_dim):
          z = vox_width * (float(k) - (float(z_dim)-1.) / 2.0)
          r = math.sqrt(x**2 + y**2 + z**2)
          phi = math.atan2(z,math.sqrt(x**2 + y**2))
          theta = math.atan2(y,x)

          r_bin = r / range_bin_width
          theta_bin = get_array_idx(azimuth_bins, theta)
          phi_bin = get_array_idx(elevation_bins, phi)

          im_points[i,j,k,:] = torch.tensor([r_bin,theta_bin,phi_bin])

    im_points = im_points.view(1,-1,3,1).to(device)
    if verbose_debug:
      print("Shape of cartesian tensor:", im_points.shape)

  polar_img = torch.from_numpy(polar_img[0:1,:,:,:])
  polar_img = polar_img.to(device)

  # need to verify sampler function with multi-channel images
  cartesian_arr = sampler(im_points, 
                          polar_img.view(1,
                                         polar_img.shape[0],
                                         polar_img.shape[1],
                                         polar_img.shape[2],
                                         polar_img.shape[3]))
  if verbose_debug:
    print("Shape of carteisan array:", cartesian_arr.shape)
  cartesian_img = cartesian_arr.transpose(1,2).view(1,x_dim,y_dim,z_dim)

  return cartesian_img.float().cpu()



def polar_to_cartesian(r, az, el):
  x = r * math.cos(el) * math.cos(az)
  y = r * math.cos(el) * math.sin(az)
  z = r * math.sin(el)
  return (x, y, z)

def process_img(msg,t):
  global im_coords
  width = msg.width
  height = msg.height
  depth = msg.depth
  az_bins = msg.azimuth_bins
  el_bins = msg.elevation_bins
  range_bin_width = msg.range_bin_width
  arr = msg.image
  
  img = np.zeros((2, depth, width, height))
  for range_idx in range(depth):
    for az_idx in range(width):
      for el_idx in range(height):
        angle_idx = az_idx + width * el_idx
        src_idx = 2 * (range_idx + depth * angle_idx)
        img[0,range_idx,az_idx,el_idx] = arr[src_idx]
        img[1,range_idx,az_idx,el_idx] = arr[src_idx+1]

  if im_coords is None:
    im_coords = np.zeros((3, depth, width, height))
    for range_idx in range(depth):
      r = range_idx * range_bin_width
      for az_idx in range(width):
        az = az_bins[az_idx]
        for el_idx in range(height):
          el = el_bins[el_idx]
          im_coords[:, range_idx, az_idx, el_idx] = polar_to_cartesian(r, az, el)
  img = np.concatenate((img, im_coords), axis=0)

  bin_widths = (msg.range_bin_width, msg.azimuth_bins, msg.elevation_bins)
  cartesian_img = resample_to_cartesian(img, (64,128,64), 0.12, bin_widths)
  
  '''
  img = np.zeros((5, depth, width))
  for range_idx in range(depth):
    r = range_idx * range_bin_width
    for az_idx in range(width):
      az = az_bins[az_idx]

      max_el_i = 0
      max_el_d = 0
      max_el_bin = 0
      for el_idx in range(height):
        angle_idx = az_idx + width * el_idx
        src_idx = 2 * (range_idx + depth * angle_idx)
        if arr[src_idx] > max_el_i:
          max_el_i = arr[src_idx]
          max_el_d = arr[src_idx+1]
          max_el_bin = el_idx

      el = el_bins[max_el_bin]
      img[2:, range_idx, az_idx] = polar_to_cartesian(r, az, el)
      img[0, range_idx, az_idx] = max_el_i
      img[1, range_idx, az_idx] = max_el_d
  '''

  #img = torch.from_numpy(img).float()

  return (msg.header.stamp.to_sec(), cartesian_img)


# read raw messages from a rosbag
def read_bag(bag, radar_topic="/cascade/heatmap"):
  msgs = {'radar':[]}

  print('reading ' + filename)

  print('getting radar messages')
  bag_msgs = bag.read_messages(topics=[radar_topic])
  msgs['radar'] = [process_img(msg,t) for (topic,msg,t) in bag_msgs]

  max_intensity = 0
  min_intensity = 1e10
  for radar_im in msgs['radar']:
    if radar_im[1][0,:,:,:].max() > max_intensity:
      max_intensity = radar_im[1][0,:,:,:].max()
    if radar_im[1][0,:,:,:].min() < min_intensity:
      min_intensity = radar_im[1][0,:,:,:].min()

  for radar_im in msgs['radar']:
    radar_im[1][0,:,:,:] -= min_intensity
    radar_im[1][0,:,:,:] /= (max_intensity - min_intensity)

  msgs['radar'].sort()

  return msgs


def read_tf_file(filename):

  T_br = np.eye(4)

  if not os.path.exists(filename):
    print('File ' + filename + ' not found')
    return T_br

  with open(filename, mode='r') as file:
    lines = file.readlines()

  t = [float(s) for s in lines[0].split()]
  q = [float(s) for s in lines[1].split()]

  rot = tf.Rotation.from_quat(q)
  R = rot.as_matrix()
  
  T_br[:3,:3] = R
  T_br[:3,3] = t

  return T_br

if __name__ == "__main__":

    im_points = None
    im_coords = None
    fixed_points = None
    verbose_debug = True

    print('getting messages')
    radar_bag = rosbag.Bag(radar_bagfile)
    msgs = read_bag(radar_bag)


