
import os
import sys
import csv
import copy
import time
import random
import argparse
import glob
from datetime import datetime
import shutil
from joblib import Parallel, delayed

import json
import numpy as np
import pandas as pd
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

import utils.UtilsPointcloud as Ptutils
import open3d as o3d
import cv2

import imo_pcd_reader

# params
parser = argparse.ArgumentParser(description='Point Cloud Alignment')

parser.add_argument('--slam_paras_file_dir', type=str, 
                    default='/your/path/to/.../*_pcs_data_path.csv')

parser.add_argument('--camera_calib_dir', type=str, 
                    default='00.json')

parser.add_argument("--no_assign_colors", action="store_true", default=False,
                    help="Assign colors from images to lidar point cloud (default: false)")

parser.add_argument("--keep_mid_pcds", action="store_true", default=False,
                    help="Keep middle pcds data if present (default: false)")

args = parser.parse_args()

# dataset
correct_pcd_channel = "lidarFusion_pcd"
dummy_pcdname = "dummy.pcd"
data_list_file_dir = glob.glob(os.path.join(args.slam_paras_file_dir, "*_pcs_data_path.csv"))[0]
df = pd.read_csv(data_list_file_dir)
tmp_scan_paths = df['File Path'].tolist()
scan_paths = []
scan_names = []
for pc_path in tmp_scan_paths:
    dirname, pcdname = os.path.split(pc_path)
    dirname, channel_name = os.path.split(dirname)
    parts = pcdname.split("_")
    search_str = parts[0] + "_" + parts[1]
    data_list = glob.glob(os.path.join(dirname, correct_pcd_channel, search_str+"*.pcd"))
    if len(data_list) == 1:
        pcdname = os.path.basename(data_list[0])
    else:
        pcdname = dummy_pcdname
    scan_paths.append(os.path.join(dirname, correct_pcd_channel, pcdname))
    scan_names.append(pcdname)
num_frames = len(scan_paths)

pose_file_dir = glob.glob(os.path.join(args.slam_paras_file_dir, "pose*unoptimized.csv"))[0]
dirname, filename = os.path.split(pose_file_dir)
seq_name = os.path.basename(dirname)
parent_dir = os.path.abspath(os.path.join(dirname, os.pardir))
project_name = os.path.basename(parent_dir)

# date_obj = datetime.now().date()
# sub_folder = "lane_" + date_obj.strftime("%Y%m%d")
sub_folder = "lane"

save_dir = os.path.join("result_pcd", project_name, sub_folder, seq_name, "pcds")
if not os.path.exists(save_dir): os.makedirs(save_dir)

def read_csv_to_matrices(filename):
    matrices = []
    current_matrix_rows = []
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            current_matrix_rows.append([float(num) for num in row])  # Convert to floats
            matrices.append(np.array(current_matrix_rows).reshape((4, 4)))
            current_matrix_rows = []  # Reset for the next matrix

    return matrices

def copy_file(source_file, destination_file):
    try:
        shutil.copy2(source_file, destination_file)
        print("File copied successfully.")
    except FileNotFoundError:
        print(f"Error: Source file '{source_file}' not found.")
    except PermissionError:
        print(f"Error: Permission denied for '{destination_file}'.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

camera_names = ['front',
                'rear',
                'svLeftFront',
                'svRightFront',
                'svLeftRear',
                'svRightRear',
                'tvRear',
                'tvLeft',
                'tvFront',
                'tvRight']

calib_intri_dict = {}
calib_extri_dict = {}
for c_name in camera_names:
    with open(os.path.join(args.camera_calib_dir, c_name+'.json'), 'r') as f:
        camera_calib = json.load(f)
        calib_intri_dict[c_name] = np.array(camera_calib["internal"]).reshape((3, 3))
        calib_extri_dict[c_name] = np.array(camera_calib["extrinsic"]).reshape((4, 4))

def assign_colors_from_image(pc_coords, pc_path):
    dirname, filename = os.path.split(pc_path)
    parent_dir = os.path.abspath(os.path.join(dirname, os.pardir))
    parts = filename.split("_")
    search_str = parts[0] + "_" + parts[1]
    img_dir_dict = {}
    for c_name in camera_names:
        data_list = glob.glob(os.path.join(parent_dir, c_name, search_str+"*.png"))
        if len(data_list) == 1:
            img_dir_dict[c_name] = data_list[0]

    colors = imo_pcd_reader.assign_colors(pc_coords, camera_names, img_dir_dict, calib_intri_dict, calib_extri_dict)
    return colors

def cloud_transform(mat, scan_path, out_name, save_dir, excluded_area, ceiling_height):
    if out_name != dummy_pcdname:
        scan = imo_pcd_reader.read_pcd_with_excluded_area(scan_path, excluded_area, ceiling_height)
        coord = scan[:, :3]
        intensities = scan[:, -1]
        new_column = np.ones((coord.shape[0], 1))
        aug_coord = np.hstack((coord, new_column))
        trans_coord = mat.dot(aug_coord.T)
        out_coord = trans_coord.T[:, :3]
        if args.no_assign_colors:
            rgbs = np.ones((coord.shape[0], 3))
        else:
            rgbs = assign_colors_from_image(coord, scan_path)
        save_pcd_path = os.path.join(save_dir, out_name)
        imo_pcd_reader.save_MAP_pcd(out_coord, rgbs, intensities, save_pcd_path)

all_matrices = read_csv_to_matrices(pose_file_dir)
excluded_area = np.array([[-1, 3], [-1, 1]])
ceiling_height = 2

lidar_valid_range = 50
corners = np.zeros((1, 4))
local_c = np.array([[-lidar_valid_range, -lidar_valid_range, 0, 1], 
                    [-lidar_valid_range, lidar_valid_range, 0, 1], 
                    [lidar_valid_range, lidar_valid_range, 0, 1],
                    [lidar_valid_range, -lidar_valid_range, 0, 1]])
for mat in all_matrices:
    trans_c = mat.dot(local_c.T).T
    corners = np.vstack((corners, trans_c))

min_per_column = np.amin(corners, axis=0)
max_per_column = np.amax(corners, axis=0)
g_range = np.array([[min_per_column[0], max_per_column[0]], 
                    [min_per_column[1], max_per_column[1]]])

geohash_file = os.path.basename(data_list_file_dir)
parts = geohash_file.split('_')
fname_prefix = parts[0] + '_' + parts[1] + '_' + parts[2] + '_' + parts[3]

Parallel(n_jobs=-1)(delayed(cloud_transform)(mat, scan_path, out_name, save_dir, excluded_area, ceiling_height) 
                                                for mat, scan_path, out_name in zip(all_matrices, scan_paths, scan_names))

# save whole map to pcd
pcd_list = glob.glob(os.path.join(save_dir, "*.pcd"))
whole_map_dir = os.path.join(os.path.abspath(os.path.join(save_dir, os.pardir)), "whole_map.pcd")
whole_ground_map_dir = os.path.join(os.path.abspath(os.path.join(save_dir, os.pardir)), "whole_ground_map.pcd")
save_ratio = 0.3
imo_pcd_reader.generate_whole_map(pcd_list, save_ratio, whole_map_dir, whole_ground_map_dir)

# generate 2d map for labeling
resolution = 0.05
mapdir, mapfilename = os.path.split(whole_map_dir)
img_rgb = os.path.join(mapdir, fname_prefix + "_rgb.png")
img_intensity = os.path.join(mapdir, fname_prefix + "_intensity.png")
ground_img_intensity = os.path.join(mapdir, fname_prefix + "_ground_intensity.png")
height_data = os.path.join(mapdir, fname_prefix + "_height.csv")
origin_point = os.path.join(mapdir, fname_prefix + "_origin_point.csv")
imo_pcd_reader.generate_2d_map(whole_map_dir, whole_ground_map_dir, g_range, resolution, img_rgb, img_intensity, ground_img_intensity, height_data, origin_point)

pcs_data_path = os.path.join(mapdir, fname_prefix + "_pcs_data_path.csv")
local2global_pose = os.path.join(mapdir, fname_prefix + "_local2global_pose.csv")

copy_file(pose_file_dir, local2global_pose)
with open(pcs_data_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    for pc_path in tmp_scan_paths:
        dirname, pcdname = os.path.split(pc_path)
        dirname, channel_name = os.path.split(dirname)
        dirname, scene_name = os.path.split(dirname)
        project_dir, sub_name = os.path.split(dirname)
        outter_dir, project_name = os.path.split(project_dir)
        if glob.glob(os.path.join(project_dir, "samples", scene_name, channel_name, pcdname)):
            writer.writerow([os.path.join(project_name, "samples", scene_name, channel_name, pcdname)])
        else:
            writer.writerow([os.path.join(project_name, "sweeps", scene_name, channel_name, pcdname)])

if not args.keep_mid_pcds:
    try:
        shutil.rmtree(save_dir, ignore_errors=True)
    except Exception as e:
        print(f"Error removing folder: {e}")

    try:
        os.remove(whole_map_dir)
    except FileNotFoundError:
        print(f"File not found: {whole_map_dir}")
    except PermissionError:
        print(f"Permission denied: {whole_map_dir}")