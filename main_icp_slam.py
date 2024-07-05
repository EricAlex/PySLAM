import os
import sys
import csv
import copy
import time
import random
import argparse
import glob
import logging
from joblib import Parallel, delayed, parallel_backend
from datetime import datetime

import numpy as np
np.set_printoptions(precision=4)

from tqdm import tqdm
import geohash

from utils.ScanContextManager import *
from utils.PoseGraphManager import *
from utils.UtilsMisc import *
import utils.UtilsPointcloud as Ptutils
import utils.ICP as ICP
import utils.UtilsIMU as IMU
import open3d as o3d

import imo_pcd_reader

# params
parser = argparse.ArgumentParser(description='PyICP SLAM arguments')

parser.add_argument('--num_icp_points', type=int, default=5000) # 5000 is enough for real time

parser.add_argument('--num_rings', type=int, default=20) # same as the original paper
parser.add_argument('--num_sectors', type=int, default=60) # same as the original paper
parser.add_argument('--num_candidates', type=int, default=20) # must be int
parser.add_argument('--try_gap_loop_detection', type=int, default=10) # same as the original paper

parser.add_argument('--loop_threshold', type=float, default=0.11) # 0.11 is usually safe (for avoiding false loop closure)

parser.add_argument('--data_base_dir', type=str, 
                    default='/your/path/.../data_odometry_velodyne/dataset/sequences')
parser.add_argument('--sequence_idx', type=str, default='00')

parser.add_argument('--save_gap', type=int, default=300)

parser.add_argument('--IMU_dir', type=str, default='IMU.csv')

parser.add_argument('--indoor', action="store_true", default=False, help="will use lidar odometry if present (default: outdoor)")

parser.add_argument('--sub_map_frames', type=int, default=100)

args = parser.parse_args()

def setup_logging(log_file):
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()
        ]
    )

def init_logging():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

def filename_2_timestamp(filename):
    parts = filename.split('_')
    query_datetime = parts[0] + '_' + parts[1][:-3]  # Slice to remove the last 3 digits (milliseconds)
    query_dt = datetime.strptime(query_datetime, '%Y%m%d_%H%M%S') 
    seconds = query_dt.timestamp()
    milliseconds = int(parts[1][-3:])
    return int(seconds * 1e9) + milliseconds * 1e6

dataset_dir = os.path.abspath(os.path.join(args.data_base_dir, os.pardir))
log_dir = os.path.join("result", os.path.basename(dataset_dir))
if not os.path.exists(log_dir): os.makedirs(log_dir)
log_file = os.path.join(log_dir, "log.txt")
setup_logging(log_file)

# dataset 
folder_paths = glob.glob(os.path.join(args.data_base_dir, "scene*"))
folder_names = [os.path.basename(path) for path in folder_paths if os.path.isdir(path)]
scene_names = sorted(folder_names, key=lambda x: int(x.replace("scene", "")))

time_break_th = 1 # seconds

pcd_segments = []
tmp_seg = []
pc_extension = ".pcd"
good_ratio_th = 0.5
min_frame_th = 200
pcd_channel = 'lidarTop'
for index, scene_n in enumerate(scene_names):
    this_dir = os.path.join(args.data_base_dir, scene_n, pcd_channel)
    tmp_pcd_names = os.listdir(this_dir)
    pcd_names = [f for f in os.listdir(this_dir) if f.endswith(pc_extension)]
    if len(pcd_names) == 0:
        continue
    if(len(pcd_names) < len(tmp_pcd_names)):
        logging.warning(f"Transmission errors in {scene_n}, good ratio: {len(pcd_names)}/{len(tmp_pcd_names)}")
        good_ratio = float(len(pcd_names))/len(tmp_pcd_names)
        if good_ratio < good_ratio_th:
            if len(tmp_seg) > min_frame_th:
                pcd_segments.append(tmp_seg)
                logging.info(f"Segment {len(pcd_segments)}, length: {len(tmp_seg)}, time gap: at least 20 seconds")
            tmp_seg = []
            continue
    pcd_names.sort()
    pcd_paths = [os.path.join(this_dir, name) for name in pcd_names]
    tmp_seg.extend(pcd_paths)
    if index == (len(scene_names)-1):
        if len(tmp_seg) > min_frame_th:
            pcd_segments.append(tmp_seg)
            logging.info(f"Segment {len(pcd_segments)}, length: {len(tmp_seg)}")
    else:
        next_dir = os.path.join(args.data_base_dir, scene_names[index+1], pcd_channel)
        next_pcd_names = [f for f in os.listdir(next_dir) if f.endswith(pc_extension)]
        if len(next_pcd_names) == 0:
            if len(tmp_seg) > min_frame_th:
                pcd_segments.append(tmp_seg)
                logging.info(f"Segment {len(pcd_segments)}, length: {len(tmp_seg)}, time gap: at least 20 seconds")
            tmp_seg = []
            continue
        next_pcd_names.sort()
        next_pcd_paths = [os.path.join(next_dir, name) for name in next_pcd_names]
        this_end_dt = IMU.datetime_to_timestamp(os.path.basename(pcd_paths[-1]))
        next_begin_dt = IMU.datetime_to_timestamp(os.path.basename(next_pcd_paths[0]))
        delta_dt_seconds = float(next_begin_dt - this_end_dt)/1e9
        if delta_dt_seconds < 0 or delta_dt_seconds > time_break_th:
            if len(tmp_seg) > min_frame_th:
                pcd_segments.append(tmp_seg)
                logging.info(f"Segment {len(pcd_segments)}, length: {len(tmp_seg)}, time gap: {delta_dt_seconds} seconds")
            tmp_seg = []

parent_base_dir = os.path.abspath(os.path.join(args.data_base_dir, os.pardir))
pose = IMU.IMUPose(os.path.join(parent_base_dir, "IMU.csv"))

# Check IMU
logging.info(f">>>>>>>>>>>>>>>> Cheking IMU data coverage >>>>>>>>>>>>>>>>")
IMU_all_good = True
for seg_idx, scan_paths in enumerate(pcd_segments):
    dirname, filename = os.path.split(scan_paths[0])
    parts = filename.split('_')
    query_timestamp = parts[0] + '_' + parts[1]
    Latitude, Longitude, Altitude = pose.getGPS(query_timestamp)
    if Latitude is None:
        logging.warning(f"No IMU data for {scan_paths[0]}")
    dirname, filename = os.path.split(scan_paths[-1])
    parts = filename.split('_')
    query_timestamp = parts[0] + '_' + parts[1]
    Latitude_e, Longitude_e, Altitude_e = pose.getGPS(query_timestamp)
    if Latitude_e is None:
        logging.warning(f"No IMU data for {scan_paths[-1]}")
    if Latitude is None or Latitude_e is None:
        logging.warning(f"Caution! Segment {seg_idx+1} may have no GPS IMU data coverage!")
        IMU_all_good = False

if IMU_all_good:
    logging.info(f"IMU data all good!")

def subMap(scan_paths, seg_idx):
    logger = logging.getLogger(__name__)
    setup_logging(log_file)
    num_frames = len(scan_paths)
    # Pose Graph Manager (for back-end optimization) initialization
    PGM = PoseGraphManager()
    PGM.addPriorFactor()

    ResultSaver = PoseGraphResultSaver(init_pose=PGM.curr_se3, 
                                    save_gap=(num_frames-1),
                                    num_frames=num_frames,
                                    seq_idx=seg_idx,
                                    save_dir="Dummy_dir")

    ref_timestamp = ''
    query_timestamp = ''
    for for_idx, scan_path in tqdm(enumerate(scan_paths), total=num_frames, mininterval=10.0):

        ref_timestamp = copy.deepcopy(query_timestamp)
        dirname, filename = os.path.split(scan_path)
        parts = filename.split('_')
        query_timestamp = parts[0] + '_' + parts[1]
        pose_trans = np.eye(4)
        imu_init_guess = np.eye(4)
        if(for_idx > 0):
            pose_trans = pose.getTransformationMatrix(ref_timestamp, query_timestamp)
            # imu_init_guess = pose.getIMUInfo(ref_timestamp, query_timestamp)

        # get current information
        curr_scan_pts = Ptutils.readScan(scan_path)

        curr_scan_down_pts = Ptutils.random_sampling(curr_scan_pts, num_points=args.num_icp_points)

        # save current node
        PGM.curr_node_idx = for_idx # make start with 0
        if(PGM.curr_node_idx == 0):
            PGM.prev_node_idx = PGM.curr_node_idx
            prev_scan_pts = copy.deepcopy(curr_scan_pts)
            icp_initial = np.eye(4)
            continue
        
        prev_scan_down_pts = Ptutils.random_sampling(prev_scan_pts, num_points=args.num_icp_points)

        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(curr_scan_down_pts)

        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(prev_scan_down_pts)

        c_d_th = 0.8
        if args.indoor:
            final_transformation, has_converged, fitness_score = imo_pcd_reader.performNDT(curr_scan_pts, prev_scan_pts, icp_initial, 0.2, 0.4, 0.01, 0.1, 50)
            if fitness_score > c_d_th and pose_trans is not None:
                imu_final_transformation, imu_has_converged, imu_fitness_score = imo_pcd_reader.performNDT(curr_scan_pts, prev_scan_pts, pose_trans, 0.2, 0.4, 0.01, 0.1, 50)
                if imu_fitness_score < fitness_score:
                    logger.warning(f"seg_idx: {seg_idx}, idx: {for_idx}, lidar odometry fitness_score too high {fitness_score}, IMU recalculated fitness_score: {imu_fitness_score}")
                    final_transformation = imu_final_transformation
                    fitness_score = imu_fitness_score
        else:
            fitness_score = None
            if pose_trans is not None:
                final_transformation, has_converged, fitness_score = imo_pcd_reader.performNDT(curr_scan_pts, prev_scan_pts, pose_trans, 0.2, 0.4, 0.01, 0.1, 50)

            if fitness_score is None:
                final_transformation, has_converged, fitness_score = imo_pcd_reader.performNDT(curr_scan_pts, prev_scan_pts, icp_initial, 0.2, 0.4, 0.01, 0.1, 50)
                logger.warning(f"seg_idx: {seg_idx}, idx: {for_idx}, No GPS IMU info, use lidar odometry. fitness_score: {fitness_score}")
            else:
                if fitness_score > c_d_th:
                    odom_final_transformation, odom_has_converged, odom_fitness_score = imo_pcd_reader.performNDT(curr_scan_pts, prev_scan_pts, icp_initial, 0.2, 0.4, 0.01, 0.1, 50)
                    if odom_fitness_score < fitness_score:
                        logger.warning(f"seg_idx: {seg_idx}, idx: {for_idx}, fitness_score too high {fitness_score}, maybe GPS is unreliable, lidar odometry recalculated fitness_score: {odom_fitness_score}")
                        final_transformation = odom_final_transformation
                        fitness_score = odom_fitness_score

        if 2*fitness_score > c_d_th:
            max_c_d = c_d_th
        else:
            max_c_d = 2*fitness_score

        reg_p2p = o3d.pipelines.registration.registration_generalized_icp(source = source, 
                                                                target = target, 
                                                                max_correspondence_distance = max_c_d,
                                                                init = final_transformation
                                                                )
        odom_transform = reg_p2p.transformation

        # update the current (moved) pose 
        PGM.curr_se3 = np.matmul(PGM.curr_se3, odom_transform)
        icp_initial = odom_transform # assumption: constant velocity model (for better next ICP converges)

        # add the odometry factor to the graph 
        PGM.addOdometryFactor(odom_transform)

        # renewal the prev information 
        PGM.prev_node_idx = PGM.curr_node_idx
        prev_scan_pts = copy.deepcopy(curr_scan_pts)

        # save the ICP odometry pose result (no loop closure)
        ResultSaver.saveUnoptimizedPoseGraphResultNoWrite(PGM.curr_se3, PGM.curr_node_idx)
    
    return ResultSaver.getPoseList()

def alignSections(target_poses, target_pc_paths, target_idx, source_poses, source_pc_paths, source_idx):
    logger = logging.getLogger(__name__)
    setup_logging(log_file)
    excluded_area = np.array([[-1, 3], [-1, 1]])
    ceiling_height = 100
    read_ratio = 3.0/len(target_pc_paths)

    target_matrices = [target_poses[i].reshape(4, 4) for i in range(target_poses.shape[0])]
    coord_to_stack = []
    for mat, scan_path in zip(target_matrices, target_pc_paths):
        scan = imo_pcd_reader.read_pcd_with_excluded_area_read_ratio(scan_path, excluded_area, ceiling_height, read_ratio)
        # scan = imo_pcd_reader.read_pcd_with_prefiltering(scan_path, 0.2, 0.1, 60, 0.5, 2)
        coord = scan[:, :3]
        new_column = np.ones((coord.shape[0], 1))
        aug_coord = np.hstack((coord, new_column))
        trans_coord = mat.dot(aug_coord.T)
        out_coord = trans_coord.T[:, :3]
        coord_to_stack.append(out_coord)
    target_coords = np.vstack(coord_to_stack)

    source_matrices = [source_poses[i].reshape(4, 4) for i in range(source_poses.shape[0])]
    coord_to_stack = []
    for mat, scan_path in zip(source_matrices, source_pc_paths):
        scan = imo_pcd_reader.read_pcd_with_excluded_area_read_ratio(scan_path, excluded_area, ceiling_height, read_ratio)
        # scan = imo_pcd_reader.read_pcd_with_prefiltering(scan_path, 0.2, 0.1, 60, 0.5, 2)
        coord = scan[:, :3]
        new_column = np.ones((coord.shape[0], 1))
        aug_coord = np.hstack((coord, new_column))
        trans_coord = mat.dot(aug_coord.T)
        out_coord = trans_coord.T[:, :3]
        coord_to_stack.append(out_coord)
    source_coords = np.vstack(coord_to_stack)

    c_d_th = 0.8
    final_transformation, has_converged, fitness_score = imo_pcd_reader.performNDT(source_coords, target_coords, target_matrices[-1], 0.2, 0.4, 0.01, 0.1, 50)
    if fitness_score > c_d_th:
        logger.warning(f"source sec {source_idx} to target sec {target_idx}, has_converged: {has_converged}, fitness_score: {fitness_score}")
    if 2*fitness_score > c_d_th:
        max_c_d = c_d_th
    else:
        max_c_d = 2*fitness_score
    
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_coords)

    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(target_coords)

    reg_p2p = o3d.pipelines.registration.registration_generalized_icp(source = source, 
                                                            target = target, 
                                                            max_correspondence_distance = max_c_d,
                                                            init = final_transformation
                                                            )
    return reg_p2p.transformation

idx = -1

for scan_paths in pcd_segments:
    num_frames = len(scan_paths)
    if num_frames >= min_frame_th:
        idx = idx + 1

        map_idx = "map" + str(idx)
        parent_dir = os.path.abspath(os.path.join(args.data_base_dir, os.pardir))
        save_dir = os.path.join("result", os.path.basename(parent_dir), map_idx)

        if os.path.exists(save_dir):
            pose_part = "pose*.csv"
            pcs_pth_part = "*_pcs_data_path.csv"
            pose_matching = glob.glob(os.path.join(save_dir, pose_part))
            pcs_pth_matching = glob.glob(os.path.join(save_dir, pcs_pth_part))
            if pose_matching and pcs_pth_matching:
                continue

        logging.info(f">>>>>>>>>>>>>>>> Start Processing Segment {idx+1} >>>>>>>>>>>>>>>>")

        if not os.path.exists(save_dir): os.makedirs(save_dir)

        smf = args.sub_map_frames
        if num_frames < 2*smf:
            final_pose_list = subMap(scan_paths, 1)
        else:
            sec_scan_paths = []
            sectionNum = int(num_frames/smf)
            for sec in range(sectionNum):
                sec_s = sec*smf
                if sec == (sectionNum-1):
                    sec_e = num_frames
                else:
                    sec_e = (sec+1)*smf+1
                sec_scan_paths.append(scan_paths[sec_s:sec_e])
        
            with parallel_backend('loky'): results = Parallel(n_jobs=-1)(delayed(subMap)(sec_paths, for_idx) for for_idx, sec_paths in enumerate(sec_scan_paths))
            with parallel_backend('loky'): sec_results = Parallel(n_jobs=-1)(delayed(alignSections)(result, sec_paths, for_idx, results[for_idx+1], sec_scan_paths[for_idx+1], for_idx+1) 
                                                for for_idx, (result, sec_paths) in enumerate(zip(results[:-1], sec_scan_paths[:-1])))
            
            pose_list_to_stack = [results[0][:-1,:]]
            sec_comb_trans = np.eye(4)
            for for_idx, sec_comb in enumerate(sec_results):
                sec_comb_trans = sec_comb_trans.dot(sec_comb)
                if for_idx == (len(sec_results)-1):
                    tmp_sec_pose = results[for_idx+1]
                else:
                    tmp_sec_pose = results[for_idx+1][:-1,:]
                tmp_list = np.ones(tmp_sec_pose.shape)
                for i in range(tmp_sec_pose.shape[0]):
                    tmp_list[i] = sec_comb_trans.dot(tmp_sec_pose[i].reshape(4, 4)).flatten()
                pose_list_to_stack.append(tmp_list)
            
            final_pose_list = np.vstack(pose_list_to_stack)
        
        filename = "pose" + map_idx + "unoptimized.csv"
        filename = os.path.join(save_dir, filename)
        np.savetxt(filename, final_pose_list, delimiter=",")
        
        latitude_list = []
        longitude_list = []
        altitude_list = []
        for scan_path in scan_paths:
            filename = os.path.basename(scan_path)
            parts = filename.split('_')
            query_timestamp = parts[0] + '_' + parts[1]
            Latitude, Longitude, Altitude = pose.getGPS(query_timestamp)
            if Latitude is not None:
                latitude_list.append(Latitude)
                longitude_list.append(Longitude)
                altitude_list.append(Altitude)
        
        if len(latitude_list) > 0:
            mid_latitude = sum(latitude_list) / len(latitude_list)
            mid_longitude = sum(longitude_list) / len(longitude_list)
            mid_altitude = sum(altitude_list) / len(altitude_list)
            latitude_min = min(latitude_list)
            latitude_max = max(latitude_list)
            longitude_min = min(longitude_list)
            longitude_max = max(longitude_list)
            range_x, range_y, range_z = pose._gps_to_enu(latitude_min, longitude_min, mid_altitude, latitude_max, longitude_max, mid_altitude)
            scan_paths_name = str(geohash.encode(mid_latitude, mid_longitude, 12)) + "_" + str(int(range_y)) + "_" + str(int(range_x)) + "_0_pcs_data_path.csv"
        else:
            scan_paths_name = "NULL_0_0_0_pcs_data_path.csv"
        # Save scan_paths to CSV
        scan_paths_csv = os.path.join(save_dir, scan_paths_name)
        with open(scan_paths_csv, "w", newline="") as csvfile:  # Open in write mode
            writer = csv.writer(csvfile)
            writer.writerow(["File Path"])
            for path in scan_paths:
                writer.writerow([path])
        
        scan_ts = [IMU.datetime_to_timestamp(os.path.basename(scan_path)) for scan_path in scan_paths]
        
        timestamp_pose_csv = os.path.join(save_dir, "timestamp_pose.csv")
        with open(timestamp_pose_csv, "w", newline="") as csvfile:  # Open in write mode
            writer = csv.writer(csvfile)
            for for_idx, ts in enumerate(scan_ts):
                writer.writerow([ts] + final_pose_list[for_idx, :].tolist())

