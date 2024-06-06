import os
import csv
import pandas as pd
import numpy as np
import math
from datetime import datetime
import time
from scipy.spatial.transform import Rotation as R

def matrix_to_euler(matrix):
    """
    Extracts yaw, pitch, and roll angles from a transformation matrix.

    Args:
        matrix (numpy.ndarray): The 4x4 transformation matrix.

    Returns:
        A tuple containing yaw, pitch, and roll angles in radians.
    """
    R = matrix[:3, :3]  # Extract the rotation submatrix

    # Check for gimbal lock conditions
    if np.isclose(R[0, 2], 1):  # Gimbal lock at pitch = +90 degrees
        yaw = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.pi / 2  
        roll = 0
    elif np.isclose(R[0, 2], -1):  # Gimbal lock at pitch = -90 degrees
        yaw = np.arctan2(-R[2, 1], -R[2, 2])
        pitch = -np.pi / 2
        roll = 0
    else:
        # General case
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arcsin(-R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])

    return yaw, pitch, roll

def create_rotation_matrix(yaw, pitch, roll):
    """Creates a 3x3 rotation matrix from yaw, pitch, and roll angles (in radians)."""

    yaw, pitch, roll = np.radians([yaw, pitch, roll])

    # Pre-calculate sine and cosine values for efficiency
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
    cos_roll, sin_roll = np.cos(roll), np.sin(roll)

    # Construct the rotation matrix
    R_z = np.array([  # Yaw rotation
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])

    R_y = np.array([  # Pitch rotation
        [cos_pitch, 0, sin_pitch],
        [0, 1, 0],
        [-sin_pitch, 0, cos_pitch]
    ])

    R_x = np.array([  # Roll rotation
        [1, 0, 0],
        [0, cos_roll, -sin_roll],
        [0, sin_roll, cos_roll]
    ])

    # Combine the rotations (order matters!)
    R = R_z @ R_y @ R_x  # Z-Y-X order (most common for yaw-pitch-roll)

    return R

def create_transformation_matrix(yaw, pitch, roll, x, y, z):
    """Creates a 4x4 transformation matrix from yaw, pitch, roll, and XYZ translations."""

    # Convert angles to radians
    yaw, pitch, roll = np.radians([yaw, pitch, roll])

    # Rotation matrices around X, Y, and Z axes
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix
    R = np.dot(R_z, np.dot(R_y, R_x))

    # Create the transformation matrix
    T = np.eye(4)  # Initialize as identity matrix
    T[:3, :3] = R   # Insert rotation matrix
    T[:3, 3] = [x, y, z]  # Insert translation vector

    return T

def datetime_to_timestamp(dt_name):
    parts = dt_name.split('_')
    query_datetime = parts[0] + '_' + parts[1][:-3]  # Slice to remove the last 3 digits (milliseconds)
    query_dt = datetime.strptime(query_datetime, '%Y%m%d_%H%M%S') 
    seconds = query_dt.timestamp()
    milliseconds = int(parts[1][-3:])
    return int(seconds * 1e9) + milliseconds * 1e6

class IMUPose:
    def __init__(self, csv_file_path):
        self.df = pd.read_csv(csv_file_path, dtype={0: str})
        self.R = 6371000  # Earth's radius
        self.timestamp_th = 0.05 * 1e9 # nanoseconds
        self.timestamp_field = 'Gnss Posix'

    def getGPS(self, query_datetime):
        query_timestamp = datetime_to_timestamp(query_datetime)
        index ,query_entry = self.find_closest_entry(query_timestamp)
        if query_entry is not None:
            return query_entry['Latitude[°]'], query_entry['Longitude[°]'], query_entry['Altitude[m]']
        else:
            return None, None, None

    def getIMUInfo(self, ref_datetime, query_datetime):
        ref_timestamp = datetime_to_timestamp(ref_datetime)
        query_timestamp = datetime_to_timestamp(query_datetime)
        start_index, ref_entry = self.find_closest_entry(ref_timestamp)
        end_index, query_entry = self.find_closest_entry(query_timestamp)

        if ref_entry is not None and query_entry is not None:
            acc_x_mean = 0.0
            acc_y_mean = 0.0
            acc_z_mean = 0.0
            gyro_x_mean = 0.0
            gyro_y_mean = 0.0
            gyro_z_mean = 0.0
            for imu_index in range(start_index, end_index + 1):
                query_entry = self.df.iloc[imu_index]
                gyro_z = query_entry['Yaw rate[°/s]']
                gyro_z_mean += gyro_z

                gyro_y = query_entry['Pitch rate[°/s]']
                gyro_y_mean += gyro_y

                gyro_x = query_entry['Roll rate[°/s]']
                gyro_x_mean += gyro_x

                acc_x = query_entry['Acceleration-x[m/s2]']
                acc_x_mean += acc_x

                acc_y = query_entry['Acceleration-y[m/s2]']
                acc_y_mean += acc_y

                acc_z = query_entry['Acceleration-z[m/s2]']
                acc_z_mean += acc_z

            imu_count = end_index - start_index + 1
            acc_x_mean /= imu_count
            acc_y_mean /= imu_count
            acc_z_mean /= imu_count
            gyro_x_mean /= imu_count
            gyro_y_mean /= imu_count
            gyro_z_mean /= imu_count

            scan_period = 0.1
            T = np.eye(4)  # Initialize as identity matrix
            quaternion = np.array([1, gyro_x_mean * scan_period / 2, gyro_y_mean * scan_period / 2, gyro_z_mean * scan_period / 2])
            rot = R.from_quat(quaternion)
            rotation_matrix = rot.as_matrix()
            T[:3, :3] = rotation_matrix
            return T
        else:
            return None

    def getTransformationMatrix(self, ref_datetime, query_datetime):
        ref_timestamp = datetime_to_timestamp(ref_datetime)
        query_timestamp = datetime_to_timestamp(query_datetime)
        ref_index, ref_entry = self.find_closest_entry(ref_timestamp)
        query_index, query_entry = self.find_closest_entry(query_timestamp)
        if ref_entry is not None and query_entry is not None:
            # x, y, z = self._gps_to_enu(ref_entry['Longitude[°]'], ref_entry['Latitude[°]'], ref_entry['Altitude[m]'],
            #                     query_entry['Longitude[°]'], query_entry['Latitude[°]'], query_entry['Altitude[m]'])
            # yaw = query_entry['Orientation[°]'] - ref_entry['Orientation[°]']
            # pitch = query_entry['Pitch angle[°]'] - ref_entry['Pitch angle[°]']
            # roll = query_entry['Roll angle[°]'] - ref_entry['Roll angle[°]']
            # enu2ref = create_rotation_matrix(ref_entry['Orientation[°]'], ref_entry['Pitch angle[°]'], ref_entry['Roll angle[°]'])
            # ref_trans = np.dot(enu2ref, np.array([x, y, z]))
            # return create_transformation_matrix(-yaw, -pitch, -roll, -ref_trans[0], -ref_trans[1], -ref_trans[2])

            x, y, z = self._gps_to_enu(ref_entry['Longitude[°]'], ref_entry['Latitude[°]'], ref_entry['Altitude[m]'],
                                query_entry['Longitude[°]'], query_entry['Latitude[°]'], query_entry['Altitude[m]'])
            yaw = query_entry['Orientation[°]'] - ref_entry['Orientation[°]']
            pitch = query_entry['Pitch angle[°]'] - ref_entry['Pitch angle[°]']
            roll = query_entry['Roll angle[°]'] - ref_entry['Roll angle[°]']
            enu2ref = create_rotation_matrix(ref_entry['Orientation[°]'], ref_entry['Pitch angle[°]'], ref_entry['Roll angle[°]'])
            ref_speed = 5/18 * ref_entry['Speed[KPH]']
            query_speed = 5/18 * query_entry['Speed[KPH]']
            delta_time = float(query_timestamp - ref_timestamp)/1e9
            ref_trans = np.dot(enu2ref, np.array([x, y, z]))
            move_dist = abs(0.5*(ref_speed + query_speed)*delta_time)
            trans_norm = np.linalg.norm(ref_trans)
            if np.isclose(trans_norm, 0, 1e-4):
                gain = 1
            else:
                gain = move_dist/trans_norm
            return create_transformation_matrix(-yaw, -pitch, -roll, -ref_trans[0]*gain, -ref_trans[1]*gain, -ref_trans[2]*gain)
        else:
            return None

    def find_closest_entry(self, timestamp):
        closest_index = self.df[self.timestamp_field].searchsorted(timestamp)  # Binary search

        # Handle potential out-of-bounds index
        if closest_index == 0:
            tooEarly = self.df.iloc[0]
            if abs(tooEarly[self.timestamp_field] - timestamp) < self.timestamp_th:
                return closest_index, tooEarly
            else:
                return None, None
        if closest_index == len(self.df):
            tooLate = self.df.iloc[-1]
            if abs(tooLate[self.timestamp_field] - timestamp) < self.timestamp_th:
                return closest_index, tooLate
            else:
                return None, None

        before = self.df.iloc[closest_index - 1]
        after = self.df.iloc[closest_index]
        if abs(before[self.timestamp_field] - timestamp) < abs(after[self.timestamp_field] - timestamp):
            if abs(before[self.timestamp_field] - timestamp) < self.timestamp_th:
                return closest_index - 1, before
            else:
                return None, None
        else:
            if abs(after[self.timestamp_field] - timestamp) < self.timestamp_th:
                return closest_index, after
            else:
                return None, None

    def _gps_to_enu(self, lat0, lon0, h0, lat, lon, h):
        # Convert degrees to radians
        lat0_rad = math.radians(lat0)
        lon0_rad = math.radians(lon0)
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        dlat = lat_rad - lat0_rad
        dlon = lon_rad - lon0_rad
        
        x = self.R * dlon * math.cos(lat0_rad)
        y = self.R * dlat
        z = h - h0
        return x, y, z
