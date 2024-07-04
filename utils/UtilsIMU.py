import os
import csv
import pandas as pd
import numpy as np
import math
from datetime import datetime
import time
from scipy.spatial.transform import Rotation as R
import pymap3d as pm

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
        index, query_entry = self.find_closest_entry(query_timestamp)
        if query_entry is not None:
            columns_to_check = ['Longitude[°]', 'Latitude[°]', 'Altitude[m]']
            is_numeric = True
            for col in columns_to_check:
                try:
                    converted_col = pd.to_numeric(query_entry[col], errors='coerce')
                except ValueError:
                    is_numeric = False
            if is_numeric:
                return query_entry['Latitude[°]'], query_entry['Longitude[°]'], query_entry['Altitude[m]']
            else:
                return None, None, None
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
            for imu_index in range(start_index, end_index):
                query_entry = self.df.iloc[imu_index]
                gyro_z = query_entry['Yaw rate[°/s]'] / 180.0 * 3.1415926
                gyro_z_mean += gyro_z

                gyro_y = query_entry['Pitch rate[°/s]'] / 180.0 * 3.1415926
                gyro_y_mean += gyro_y

                gyro_x = query_entry['Roll rate[°/s]'] / 180.0 * 3.1415926
                gyro_x_mean += gyro_x

                acc_x = query_entry['Acceleration-x[m/s2]']
                acc_x_mean += acc_x

                acc_y = query_entry['Acceleration-y[m/s2]']
                acc_y_mean += acc_y

                acc_z = query_entry['Acceleration-z[m/s2]']
                acc_z_mean += acc_z

            imu_count = end_index - start_index
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

            euler = rot.as_euler('xyz', degrees=True)
            euler_angles = (0, euler[1], euler[2])
            rot_from_euler = R.from_euler('xyz', euler_angles, degrees=True)
            rot = rot_from_euler
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
            columns_to_check = ['Longitude[°]', 'Latitude[°]', 'Altitude[m]']
            is_numeric = True
            for col in columns_to_check:
                try:
                    converted_col = pd.to_numeric(query_entry[col], errors='coerce')
                except ValueError:
                    is_numeric = False
                try:
                    converted_col = pd.to_numeric(ref_entry[col], errors='coerce')
                except ValueError:
                    is_numeric = False
            if not is_numeric:
                return None
            else:
                east, north, up = self._gps_to_enu(ref_entry['Longitude[°]'], ref_entry['Latitude[°]'], ref_entry['Altitude[m]'],
                                    query_entry['Longitude[°]'], query_entry['Latitude[°]'], query_entry['Altitude[m]'])
                yaw = query_entry['Orientation[°]'] - ref_entry['Orientation[°]']
                pitch = query_entry['Pitch angle[°]'] - ref_entry['Pitch angle[°]']
                roll = query_entry['Roll angle[°]'] - ref_entry['Roll angle[°]']
                ref_Orientation = ref_entry['Orientation[°]']
                if ref_Orientation < 0:
                    ref_Orientation += 360
                enu2ref = R.from_euler('zyx', [ref_Orientation, ref_entry['Pitch angle[°]'], ref_entry['Roll angle[°]']], degrees=True).as_matrix()
                ref_speed = 5/18 * ref_entry['Speed[KPH]']
                query_speed = 5/18 * query_entry['Speed[KPH]']
                delta_time = float(query_timestamp - ref_timestamp)/1e9
                ref_trans = enu2ref @ np.array([north, east, up])
                move_dist = abs(0.5*(ref_speed + query_speed)*delta_time)
                trans_norm = np.linalg.norm(ref_trans)
                if np.isclose(trans_norm, 0, 1e-4):
                    gain = 1
                else:
                    gain = move_dist/trans_norm
                if yaw > 180:
                    yaw -= 360
                if yaw < -180:
                    yaw += 360
                T = np.eye(4)
                T[:3, :3] = R.from_euler('zyx', [-yaw, -pitch, -roll], degrees=True).as_matrix()
                # T[:3, 3] = [-ref_trans[1]*gain, -ref_trans[0]*gain, ref_trans[2]*gain]
                T[:3, 3] = [-ref_trans[1]*gain, 0, ref_trans[2]*gain]
                return T
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

    def _gps_to_enu(self, ref_lon, ref_lat, ref_alt, target_lon, target_lat, target_alt):
        enu2 = pm.geodetic2enu(target_lon, target_lat, target_alt, ref_lon, ref_lat, ref_alt)
        
        return enu2[0], enu2[1], enu2[2]
