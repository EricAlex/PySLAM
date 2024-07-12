#! /bin/bash

downloadBucket="imo-bev-mid-data-1322593371"
uploadBucket="imo-bev-lane-1322593371"
dataset="Parking_PC_240510_chery.M3X_6083_SuZhou_1"
cloudType="cloud"
tmp_dir="tmp"
codeBase="PySLAM"
indoor=1
subMapFrameNum_outdoor=20
subMapFrameNum_indoor=80

if [[ "${dataset}" == Parking* ]]; then
    indoor=1
else
    indoor=0
fi

# # Download data

# python lidar_dataset.py --config_dir cloud.yaml --cloud_type ${cloudType} --bucket ${downloadBucket} --prefix ${dataset} --dir_path ../${tmp_dir}

# # Preprocess data, merge

# cp merge.sh ../${tmp_dir}/merge.sh
# cd ../${tmp_dir} && chmod +x merge.sh
# ./merge.sh ${dataset}
# cd ../${codeBase}

# SLAM

if [ ${indoor} -ne 0 ]; then # indoor is true
    python main_icp_slam.py --data_base_dir ../${tmp_dir}/${dataset}/sweeps --num_icp_points 40000 --sub_map_frames ${subMapFrameNum_indoor} --indoor
else
    python main_icp_slam.py --data_base_dir ../${tmp_dir}/${dataset}/sweeps --num_icp_points 40000 --sub_map_frames ${subMapFrameNum_outdoor}
fi

# Create map

map_count=0
for map in result/${dataset}/map*; do
    if [ -d "${map}" ]; then
        map_count=$((map_count + 1))
    fi
done

echo "map count: " ${map_count}

for map in result/${dataset}/map*; do
    if [ -d "${map}" ]; then
        python PcAlign.py --camera_calib_dir intrinsic_extrinsic --slam_paras_file_dir ${map} --map_count ${map_count} &
    fi
done
wait

# Upload map data

# python lidar_dataset.py --upload --config_dir cloud.yaml --cloud_type ${cloudType} --bucket ${uploadBucket} --prefix ${dataset} --dir_path result_pcd/${dataset}

# Remove raw data

# cp delete_tmp.sh ../${tmp_dir}/delete_tmp.sh
# cd ../${tmp_dir} && chmod +x delete_tmp.sh
# ./delete_tmp.sh ${dataset}
# cd ../${codeBase}

