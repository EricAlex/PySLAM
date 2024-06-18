#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <string>
#include <unordered_map>
#include <limits>
#include <random>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <fstream>
#include <chrono>
#include "omp.h"

namespace py = pybind11;
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
struct PointIMO
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t laserid;
  double timeoffset;
  float yawangle;
  uint8_t mirrorid;
  PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointIMO,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint16_t, laserid, laserid)
                                   (double, timeoffset, timeoffset)
                                   (float, yawangle, yawangle)
                                   (uint8_t, mirrorid, mirrorid)
)

struct PointMAP
{
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  float intensity;
  PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointMAP,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, intensity, intensity)
)

struct PointAL
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t segLabel;
  PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointAL,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint16_t, segLabel, segLabel)
)

Eigen::MatrixXd numpy_to_eigen(const py::array_t<double> &np_array) {
    Eigen::MatrixXd eigen_matrix(np_array.shape(0), np_array.shape(1));
    for (int i = 0; i < np_array.shape(0); ++i) {
        for (int j = 0; j < np_array.shape(1); ++j) {
            eigen_matrix(i, j) = np_array.at(i, j); 
        }
    }
    return eigen_matrix;
}

template <typename MatrixType>
MatrixType numpy_to_eigen_t(const py::array_t<double> &np_array) {
    // Ensure dimensions are valid for the specified MatrixType
    if (np_array.ndim() != 2 || 
        np_array.shape(0) != MatrixType::RowsAtCompileTime ||
        np_array.shape(1) != MatrixType::ColsAtCompileTime) {
        throw std::runtime_error("Incorrect array dimensions for the specified matrix type.");
    }

    MatrixType eigen_matrix(np_array.shape(0), np_array.shape(1)); 
    for (int i = 0; i < np_array.shape(0); ++i) {
        for (int j = 0; j < np_array.shape(1); ++j) {
            eigen_matrix(i, j) = np_array.at(i, j); 
        }
    }
    return eigen_matrix; 
}

void select_random(int N, int K, std::vector<int>& numbers) {
    numbers.resize(N + 1);
    std::iota(numbers.begin(), numbers.end(), 0); // Fill with 0, 1, 2, ... N
    // Shuffle the numbers randomly
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(numbers.begin(), numbers.end(), g);
    // Select the first K elements
    numbers.resize(K);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr numpyToPCL(py::array_t<float> &input_array) {
    // Check if the NumPy array has the correct shape (Nx3)
    if (input_array.ndim() != 2 || input_array.shape(1) != 3) {
        throw std::runtime_error("Input NumPy array should be of shape (N, 3)");
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->width = input_array.shape(0);
    cloud->height = 1; // Unorganized
    cloud->points.resize(cloud->width * cloud->height);

    auto data = input_array.unchecked(); // Access data directly
    for (size_t i = 0; i < cloud->size(); i++) {
        cloud->points[i].x = data(i, 0);
        cloud->points[i].y = data(i, 1);
        cloud->points[i].z = data(i, 2);
    }

    return cloud;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ground_plane_fitting(
        const typename pcl::PointCloud<PointT>::Ptr&source_points,
        Eigen::Vector4f& floor_coeffs)
{   
    float normal_filter_thresh = 20.0, floor_normal_thresh = 10.0;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*source_points, *height_filtered);

    std::copy_if(height_filtered->begin(), height_filtered->end(), std::back_inserter(filtered->points), [&](const pcl::PointXYZI& p) {
      return p.z < 0.5;
    });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    auto normal_filtering = [&](const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) -> pcl::PointCloud<pcl::PointXYZI>::Ptr {
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setKSearch(10);
        ne.setViewPoint(0.0f, 0.0f, 1.0f);
        ne.compute(*normals);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        filtered->reserve(cloud->size());

        for(int i = 0; i < cloud->size(); i++) {
            float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
            if(std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
                filtered->push_back(cloud->at(i));
            } else {
                out_filtered->points.emplace_back(cloud->at(i));
            }
        }

        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = false;
        return filtered;
    };
    typename pcl::PointCloud<PointT>::Ptr result_cloud(new pcl::PointCloud<PointT>);
    filtered = normal_filtering(filtered);
    if (filtered->points.size() < 10) {
        return result_cloud;
    }
    // RANSAC
    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(filtered));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
    ransac.setDistanceThreshold(0.2);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);
    // verticality check of the detected floor's normal
    Eigen::Vector4f reference = Eigen::Vector4f::UnitZ();

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);

    pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    double dot = coeffs.head<3>().dot(reference.head<3>());

    if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
        // the normal is not vertical
        return result_cloud;
    }
    // make the normal upward
    if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
      coeffs *= -1.0f;
    }
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(filtered);
    extract.setIndices(inliers);
    extract.setNegative (true);
    extract.filter(*inlier_cloud);
    floor_coeffs = Eigen::Vector4f(coeffs);

    extract.setInputCloud(filtered);
    extract.setIndices(inliers);
    extract.setNegative (false);
    extract.filter(*outlier_cloud);

    std::cout << "ransac points size: " << inlier_cloud->points.size() << std::endl;
    
    *out_filtered += *outlier_cloud;
    if (!out_filtered->points.empty() && !inlier_cloud->points.empty()) {
        auto point_to_plane_dist = [&](double x, double y, double z, const Eigen::Vector4f& floor_coeffs) -> float {
            float a = floor_coeffs[0];
            float b = floor_coeffs[1];
            float c = floor_coeffs[2];
            float d = floor_coeffs[3];
            return (a * x + b * y + c * z + d) / std::sqrt(a * a + b * b + c * c);
        };
        std::vector<float> distances;
        for (auto &p : inlier_cloud->points) {
            float dist = point_to_plane_dist(p.x, p.y, p.z, floor_coeffs);
            distances.emplace_back(dist);
        }
        float mean = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        float sum_of_squares = std::accumulate(distances.begin(), distances.end(), 0.0,
            [&mean](double sum, double val) {
                return sum + std::pow(val - mean, 2);
            }
        );
        float variance = sum_of_squares / distances.size();
        float sigma = std::sqrt(variance);
        std::cout << "sigma: " << sigma << std::endl;
        for (auto &p : out_filtered->points) {
            auto dist = point_to_plane_dist(p.x, p.y, p.z, floor_coeffs);
            if (std::fabs(dist) <= 0.2) {
                inlier_cloud->points.emplace_back(p);
            }
        }
    }
    std::cout << "result cloud points size: " << inlier_cloud->points.size() << std::endl;
    pcl::copyPointCloud(*inlier_cloud, *result_cloud);
    return result_cloud;
}

// Function to read a PCD file and convert it to a NumPy array
py::array_t<float> read_pcd(const std::string &filename) {
    pcl::PointCloud<PointIMO> cloud;

    if (pcl::io::loadPCDFile<PointIMO>(filename, cloud) == -1) {
        throw std::runtime_error("Error loading PCD file!");
    }

    // Create a NumPy array with the appropriate shape and data type
    auto result = py::array_t<float>(std::vector<size_t>{cloud.size(), 4});

    // Access NumPy array's buffer for direct writing
    auto rinfo = result.request();
    float *result_ptr = (float *)rinfo.ptr;

    // Copy data from the PCL point cloud to the NumPy array
    for (size_t i = 0; i < cloud.size(); i++) {
        result_ptr[i * 4 + 0] = cloud.points[i].x;
        result_ptr[i * 4 + 1] = cloud.points[i].y;
        result_ptr[i * 4 + 2] = cloud.points[i].z;
        result_ptr[i * 4 + 3] = cloud.points[i].intensity;
    }

    return result;
}

py::array_t<float> read_pcd_with_excluded_area(const std::string &filename,
                                               py::array_t<double> &excluded_area,
                                               double ceiling_height) {
    pcl::PointCloud<PointIMO> cloud;

    if (pcl::io::loadPCDFile<PointIMO>(filename, cloud) == -1) {
        throw std::runtime_error("Error loading PCD file!");
    }

    Eigen::MatrixXd ex_area = numpy_to_eigen(excluded_area);
    std::vector<size_t> filter_idx;
    for (size_t i = 0; i < cloud.size(); i++) {
        PointIMO pt = cloud.points[i];
        if(pt.x>ex_area(0, 0)&&pt.x<ex_area(0, 1)&&pt.y>ex_area(1, 0)&&pt.y<ex_area(1, 1))
            continue;
        if(pt.z>ceiling_height)
            continue;
        filter_idx.push_back(i);
    }

    // Create a NumPy array with the appropriate shape and data type
    auto result = py::array_t<float>(std::vector<size_t>{filter_idx.size(), 4});

    // Access NumPy array's buffer for direct writing
    auto rinfo = result.request();
    float *result_ptr = (float *)rinfo.ptr;

    // Copy data from the PCL point cloud to the NumPy array
    for (size_t i = 0; i < filter_idx.size(); i++) {
        result_ptr[i * 4 + 0] = cloud.points[filter_idx[i]].x;
        result_ptr[i * 4 + 1] = cloud.points[filter_idx[i]].y;
        result_ptr[i * 4 + 2] = cloud.points[filter_idx[i]].z;
        result_ptr[i * 4 + 3] = cloud.points[filter_idx[i]].intensity;
    }

    return result;
}

py::array_t<float> extract_ground(py::array_t<float>& input_array) {
    // Copy data from the PCL point cloud to the NumPy array
    pcl::PointCloud<PointIMO>::Ptr source_cloud(new pcl::PointCloud<PointIMO>());
    source_cloud->width = input_array.shape(0);
    source_cloud->height = 1; // Unorganized
    source_cloud->points.resize(source_cloud->width * source_cloud->height);

    auto data = input_array.unchecked(); // Access data directly
    for (size_t i = 0; i < source_cloud->size(); ++i) {
        source_cloud->points[i].x = data(i, 0);
        source_cloud->points[i].y = data(i, 1);
        source_cloud->points[i].z = data(i, 2);
        source_cloud->points[i].intensity = data(i, 3);
    }

    Eigen::Vector4f floor_coeffs;
    pcl::PointCloud<PointIMO>::Ptr ground_cloud  = ground_plane_fitting<PointIMO>(source_cloud, floor_coeffs);
    auto ground_result = py::array_t<float>(std::vector<size_t>{ground_cloud->points.size(), 4});
    auto ginfo = ground_result.request();
    float *ground_result_ptr = (float *)ginfo.ptr;
    for (size_t i = 0; i < ground_cloud->points.size(); ++i) {
        ground_result_ptr[i * 4 + 0] = ground_cloud->points[i].x;
        ground_result_ptr[i * 4 + 1] = ground_cloud->points[i].y;
        ground_result_ptr[i * 4 + 2] = ground_cloud->points[i].z;
        ground_result_ptr[i * 4 + 3] = ground_cloud->points[i].intensity;
    }
    return ground_result;
}

py::array_t<float> read_pcd_with_excluded_area_read_ratio(const std::string &filename,
                                                        py::array_t<double> &excluded_area,
                                                        double ceiling_height,
                                                        double read_ratio) {
    pcl::PointCloud<PointIMO> cloud;

    if (pcl::io::loadPCDFile<PointIMO>(filename, cloud) == -1) {
        throw std::runtime_error("Error loading PCD file!");
    }
    if(read_ratio>1){
        throw std::runtime_error("read_ratio 取值范围(0, 1)! ");
    }

    std::vector<int> rand_idx;
    int max_idx = cloud.points.size() - 1;
    select_random(max_idx, max_idx*read_ratio, rand_idx);

    Eigen::MatrixXd ex_area = numpy_to_eigen(excluded_area);
    std::vector<size_t> filter_idx;
    for(size_t i=0; i<rand_idx.size(); ++i){
        PointIMO pt = cloud.points[rand_idx[i]];
        if(pt.x>ex_area(0, 0)&&pt.x<ex_area(0, 1)&&pt.y>ex_area(1, 0)&&pt.y<ex_area(1, 1))
            continue;
        if(pt.z>ceiling_height)
            continue;
        filter_idx.push_back(rand_idx[i]);
    }

    // Create a NumPy array with the appropriate shape and data type
    auto result = py::array_t<float>(std::vector<size_t>{filter_idx.size(), 4});

    // Access NumPy array's buffer for direct writing
    auto rinfo = result.request();
    float *result_ptr = (float *)rinfo.ptr;

    // Copy data from the PCL point cloud to the NumPy array
    for (size_t i = 0; i < filter_idx.size(); i++) {
        result_ptr[i * 4 + 0] = cloud.points[filter_idx[i]].x;
        result_ptr[i * 4 + 1] = cloud.points[filter_idx[i]].y;
        result_ptr[i * 4 + 2] = cloud.points[filter_idx[i]].z;
        result_ptr[i * 4 + 3] = cloud.points[filter_idx[i]].intensity;
    }

    return result;
}

py::array_t<float> read_pcd_with_prefiltering(const std::string &filename, double resolution,
                                                double near_thresh, double far_thresh,
                                                double radius, int min_neighbors) {
    pcl::PointCloud<PointIMO>::Ptr cloud(new pcl::PointCloud<PointIMO>());
    if (pcl::io::loadPCDFile<PointIMO>(filename, *cloud) == -1) {
        throw std::runtime_error("Error loading PCD file!");
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*cloud, *source_points);
    
    // downsample
    pcl::VoxelGrid<pcl::PointXYZI>::Ptr downsample_filter(new pcl::VoxelGrid<pcl::PointXYZI>());
    downsample_filter->setLeafSize(resolution, resolution, resolution);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ds_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    downsample_filter->setInputCloud(source_points);
    downsample_filter->filter(*ds_filtered);

    // distance_filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr distance_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    distance_filtered->reserve(ds_filtered->points.size());
    std::copy_if(ds_filtered->points.begin(), ds_filtered->points.end(), std::back_inserter(distance_filtered->points), [&](const pcl::PointXYZI& p) {
      double d = p.getVector3fMap().norm();
      return d > near_thresh && d < far_thresh;
    });
    distance_filtered->width = distance_filtered->size();
    distance_filtered->height = 1;
    distance_filtered->is_dense = false;

    // outlier_removal
    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZI>::Ptr outlier_removal_filter(new pcl::RadiusOutlierRemoval<pcl::PointXYZI>());
    outlier_removal_filter->setRadiusSearch(radius);
    outlier_removal_filter->setMinNeighborsInRadius(min_neighbors);
    outlier_removal_filter->setInputCloud(distance_filtered);
    outlier_removal_filter->filter(*result_cloud);

    // Create a NumPy array with the appropriate shape and data type
    auto result = py::array_t<float>(std::vector<size_t>{result_cloud->points.size(), 4});

    // Access NumPy array's buffer for direct writing
    auto rinfo = result.request();
    float *result_ptr = (float *)rinfo.ptr;

    // Copy data from the PCL point cloud to the NumPy array
    for (size_t i = 0; i < result_cloud->points.size(); i++) {
        result_ptr[i * 4 + 0] = result_cloud->points[i].x;
        result_ptr[i * 4 + 1] = result_cloud->points[i].y;
        result_ptr[i * 4 + 2] = result_cloud->points[i].z;
        result_ptr[i * 4 + 3] = result_cloud->points[i].intensity;
    }
    return result;
}

py::array_t<float> assign_colors(py::array_t<float> &coords,
                                 std::vector<std::string> &camera_names,
                                 std::unordered_map<std::string, std::string> &img_file_pth,
                                 std::unordered_map<std::string, py::array_t<double>> &calib_intri_map,
                                 std::unordered_map<std::string, py::array_t<double>> &calib_extri_map){
    if (coords.ndim() != 2 || coords.shape(1) != 3)
        throw std::runtime_error("Coords array must have shape (N, 3)");

    auto colors = py::array_t<float>(std::vector<long>{coords.shape(0), 3});

    std::unordered_map<std::string, cv::Mat> img_data_map;
    for (auto it = img_file_pth.begin(); it != img_file_pth.end(); ++it){
        cv::Mat image = cv::imread(it->second, cv::IMREAD_COLOR);
        if (image.empty()){
            throw std::runtime_error("无法读取图片, 请检查路径! " + it->first + " / " + it->second);
            return colors;
        }
        img_data_map[it->first] = image;
    }

    std::unordered_map<std::string, Eigen::MatrixXd> intri_matrix_map, extri_matrix_map;
    for (auto it = calib_intri_map.begin(); it != calib_intri_map.end(); ++it){
        intri_matrix_map[it->first] = numpy_to_eigen(it->second);
    }
    for (auto it = calib_extri_map.begin(); it != calib_extri_map.end(); ++it){
        extri_matrix_map[it->first] = numpy_to_eigen(it->second);
    }

    long pt_size = coords.shape(0);
    auto coords_ptr = coords.unchecked();
    auto rinfo = colors.request();
    float *colors_ptr = (float *)rinfo.ptr;
    #pragma omp parallel for
    for(size_t i=0; i<pt_size; ++i){
        float min_dist = std::numeric_limits<float>::infinity();
        Eigen::Vector4d xyz_homogenous(coords_ptr(i, 0), coords_ptr(i, 1), coords_ptr(i, 2), 1.0f);
        for(auto& c_name : camera_names){
            if(img_data_map.find(c_name) != img_data_map.end()){
                Eigen::Vector4d tmp_cam = extri_matrix_map[c_name] * xyz_homogenous;
                Eigen::Vector3d p_cam(tmp_cam(0), tmp_cam(1), tmp_cam(2));
                Eigen::Vector3d p_img = intri_matrix_map[c_name] * p_cam;
                if(p_img(2)>0){
                    int x_img = int(p_img(0) / p_img(2));
                    int y_img = int(p_img(1) / p_img(2));
                    int margin_size = 50;
                    int x_min = margin_size;
                    int x_max = img_data_map[c_name].cols - margin_size;
                    int y_min = margin_size;
                    int y_max = img_data_map[c_name].rows - margin_size;
                    if( c_name == "svRightRear" || c_name == "svLeftFront" )
                        x_max = 0.75 * img_data_map[c_name].cols;
                    if( c_name == "svLeftRear" || c_name == "svRightFront" )
                        x_min = 0.25 * img_data_map[c_name].cols;
                    if( c_name == "front" || c_name == "rear" )
                        y_max = 0.8 * img_data_map[c_name].rows;
                    if( x_min <= x_img && x_img < x_max && y_min <= y_img && y_img < y_max ){
                        int centor_dist = abs(x_img - 0.5*img_data_map[c_name].cols) + abs(y_img - 0.5*img_data_map[c_name].rows);
                        if( centor_dist < min_dist ){
                            min_dist = centor_dist;
                            cv::Vec3b pixel = img_data_map[c_name].at<cv::Vec3b>(y_img, x_img);
                            colors_ptr[i * 3 + 0] = pixel[0];
                            colors_ptr[i * 3 + 1] = pixel[1];
                            colors_ptr[i * 3 + 2] = pixel[2];
                            if( c_name == "front" || c_name == "rear" )
                                break;
                        }
                    }
                }
            }
        }
    }

    return colors;
}

void save_MAP_pcd(py::array_t<float> &coords,
                  py::array_t<float> &rgbs,
                  py::array_t<float> &intensities,
                  py::array_t<float> &ground_coords,
                  py::array_t<float> &ground_intensities,
                  const std::string &filename) {
    // Ensure NumPy arrays have the right shapes and types
    if (coords.ndim() != 2 || coords.shape(1) != 3)
        throw std::runtime_error("Coords array must have shape (N, 3)");
    if (rgbs.ndim() != 2 || rgbs.shape(0) != coords.shape(0) || rgbs.shape(1) != 3)
        throw std::runtime_error("Rgbs array must have shape (N, 3)");
    if (intensities.ndim() != 1 || intensities.shape(0) != coords.shape(0))
        throw std::runtime_error("Intensities array must have shape (N,)");

    // Create PCL point cloud with intensity
    pcl::PointCloud<PointMAP> cloud; 
    cloud.width = coords.shape(0);
    cloud.height = 1; // Unorganized
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    // Fill the point cloud data
    auto coords_ptr = coords.unchecked();
    auto rgbs_ptr = rgbs.unchecked();
    auto intensities_ptr = intensities.unchecked();
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = coords_ptr(i, 0);
        cloud.points[i].y = coords_ptr(i, 1);
        cloud.points[i].z = coords_ptr(i, 2);
        cloud.points[i].r = rgbs_ptr(i, 0);
        cloud.points[i].g = rgbs_ptr(i, 1);
        cloud.points[i].b = rgbs_ptr(i, 2);
        cloud.points[i].intensity = intensities_ptr(i);
    }

    // Save the PCD file
    pcl::io::savePCDFileBinary(filename, cloud);

    // Create PCL point cloud with intensity
    pcl::PointCloud<PointMAP> ground_cloud; 
    ground_cloud.width = ground_coords.shape(0);
    ground_cloud.height = 1; // Unorganized
    ground_cloud.is_dense = false;
    ground_cloud.points.resize(ground_cloud.width * ground_cloud.height);

    // Fill the point cloud data
    auto ground_coords_ptr = ground_coords.unchecked();
    auto ground_intensities_ptr = ground_intensities.unchecked();
    for (size_t i = 0; i < ground_cloud.points.size(); ++i) {
        ground_cloud.points[i].x = ground_coords_ptr(i, 0);
        ground_cloud.points[i].y = ground_coords_ptr(i, 1);
        ground_cloud.points[i].z = ground_coords_ptr(i, 2);
        ground_cloud.points[i].intensity = ground_intensities_ptr(i);
    }

    // Save the PCD file
    pcl::io::savePCDFileBinary(filename + "_ground.pcd", ground_cloud);
}

void generate_whole_map(std::vector<std::string> &pcd_pth,
                        float save_ratio,
                        const std::string &filename, const std::string &ground_map_file) {
    pcl::PointCloud<PointMAP>::Ptr final_cloud (new pcl::PointCloud<PointMAP>);
    pcl::PointCloud<PointMAP>::Ptr final_ground_cloud (new pcl::PointCloud<PointMAP>);
    for(size_t pcd_i=0; pcd_i<pcd_pth.size(); ++pcd_i){
        pcl::PointCloud<PointMAP>::Ptr cloud (new pcl::PointCloud<PointMAP>);
        pcl::PointCloud<PointMAP>::Ptr ground_cloud(new pcl::PointCloud<PointMAP>);
        if (pcd_pth[pcd_i].find("_ground.pcd") != std::string::npos) {
            if (pcl::io::loadPCDFile<PointMAP>(pcd_pth[pcd_i], *ground_cloud) == -1) {
                throw std::runtime_error("无法读取PCD文件, 请检查路径! " + pcd_pth[pcd_i]);
                return;
            }
        } else {
            if (pcl::io::loadPCDFile<PointMAP>(pcd_pth[pcd_i], *cloud) == -1) {
                throw std::runtime_error("无法读取PCD文件, 请检查路径! " + pcd_pth[pcd_i]);
                return;
            }
        }
        
        if(save_ratio>1){
            throw std::runtime_error("save_ratio 取值范围(0, 1)! ");
            return;
        }
        std::vector<int> rand_idx;
        int max_idx = cloud->points.size() - 1;
        select_random(max_idx, max_idx*save_ratio, rand_idx);

        pcl::PointCloud<PointMAP>::Ptr DScloud(new pcl::PointCloud<PointMAP>);
        DScloud->width = rand_idx.size();
        DScloud->height = 1; // Unorganized
        DScloud->is_dense = false;
        DScloud->points.resize(DScloud->width * DScloud->height);
        for(size_t i=0; i<rand_idx.size(); ++i){
            DScloud->points[i].x = cloud->points[rand_idx[i]].x;
            DScloud->points[i].y = cloud->points[rand_idx[i]].y;
            DScloud->points[i].z = cloud->points[rand_idx[i]].z;
            DScloud->points[i].r = cloud->points[rand_idx[i]].r;
            DScloud->points[i].g = cloud->points[rand_idx[i]].g;
            DScloud->points[i].b = cloud->points[rand_idx[i]].b;
            DScloud->points[i].intensity = cloud->points[rand_idx[i]].intensity;
        }

        // Concatenate the point clouds
        *final_cloud += *DScloud;
        *final_ground_cloud += *ground_cloud;
        if(pcd_i % 10 == 0)
            std::cout<<pcd_i+1<<"/"<<pcd_pth.size()<<", ";
    }

    pcl::io::savePCDFileBinary(filename, *final_cloud);
    pcl::io::savePCDFileBinary(ground_map_file, *final_ground_cloud);
}

void stretchContrast98(cv::Mat& image) {
    // Convert to grayscale if necessary
    if (image.channels() == 3) {
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }

    // Flatten image and sort values
    std::vector<uchar> pixels;
    pixels.assign(image.datastart, image.dataend);
    std::sort(pixels.begin(), pixels.end());

    // Find 1% and 99% percentile values
    int arraySize = pixels.size();
    int minIndex = arraySize * 0.01;  // Index of 1st percentile
    int maxIndex = arraySize * 0.99;  // Index of 99th percentile
    int minValue = pixels[minIndex];
    int maxValue = pixels[maxIndex];

    // Contrast stretching
    image.convertTo(image, -1, 255.0 / (maxValue - minValue), -minValue * 255.0 / (maxValue - minValue));
}

cv::Mat stretchContrast98_float(const std::vector<std::vector<float>> &image,
                                size_t valid_pixel_c,
                                float percent) {
    size_t grid_height(0), grid_width(0);
    grid_height = image.size();
    grid_width = grid_height>0?image[0].size():grid_width;
    // Flatten image and sort values
    std::vector<float> pixels;
    for (const auto& row : image) {
        for (const auto& element : row) {
            pixels.push_back(element);
        }
    }
    std::sort(pixels.begin(), pixels.end());

    // Find 1% and 99% percentile values
    int arraySize = pixels.size();
    int minIndex = arraySize - valid_pixel_c * (1 - percent);  // Index of 1st percentile
    int maxIndex = arraySize - valid_pixel_c * percent;  // Index of 99th percentile
    float minValue = pixels[minIndex];
    float maxValue = pixels[maxIndex];

    cv::Mat intensity_mat(grid_height, grid_width, CV_8UC1); // Create an OpenCV Mat
    // Normalize and put data into the image
    for (int i = 0; i < grid_height; ++i) {
        for (int j = 0; j < grid_width; ++j) {
            float intensity = image[i][j]<minValue?minValue:image[i][j];
            intensity = intensity>maxValue?maxValue:intensity;
            intensity_mat.at<uchar>(i, j) = (uchar)(255 * (intensity - minValue) / (maxValue - minValue));
        }
    }

    return intensity_mat;
}

void generate_2d_map(const std::string &pcd_filename,
                     const std::string &ground_map_file,
                     py::array_t<float> &ranges,
                     double resolution,
                     const std::string &img_rgb,
                     const std::string &img_intensity,
                     const std::string &ground_img_intensity,
                     const std::string &height_data,
                     const std::string &ground_height_data,
                     const std::string &origin_point){
    pcl::PointCloud<PointMAP>::Ptr cloud (new pcl::PointCloud<PointMAP>);
    if (pcl::io::loadPCDFile<PointMAP> (pcd_filename, *cloud) == -1){
        throw std::runtime_error("无法读取PCD文件, 请检查路径! " + pcd_filename);
        return;
    }
    float grid_resolution = resolution;
    Eigen::Matrix2f range_mat = numpy_to_eigen_t<Eigen::Matrix2f>(ranges);
    float min_x = range_mat(0, 0);
    float max_x = range_mat(0, 1);
    float min_y = range_mat(1, 0);
    float max_y = range_mat(1, 1);
    int grid_width = int((max_x - min_x) / grid_resolution);
    int grid_height = int((max_y - min_y) / grid_resolution);
    float nanValue = nan("");
    std::vector<std::vector<float>> r_grid(grid_height, std::vector<float>(grid_width, 0.0));
    std::vector<std::vector<float>> g_grid(grid_height, std::vector<float>(grid_width, 0.0));
    std::vector<std::vector<float>> b_grid(grid_height, std::vector<float>(grid_width, 0.0));
    std::vector<std::vector<float>> intensity_grid(grid_height, std::vector<float>(grid_width, 0.0));
    std::vector<std::vector<float>> height_grid(grid_height, std::vector<float>(grid_width, nanValue));
    std::vector<std::vector<size_t>> height_grid_c(grid_height, std::vector<size_t>(grid_width, 0));
    size_t valid_pixel_c(0);
    for (const auto& point : cloud->points) {
        int y_index = int((point.x - min_x) / grid_resolution);
        int x_index = int((point.y - min_y) / grid_resolution);
        // Check if the indices are within the grid bounds
        if (x_index >= 0 && x_index < grid_width && y_index >= 0 && y_index < grid_height) {
            if(intensity_grid[y_index][x_index]==0.0){
                intensity_grid[y_index][x_index] = point.intensity;
            }else{
                intensity_grid[y_index][x_index] = std::max(intensity_grid[y_index][x_index], point.intensity);
            }
            if(isnan(height_grid[y_index][x_index])){
                valid_pixel_c++;
                height_grid[y_index][x_index] = point.z;
            }else{
                height_grid[y_index][x_index] = (height_grid_c[y_index][x_index]*height_grid[y_index][x_index] + point.z)/(height_grid_c[y_index][x_index]+1);
            }
            height_grid_c[y_index][x_index]++;
            if(r_grid[y_index][x_index]==0.0){
                r_grid[y_index][x_index] = point.r;
            }else{
                r_grid[y_index][x_index] = sqrt(0.5*(r_grid[y_index][x_index]*r_grid[y_index][x_index] + point.r*point.r));
            }
            if(g_grid[y_index][x_index]==0.0){
                g_grid[y_index][x_index] = point.g;
            }else{
                g_grid[y_index][x_index] = sqrt(0.5*(g_grid[y_index][x_index]*g_grid[y_index][x_index] + point.g*point.g));
            }
            if(b_grid[y_index][x_index]==0.0){
                b_grid[y_index][x_index] = point.b;
            }else{
                b_grid[y_index][x_index] = sqrt(0.5*(b_grid[y_index][x_index]*b_grid[y_index][x_index] + point.b*point.b));
            }
        }
    }

    pcl::PointCloud<PointMAP>::Ptr ground_cloud (new pcl::PointCloud<PointMAP>);
    if (pcl::io::loadPCDFile<PointMAP> (ground_map_file, *ground_cloud) == -1){
        throw std::runtime_error("无法读取PCD文件, 请检查路径! " + ground_map_file);
        return;
    }
    std::vector<std::vector<float>> ground_grid(grid_height, std::vector<float>(grid_width, 0.0));
    std::vector<std::vector<float>> ground_height_grid(grid_height, std::vector<float>(grid_width, nanValue));
    std::vector<std::vector<size_t>> ground_height_grid_c(grid_height, std::vector<size_t>(grid_width, 0));
    size_t valid_pixel_idx(0);
    for (const auto& point : ground_cloud->points) {
        int y_index = int((point.x - min_x) / grid_resolution);
        int x_index = int((point.y - min_y) / grid_resolution);
        // Check if the indices are within the grid bounds
        if (x_index >= 0 && x_index < grid_width && y_index >= 0 && y_index < grid_height) {
            if(ground_grid[y_index][x_index]==0.0){
                valid_pixel_idx++;
                ground_grid[y_index][x_index] = point.intensity;
            }else{
                ground_grid[y_index][x_index] = std::max(ground_grid[y_index][x_index], point.intensity);
                // ground_grid[y_index][x_index] = 0.5 * (ground_grid[y_index][x_index] + point.intensity);
            }
            if(isnan(ground_height_grid[y_index][x_index])){
                ground_height_grid[y_index][x_index] = point.z;
            }else{
                ground_height_grid[y_index][x_index] = (ground_height_grid_c[y_index][x_index]*ground_height_grid[y_index][x_index] + point.z)/(ground_height_grid_c[y_index][x_index]+1);
            }
            ground_height_grid_c[y_index][x_index]++;
        }
    }
    cv::Mat ground_intensity_mat = stretchContrast98_float(ground_grid, valid_pixel_idx, 0.03);
    cv::Mat colormapped_ground_intensity;
    cv::applyColorMap(ground_intensity_mat, colormapped_ground_intensity, cv::COLORMAP_PARULA);

    // (0, 0)
    int center_y_axis = int((0 - min_x) / grid_resolution);
    int center_x_axis = int((0 - min_y) / grid_resolution);

    // (50, 0)
    int y_axis = int((50 - min_x) / grid_resolution);
    int x_axis = int((0 - min_y) / grid_resolution);
    cv::line(colormapped_ground_intensity, cv::Point(center_x_axis, center_y_axis), cv::Point(x_axis, y_axis), cv::Scalar(0, 0, 255), 4);
    // (0, 50)
    y_axis = int((0 - min_x) / grid_resolution);
    x_axis = int((50 - min_y) / grid_resolution);
    cv::line(colormapped_ground_intensity, cv::Point(center_x_axis, center_y_axis), cv::Point(x_axis, y_axis), cv::Scalar(0, 255, 0), 4);
    cv::imwrite(ground_img_intensity, colormapped_ground_intensity);

    cv::Mat intensity_mat = stretchContrast98_float(intensity_grid, valid_pixel_c, 0.03);
    cv::Mat colormapped_intensity;
    cv::applyColorMap(intensity_mat, colormapped_intensity, cv::COLORMAP_PARULA);
    cv::imwrite(img_intensity, colormapped_intensity);

    cv::Mat rgb_image(grid_height, grid_width, CV_8UC3);
    for (int i = 0; i < grid_height; ++i) {
        for (int j = 0; j < grid_width; ++j) {
            rgb_image.at<cv::Vec3b>(i, j) = cv::Vec3b(int(r_grid[i][j]), int(g_grid[i][j]), int(b_grid[i][j]));
        }
    }
    cv::imwrite(img_rgb, rgb_image);

    std::ofstream csvFile(height_data);
    if (csvFile.is_open()) {
        for (const auto& row : height_grid) {
            for (size_t i = 0; i < row.size(); ++i) {
                csvFile << row[i];
                if (i != row.size() - 1) {
                    csvFile << ",";
                }
            }
            csvFile << "\n";
        }
        csvFile.close();
    } else {
        throw std::runtime_error("无法写文件, 请检查路径! " + height_data);
        return;
    }

    csvFile.open(ground_height_data);
    if (csvFile.is_open()) {
        for (const auto& row : ground_height_grid) {
            for (size_t i = 0; i < row.size(); ++i) {
                csvFile << row[i];
                // csvFile << 0;
                if (i != row.size() - 1) {
                    csvFile << ",";
                }
            }
            csvFile << "\n";
        }
        csvFile.close();
    } else {
        throw std::runtime_error("无法写文件, 请检查路径! " + ground_height_data);
        return;
    }

    std::ofstream opf(origin_point);
    if (opf.is_open()) {
        opf << "origin_x,origin_y,grid resolution\n";
        opf << -min_x/grid_resolution << "," << -min_y/grid_resolution << "," << grid_resolution;
        opf.close();
    } else {
        throw std::runtime_error("无法写文件, 请检查路径! " + origin_point);
        return;
    }
}

py::tuple performNDT(py::array_t<float> &source_array,
                     py::array_t<float> &target_array,
                     py::array_t<double> &init_mat,
                     float leafSize,
                     double resolution,
                     double epsilon,
                     double stepSize,
                     int maxIter)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = numpyToPCL(source_array);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = numpyToPCL(target_array);

    // Voxel grid filtering
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_src, voxel_filter_tgt;
    voxel_filter_src.setLeafSize(leafSize, leafSize, leafSize);
    voxel_filter_tgt.setLeafSize(leafSize, leafSize, leafSize);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter_src.setInputCloud(source_cloud); 
    voxel_filter_src.filter(*filtered_source_cloud);
    voxel_filter_tgt.setInputCloud(target_cloud);
    voxel_filter_tgt.filter(*filtered_target_cloud);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // ndt.setTransformationEpsilon(epsilon);
    // ndt.setStepSize(stepSize);
    // ndt.setMaximumIterations(maxIter);
    ndt.setResolution(resolution);
    ndt.setInputSource(filtered_source_cloud);
    ndt.setInputTarget(filtered_target_cloud);

    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    Eigen::Matrix4f init_guess = numpy_to_eigen_t<Eigen::Matrix4f>(init_mat);
    ndt.align(output_cloud, init_guess);

    Eigen::Matrix4f final_transformation = ndt.getFinalTransformation();
    bool has_converged = ndt.hasConverged();
    double fitness_score = ndt.getFitnessScore();

    return py::make_tuple(final_transformation, has_converged, fitness_score);
}

PYBIND11_MODULE(imo_pcd_reader, m) {
    m.doc() = "Module for reading PCD files with attributes using PyBind11";
    m.def("read_pcd", &read_pcd, "Reads a PCD file and returns a NumPy array");
    m.def("read_pcd_with_excluded_area", &read_pcd_with_excluded_area, "Reads a PCD file and returns a NumPy array outside the excluded area");
    m.def("extract_ground", &extract_ground, "extract ground");
    m.def("read_pcd_with_prefiltering", &read_pcd_with_prefiltering, "Reads a PCD file and returns a prefiltering NumPy array ");
    m.def("read_pcd_with_excluded_area_read_ratio", &read_pcd_with_excluded_area_read_ratio, "Reads a PCD file and returns a NumPy array outside the excluded area and downsample to given ratio");
    m.def("save_MAP_pcd", &save_MAP_pcd, "Save a MAP PCD file from NumPy arrays");
    m.def("performNDT", &performNDT, "Perform NDT using NumPy arrays");
    m.def("assign_colors", &assign_colors, "Assign colors for point cloud from images");
    m.def("generate_whole_map", &generate_whole_map, "Downsample and concatenate multiple frames pcd to generate the whole map");
    m.def("generate_2d_map", &generate_2d_map, "Generate X-Y palne 2d map from the whole map for labeling");
    m.def("ground_plane_fitting", &ground_plane_fitting<PointMAP>, "ground plane fitting");
    m.def("ground_plane_fitting", &ground_plane_fitting<PointIMO>, "ground plane fitting");
}