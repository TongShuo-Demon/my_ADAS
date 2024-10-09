#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <unordered_map>
#include <Eigen> // 引入Eigen库
#include <opencv2/opencv.hpp>
#include <cmath>


//camera intrinsic parameters, distortion coefficients, and image size
struct IntrinsicData {
    float cx;
    float cy;
    float fx;
    float fy;
    int image_height;
    int image_width;
    float k1;
    float k2;
    float k3;
    float k4;
    std::string model_type;
    float p1;
    float p2;
};



class single_view
{
private:
    Eigen::Quaterniond parse_rotation(const std::string& data) ;
    Eigen::Matrix4d construct_transform(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation);
    Eigen::Vector3d parse_translation(const std::string& data) ;
    void read_intrinsic(const std::string& intrinsic_file_pat) ;
    void construct_matrices() ; //K,D
    void read_file(const std::string& quaternion_file);
    void getLeftRealToVirualMatrix();
    void getRightRealToVirualMatrix();
    void getFrontRealToVirualMatrix();
    void getBackRealToVirualMatrix();

    //每个像素对应的归一化的三维坐标
    void get_pixel_3d_coord();
    void getFishEye3DCoord(Eigen::Matrix3d & R_left_combine);
    void distortFishEye(const cv::Mat& mat, const cv::Vec4d& D, cv::Mat& distorted_mat);
   
    IntrinsicData intrinsic_data;
    Eigen::Matrix3d K;
    Eigen::VectorXd D;
    Eigen::Matrix4d T_camera_to_world ;
    int perspective_img_h = std::floor(480);  // 透视图的高度
    int perspective_img_w = std::floor(640);  // 透视图的宽度
    double perspective_fx = 415.692;          // 透视图的横轴焦距（内参）
    double perspective_fy = 415.692;          // 透视图的纵轴焦距（内参）
    // 计算透视图的横轴和纵轴视场角（FOV）
    double perspective_FOV_x = 0;
    double perspective_FOV_y = 0;

    cv::Mat XYZ; // 3D坐标
    cv::Mat XYZ_rotated; // CV_64FC3 表示双精度浮点型，3 通道
    Eigen::Matrix3d R_left_combine;
    Eigen::Matrix3d R_right_combine;
    Eigen::Matrix3d R_front_combine;
    Eigen::Matrix3d R_back_combine;

    std::string avm_param_path ;
    std::string avm_transformer_path ;
public:
    single_view(const std::string intrinsic_distortion, const std::string extrinsic_transformer_path);
    ~single_view(){};
    void log_info();
    void SingleViewInit(std::string const config_file_path);
    void run(const std::string camera_name);
    void run_front_back(const std::string camera_name);
    cv::Mat map1_float; // 映射到原图的图像
    cv::Mat map2_float; // 映射到透视图的图像
    cv::Mat H_front_or_back; // 映射到透视图的图像

};





