#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <map>
#include <vector>
#include <algorithm>
#include "data_config.h"


// 定义一个设置结构体来存储相机名称和投影形状
struct settings {
    static std::map<std::string, cv::Size> project_shapes;
};


class FisheyeCameraProjective {
public:
    // 构造函数
    FisheyeCameraProjective(const std::string& camera_param_file, const std::string& camera_name);
    FisheyeCameraProjective(CameraParam &camera_param);
    
    // 更新去畸变
    void UpdateUndistortMaps() ;

    // 对图像进行去畸变
    cv::Mat Undistort(const cv::Mat& image) ;

    // 对图像进行投影
    cv::Mat ProjectiveTransform(const cv::Mat& image) ;

    // 根据相机方向翻转图像
    cv::Mat FlipImage(const cv::Mat& image) ;

    cv::Mat SaveUndistortPerspectiveRemp(cv::Mat &image1,int ii, std::string remap_path); 
    cv::Mat RunRemap(cv::Mat &image1,cv::Mat &new_map_x,cv::Mat &new_map_y);
private:
    // 加载相机参数
    void LoadCameraParams() ;

    std::string camera_file;
    std::string camera_name;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat resolution;
    cv::Vec2f scale_xy;
    cv::Vec2f shift_xy;
    cv::Mat undistort_maps[2];
    cv::Mat project_matrix;
    cv::Size project_shape;
};

