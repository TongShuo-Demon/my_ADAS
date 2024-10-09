#pragma once
#include <opencv2/opencv.hpp>



// 定义命名空间
namespace ImageProcessing {


// 计算灰度图像在掩码下的平均值
double get_mean_statistisc(const cv::Mat &gray, const cv::Mat &mask) ;

double mean_luminance_ratio(const cv::Mat &grayA, const cv::Mat &grayB, const cv::Mat &mask) ;

// 定义调整亮度的函数
cv::Mat adjust_luminance(const cv::Mat &gray, double factor) ;

cv::Mat get_mask(const cv::Mat &img) ;

cv::Mat get_overlap_region_mask(const cv::Mat &imA, const cv::Mat &imB) ;

std::vector<cv::Point> get_outmost_polygon_boundary(const cv::Mat &img) ;
std::pair<cv::Mat, cv::Mat> get_weight_mask_matrix(const cv::Mat &imA, const cv::Mat &imB, int dist_threshold = 5) ;
// 白平衡调整函数
cv::Mat make_white_balance_u(const cv::Mat &image) ;

}