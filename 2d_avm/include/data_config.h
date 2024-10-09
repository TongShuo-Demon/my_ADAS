#pragma once

#include <opencv2/core.hpp>

extern std::vector<std::string> camera_names;

struct CameraParam {
    std::string camera_name;
    cv::Mat intrinsic_matrix;
    cv::Mat distortion_coefficients;
    cv::Mat projective_matrix;
    cv::Mat resolution;
    cv::Vec2f scale_xy;
    cv::Vec2f shift_xy;

    cv::Size crop_size;
};

struct Avm2DConfig {
    std::vector<CameraParam> camera_params;
    int xl, yl, xr, yr;
    int birdview_width, birdview_height;
    std::vector<cv::Mat> merge_weights;
    std::vector<cv::Mat> luminance_masks;
    cv::Mat birdview_selfcar;

};


struct SingleViewConfig {

};

