#pragma once
#include "fisheye_camera_projective.h"
#include "birdview.h"
#include <iostream>
#include <fstream>
#include <data_config.h>
#include "avm2d_config.pb.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include <google/protobuf/util/json_util.h>

using namespace google::protobuf;
using namespace google::protobuf::util;

class Avm2d
{

public:
    Avm2d();
    ~Avm2d();

    cv::Mat result_birdview_image;
    void InitByConfigFile(std::string const config_file_path);
    
    void Avm2dRun(std::vector<cv::Mat> &input_images, cv::Mat &result_image, bool isReadConfig);

private:
    void LoadCameraParams(CameraParam &camera_param, std::string const& camera_file);


     
    std::shared_ptr<FisheyeCameraProjective> fisheye_projective_transform;
    std::shared_ptr<BirdView> birdview_camera;
    adas::Config  avm2d_msg_;
    std::map<std::string, CameraParam> camera_params_;
    std::vector<cv::Mat> image_maps_;
    Avm2DConfig avm2d_config_;




};


