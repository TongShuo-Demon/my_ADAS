#include "2d_avm.h"


Avm2d::Avm2d(){};


Avm2d::~Avm2d(){};

// // 从 JSON 文件读取消息
// bool LoadAvm2DConfigFromJsonFile(adas::config_path& message, const std::string& filename) {
//     std::ifstream input(filename);
//     if (!input.is_open()) {
//         std::cerr << "Failed to open file: " << filename << std::endl;
//         return false;
//     }

//     std::string json_string((std::istreambuf_iterator<char>(input)),
//                             std::istreambuf_iterator<char>());
//     input.close();

//     google::protobuf::util::JsonParseOptions options;
//     auto status = JsonStringToMessage(json_string, &message, options);
//     if (!status.ok()) {
//         std::cerr << "Failed to parse JSON to message: " << status.ToString() << std::endl;
//         return false;
//     }

//     return true;
// }
using namespace std;

// 将消息转换为 JSON 字符串
string MessageToJson(const Message &message) {
    string json_string;
    JsonPrintOptions options;
    Status status = MessageToJsonString(message, &json_string, options);
    if (!status.ok()) {
        cerr << "Failed to convert message to JSON: " << status.ToString() << endl;
        return "";
    }
    return json_string;
}

// 从 JSON 字符串中解析消息
bool JsonToMessage(const string &json_string, Message &message) {
    JsonParseOptions options;
    Status status = JsonStringToMessage(json_string, &message, options);
    if (!status.ok()) {
        cerr << "Failed to parse JSON to message: " << status.ToString() << endl;
        return false;
    }
    return true;
}

// 序列化消息到 JSON 文件
void SerializeToJsonFile(const string &filename, const adas::Config  &config) {
    // Convert messages to JSON string
    string config_json = MessageToJson(config);

    // Write JSON string to file
    ofstream file(filename);
    if (!file) {
        cerr << "Failed to open file for writing: " << filename << endl;
        return;
    }
    file << config_json;
    file.close();
}

// 从 JSON 文件中解析消息
void ParseFromJsonFile(const string &filename,  adas::Config &config) {
    // Read JSON file
    ifstream file(filename);
    if (!file) {
        cerr << "Failed to open file for reading: " << filename << endl;
        return;
    }
    string json_string((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    file.close();

    // Print JSON string for debugging
    // cout << "Read JSON string:\n" << json_string << endl;

    // Parse JSON string into Config message
    if (!JsonToMessage(json_string, config)) {
        cerr << "Failed to parse JSON to Config message." << endl;
        return;
    }
}








// 加载相机参数
void Avm2d::LoadCameraParams(CameraParam &camera_param, std::string const& camera_file) {

    cv::FileStorage fs(camera_file, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_param.intrinsic_matrix;
    fs["dist_coeffs"] >> camera_param.distortion_coefficients;
    fs["resolution"] >> camera_param.resolution;
    camera_param.resolution = camera_param.resolution.reshape(1, 2);

    cv::Mat scaleMat, shiftMat, projectMat;
    fs["scale_xy"] >> scaleMat;
    if (!scaleMat.empty()) {
        camera_param.scale_xy = cv::Vec2f(scaleMat);
    }

    fs["shift_xy"] >> shiftMat;
    if (!shiftMat.empty()) {
        camera_param.shift_xy = cv::Vec2f(shiftMat);
    }

    fs["project_matrix"] >> projectMat;
    if (!projectMat.empty()) {
        camera_param.projective_matrix = projectMat;
    }
    fs.release();

}



void Avm2d::InitByConfigFile(std::string const config_file_path){

    // // 从 JSON 文件中读取消息
    // if (!LoadAvm2DConfigFromJsonFile(avm2d_msg_, config_file_path)) {
    //     std::cerr << "Failed to load message from JSON file." << std::endl;
    // }
    ParseFromJsonFile(config_file_path,avm2d_msg_);

    std::string working_directory = std::filesystem::current_path();
    CameraParam camera_param;
    for (int i=0; i<camera_names.size(); i++) {
        std::string yaml_path = avm2d_msg_.avm_2d_config().camera_param_path() + "/" + camera_names[i] + ".yaml";
        camera_param ={};
        LoadCameraParams(camera_param, yaml_path );
        switch (i) {
            case 0:
                camera_param.crop_size = cv::Size(avm2d_msg_.avm_2d_config().birdview_width(), avm2d_msg_.avm_2d_config().yl());
                break;

            case 1:
                camera_param.crop_size = cv::Size(avm2d_msg_.avm_2d_config().birdview_width(), avm2d_msg_.avm_2d_config().yl()+30);
                break;

            case 2:
                camera_param.crop_size = cv::Size(avm2d_msg_.avm_2d_config().birdview_height(), avm2d_msg_.avm_2d_config().xl());
                break;

            case 3:
                camera_param.crop_size = cv::Size(avm2d_msg_.avm_2d_config().birdview_height(), avm2d_msg_.avm_2d_config().xl());
                break;

            default:
                break;
        }
        camera_param.camera_name = camera_names[i];
        camera_params_.insert(std::pair<std::string, CameraParam>(camera_names[i], camera_param));
        avm2d_config_.camera_params.emplace_back(camera_param);
    }
    cv::resize(cv::imread(avm2d_msg_.avm_2d_config().car_image_path()),avm2d_config_.birdview_selfcar , cv::Size(avm2d_msg_.avm_2d_config().xr()-avm2d_msg_.avm_2d_config().xl(),avm2d_msg_.avm_2d_config().yr()-avm2d_msg_.avm_2d_config().yl()),0,0,cv::INTER_LINEAR);
    avm2d_config_.xl = avm2d_msg_.avm_2d_config().xl();
    avm2d_config_.yl = avm2d_msg_.avm_2d_config().yl();
    avm2d_config_.xr = avm2d_msg_.avm_2d_config().xr();
    avm2d_config_.yr = avm2d_msg_.avm_2d_config().yr();
    avm2d_config_.birdview_width = avm2d_msg_.avm_2d_config().birdview_width();
    avm2d_config_.birdview_height = avm2d_msg_.avm_2d_config().birdview_height();
    image_maps_.clear();
    image_maps_.reserve(8);
    cv::Mat loaded_mat;
    cv::FileStorage fs2(avm2d_msg_.avm_2d_config().avm2d_remap_path(), cv::FileStorage::READ);
    fs2["front_matrix_map_x"] >> loaded_mat;
    image_maps_.emplace_back(loaded_mat);
    fs2["front_matrix_map_y"] >> loaded_mat;
    image_maps_.emplace_back(loaded_mat);
    fs2["back_matrix_map_x"] >> loaded_mat;
    image_maps_.emplace_back(loaded_mat);
    fs2["back_matrix_map_y"] >> loaded_mat;
    image_maps_.emplace_back(loaded_mat);
    fs2["left_matrix_map_x"] >> loaded_mat;
    image_maps_.emplace_back(loaded_mat);
    fs2["left_matrix_map_y"] >> loaded_mat;
    image_maps_.emplace_back(loaded_mat);
    fs2["right_matrix_map_x"] >> loaded_mat;
    image_maps_.emplace_back(loaded_mat);
    fs2["right_matrix_map_y"] >> loaded_mat;
    image_maps_.emplace_back(loaded_mat);
    fs2.release();

    cv::FileStorage fs1(avm2d_msg_.avm_2d_config().avm2d_weights_masks_path(), cv::FileStorage::READ);
    if (!fs1.isOpened()) {
        throw std::runtime_error("Could not open file for reading weights and masks.");
    }
    for (int i = 0; i < 4; ++i) // Assuming there are 4 weights and masks
    {
        cv::Mat weight, mask;
        fs1["weight_" + std::to_string(i)] >> weight;
        fs1["mask_" + std::to_string(i)] >> mask;
        avm2d_config_.merge_weights.push_back(weight);
        avm2d_config_.luminance_masks.push_back(mask);
    }

    fs2.release();
   
}


void Avm2d::Avm2dRun(std::vector<cv::Mat> &input_images, cv::Mat &result_image, bool isReadConfig ){

    cv::Mat new_map_x, new_map_y;
    std::vector<cv::Mat> projected_images;
    for (size_t i = 0; i < input_images.size(); ++i) {
        fisheye_projective_transform = std::make_shared<FisheyeCameraProjective>(camera_params_[camera_names[i]]);
        if(input_images[i].empty()){
            std::cerr << "Failed to load image: " << camera_names[i] << std::endl;
            continue;
        }
        cv::Mat warp_result;
        if(1){
            // warp_result = fisheye_projective_transform->Undistort(input_images[i]);
            // warp_result = fisheye_projective_transform->ProjectiveTransform(warp_result);
            
            warp_result = fisheye_projective_transform->SaveUndistortPerspectiveRemp(input_images[i],i,"ffff.yml");
        } else {
            new_map_x = image_maps_[i*2];
            new_map_y = image_maps_[i*2+1];
            warp_result = fisheye_projective_transform->RunRemap(input_images[i],new_map_x, new_map_y);
        }
        warp_result = fisheye_projective_transform->FlipImage(warp_result);
      
        projected_images.emplace_back(warp_result);
    }
    birdview_camera = std::make_shared<BirdView>(avm2d_config_.xl,avm2d_config_.xr,avm2d_config_.yl,avm2d_config_.yr,avm2d_config_.birdview_height,avm2d_config_.birdview_width);
    std::cout << "start stitching-----------------------------------\n";
    if(0){
        birdview_camera->get_weights_and_masks(projected_images);
        birdview_camera->update_frames(projected_images);
        // birdview_camera->make_luminance_balance(projected_images);
        birdview_camera->stitch_all_parts();
        // birdview_camera->make_white_balance();
        result_image = birdview_camera->image;

    }else{
        birdview_camera->update_frames(projected_images);
        // birdview_camera->make_luminance_balance(projected_images);
        birdview_camera->stitch_all_parts(avm2d_config_.merge_weights);
        result_image = birdview_camera->image;
        // birdview_camera->make_white_balance();
    }
    
    birdview_camera->copy_car_image(avm2d_config_.birdview_selfcar);

}