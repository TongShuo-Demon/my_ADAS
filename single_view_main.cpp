#include <single_view.h>
#include <iostream>
#include <chrono>
#include "avm2d_config.pb.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include <google/protobuf/util/json_util.h>
#include <filesystem>

using namespace google::protobuf;
using namespace google::protobuf::util;
namespace fs = std::filesystem;



// 从 JSON 字符串中解析消息
bool JsonToMessage(const string &json_string, Message &message) {
    JsonParseOptions options;
    Status status = JsonStringToMessage(json_string, &message, options);
    if (!status.ok()) {
        std::cerr << "Failed to parse JSON to message: " << status.ToString() << std::endl;
        return false;
    }
    return true;
}

// 从 JSON 文件中解析消息
void ParseFromJsonFile(const string &filename,  adas::Config &config) {
    // Read JSON file
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return;
    }
    std::string json_string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    // Parse JSON string into Config message
    if (!JsonToMessage(json_string, config)) {
        std::cerr << "Failed to parse JSON to Config message." << std::endl;
        return;
    }
}




int main(){


    adas::Config  single_view_config_tem;
    ParseFromJsonFile("/home/adairtong/workspace/ADAS/bin/message.json",single_view_config_tem);  
    std::cout << " ddd  " << single_view_config_tem.single_view_config().input_image_path() << std::endl;
    std::cout << " ddd  " << single_view_config_tem.single_view_config().camera_param_path() << std::endl;

    std::string image_dir = single_view_config_tem.single_view_config().input_image_path(); 
    std::string single_view_out = single_view_config_tem.single_view_config().output_image_path(); 
    std::string camera_param_path = single_view_config_tem.single_view_config().camera_param_path();

    std::string left_intrinsic_file_path = camera_param_path + "/avm_left_param.xml";
    std::string left_quaternion_file = camera_param_path + "/avm_left_transform.pb.txt";
    std::string right_intrinsic_file_path = camera_param_path + "/avm_right_param.xml";
    std::string right_quaternion_file = camera_param_path + "/avm_right_transform.pb.txt";
    std::string front_intrinsic_file_path = camera_param_path + "/avm_front_param.xml";
    std::string front_quaternion_file = camera_param_path + "/avm_front_transform.pb.txt";
    std::string rear_intrinsic_file_path = camera_param_path + "/avm_rear_param.xml";
    std::string rear_quaternion_file = camera_param_path + "/avm_rear_transform.pb.txt";




    std::vector<std::filesystem::path> fileList;
    for (const auto & entry: std::filesystem::directory_iterator(image_dir)){
        // std::cout << entry.path() << std::endl;
        fileList.push_back(entry.path());
    }
    sort(fileList.begin(), fileList.end());




    std::string timeStamp;
    for(size_t i = 0; i<fileList.size(); ++i){
        timeStamp = fileList[i].filename().string();  // 获取avm图时间戳
        
        //左路车轮视角
        single_view left_wheel_view(left_intrinsic_file_path,left_quaternion_file);
        left_wheel_view.run("left_camera");
        cv::Mat left_img = cv::imread(image_dir+"/"+timeStamp+"/left.jpg");
        cv::Mat persp_img_left ; 
        cv::remap(left_img, persp_img_left, left_wheel_view.map1_float, left_wheel_view.map2_float, cv::INTER_LINEAR);
        std::cout<< left_wheel_view.map1_float.size() << std::endl;
        std::cout<< left_wheel_view.map2_float.size() << std::endl;
        std::string left_out_path = single_view_out+"/"+timeStamp ;
        try {
            if (fs::create_directories(left_out_path)) {
                std::cout << "Directory created successfully!" << std::endl;
            } else {
                std::cout << "Directory already exists or failed to create." << std::endl;
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

        cv::imwrite(left_out_path+"/left.png",persp_img_left);

        //右路车轮视角
        single_view right_wheel_view(right_intrinsic_file_path,right_quaternion_file);
        right_wheel_view.run("right_camera");
        cv::Mat right_img = cv::imread(image_dir+"/"+timeStamp+"/right.jpg");
        cv::Mat persp_img_right ; 
        cv::remap(right_img, persp_img_right, right_wheel_view.map1_float, right_wheel_view.map2_float, cv::INTER_LINEAR);
        cv::imwrite(left_out_path+"/right.png",persp_img_right);
        //前视角
        single_view front_wheel_view(front_intrinsic_file_path,front_quaternion_file);
        front_wheel_view.run_front_back("front_camera");
        cv::Mat front_img = cv::imread(image_dir+"/"+timeStamp+"/front.jpg");
        cv::Mat result1_front;
        cv::remap(front_img, result1_front, front_wheel_view.map1_float, front_wheel_view.map2_float, cv::INTER_LINEAR);
        cv::imwrite(left_out_path+"/front.png",result1_front);

 

        //后视角
        single_view rear_wheel_view(rear_intrinsic_file_path,rear_quaternion_file);
        rear_wheel_view.run_front_back("back_camera");
        cv::Mat rear_img = cv::imread(image_dir+"/"+timeStamp+"/back.jpg");
        cv::Mat result1_rear;
        cv::remap(rear_img, result1_rear, rear_wheel_view.map1_float, rear_wheel_view.map2_float, cv::INTER_LINEAR);
        cv::imwrite(left_out_path+"/rear.png",result1_rear);
        
    }





    return 0;
}


