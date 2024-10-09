#include <single_view.h>
#include <iostream>
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;



int main(){



    std::string left_intrinsic_file_path = "/media/adairtong/adair1/ubuntu_workspace/wheel_camera/resources/changcheng_wey_vv6_ma112_dropngo3.0/component/camera/avm_left_param.xml";
    std::string left_quaternion_file = "/media/adairtong/adair1/ubuntu_workspace/wheel_camera/resources/changcheng_wey_vv6_ma112_dropngo3.0/component/camera/avm_left_transform.pb.txt";

    //左路车轮视角
    single_view left_wheel_view(left_intrinsic_file_path,left_quaternion_file);
    left_wheel_view.run("left_camera");
    cv::Mat left_img = cv::imread("/media/adairtong/adair1/ubuntu_workspace/raoxing_569-571//4lu/left/2783066715_cam_left.jpg");
    cv::Mat persp_img_left ; 
    cv::remap(left_img, persp_img_left, left_wheel_view.map1_float, left_wheel_view.map2_float, cv::INTER_LINEAR);
    std::cout<< left_wheel_view.map1_float.size() << std::endl;
    std::cout<< left_wheel_view.map2_float.size() << std::endl;
    cv::imwrite("persp_img_left.png",persp_img_left);
    //右路车轮视角
    std::string right_intrinsic_file_path = "/media/adairtong/adair1/ubuntu_workspace/wheel_camera/resources/changcheng_wey_vv6_ma112_dropngo3.0/component/camera/avm_right_param.xml";
    std::string right_quaternion_file = "/media/adairtong/adair1/ubuntu_workspace/wheel_camera/resources/changcheng_wey_vv6_ma112_dropngo3.0/component/camera/avm_right_transform.pb.txt";
    single_view right_wheel_view(right_intrinsic_file_path,right_quaternion_file);
    right_wheel_view.run("right_camera");
    cv::Mat right_img = cv::imread("/media/adairtong/adair1/ubuntu_workspace/raoxing_569-571//4lu/right/2735200045_cam_right.jpg");
    cv::Mat persp_img_right ; 
    cv::remap(right_img, persp_img_right, right_wheel_view.map1_float, right_wheel_view.map2_float, cv::INTER_LINEAR);
    cv::imwrite("persp_img_right.png",persp_img_right);

    //前路和右路都是简单做个俯仰角
    std::string front_intrinsic_file_path = "/media/adairtong/adair1/ubuntu_workspace/wheel_camera/resources/changcheng_wey_vv6_ma112_dropngo3.0/component/camera/avm_front_param.xml";
    std::string front_quaternion_file = "/media/adairtong/adair1/ubuntu_workspace/wheel_camera/resources/changcheng_wey_vv6_ma112_dropngo3.0/component/camera/avm_front_transform.pb.txt";
    single_view front_wheel_view(front_intrinsic_file_path,front_quaternion_file);
    front_wheel_view.run_front_back("front_camera");
    // 进行透视变换
    cv::Mat result1;
    cv::Mat front_img = cv::imread("/media/adairtong/adair1/ubuntu_workspace/raoxing_569-571//4lu/front/2723466707_cam_front.jpg");
    cv::warpPerspective(front_img, result1, front_wheel_view.H_front_or_back, cv::Size(1920, 1280), cv::INTER_LINEAR);
  
    // 提取前1000行
    cv::Rect roi(0, 0, result1.cols, 1000);
    cv::Mat result1_roi = result1(roi);
    cv::imwrite("persp_img_front.png", result1_roi);
 
 
    std::string back_intrinsic_file_path = "/media/adairtong/adair1/ubuntu_workspace/wheel_camera/resources/changcheng_wey_vv6_ma112_dropngo3.0/component/camera/avm_rear_param.xml";
    std::string back_quaternion_file = "/media/adairtong/adair1/ubuntu_workspace/wheel_camera/resources/changcheng_wey_vv6_ma112_dropngo3.0/component/camera/avm_rear_transform.pb.txt";
    single_view back_wheel_view(back_intrinsic_file_path,back_quaternion_file);
    back_wheel_view.run_front_back("back_camera");
    // 进行透视变换
    cv::Mat result1_rear;
    cv::Mat back_img = cv::imread("/media/adairtong/adair1/ubuntu_workspace/wheel_camera/resources/images/back.jpg");
    cv::warpPerspective(back_img, result1_rear, back_wheel_view.H_front_or_back, cv::Size(1920, 1280), cv::INTER_LINEAR);

    cv::Mat result1_roi_rear = result1_rear(roi);
    cv::imwrite("persp_img_back.png", result1_roi_rear);




    // 进行透视变换，直接这种方法会存在柱子倾斜之类的问题，因此需要对视角进行修正
    cv::Mat result2_left_st;
    cv::Mat left_st_img = cv::imread("/media/adairtong/adair1/ubuntu_workspace/translate_cpp_2davm_0623/resources/images/left.jpg");
     cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 313.33333, 0.0, 639.98117, 0.0, 313.33333, 399.964881, 0.0, 0.0, 1.0);
     cv::Mat dist_coeffs = (cv::Mat_<double>(1, 4) <<  0.129777, -0.029941, 0.007857, -0.001748);
     cv::Mat new_matrix,result;
     cv::Mat undistort_maps[2];
     new_matrix = camera_matrix.clone();
     new_matrix.at<double>(0,0) = 313.33333 ;
     new_matrix.at<double>(1,1) = 313.33333 ;
     new_matrix.at<double>(0,2) = 639.98117 ;
     new_matrix.at<double>(1,2) = 499.964881 ;

    cv::fisheye::initUndistortRectifyMap(
        camera_matrix,
        dist_coeffs,
        cv::Mat::eye(3, 3, CV_64F),
        new_matrix,
        cv::Size(1280, 800),
        CV_16SC2,
        undistort_maps[0],
        undistort_maps[1]);
    cv::remap(left_st_img, result, undistort_maps[0], undistort_maps[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::imwrite("persp_img_LEFT_st_UNDISTORT.png", result);

    


    return 0;
}


