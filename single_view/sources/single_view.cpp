#include <single_view.h>






single_view::single_view(const std::string intrinsic_distortion, const std::string extrinsic_transformer_path){
    avm_param_path = intrinsic_distortion;
    avm_transformer_path = extrinsic_transformer_path;

    perspective_img_h = std::floor(480);  // 透视图的高度
    perspective_img_w = std::floor(640);  // 透视图的宽度
    perspective_fx = 415.692;          // 透视图的横轴焦距（内参）
    perspective_fy = 415.692;          // 透视图的纵轴焦距（内参）

    // 计算透视图的横轴和纵轴视场角（FOV）
    perspective_FOV_x = 2.0 * std::atan(perspective_img_w / (2.0 * perspective_fx)) * 180.0 / M_PI;
    perspective_FOV_y = 2.0 * std::atan(perspective_img_h / (2.0 * perspective_fy)) * 180.0 / M_PI;
  }

void SingleViewInit(std::string const config_file_path){
    
}







void single_view::read_intrinsic(const std::string& intrinsic_file_pat) {

    // Open XML file
    std::ifstream file(intrinsic_file_pat);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + intrinsic_file_pat);
    }

    std::string line;
    std::string xml_content;
    while (std::getline(file, line)) {
        xml_content += line;
    }
    file.close();

    // Find and extract values from XML content
    size_t start_pos, end_pos;

    // cx
    start_pos = xml_content.find("<cx>") + 4;
    end_pos = xml_content.find("</cx>");
    intrinsic_data.cx = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

    // cy
    start_pos = xml_content.find("<cy>") + 4;
    end_pos = xml_content.find("</cy>");
    intrinsic_data.cy = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

    // fx
    start_pos = xml_content.find("<fx>") + 4;
    end_pos = xml_content.find("</fx>");
    intrinsic_data.fx = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

    // fy
    start_pos = xml_content.find("<fy>") + 4;
    end_pos = xml_content.find("</fy>");
    intrinsic_data.fy = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

    // image_height
    start_pos = xml_content.find("<image_height>") + 14;
    end_pos = xml_content.find("</image_height>");
    intrinsic_data.image_height = std::stoi(xml_content.substr(start_pos, end_pos - start_pos));

    // image_width
    start_pos = xml_content.find("<image_width>") + 13;
    end_pos = xml_content.find("</image_width>");
    intrinsic_data.image_width = std::stoi(xml_content.substr(start_pos, end_pos - start_pos));

    // k1
    start_pos = xml_content.find("<k1>") + 4;
    end_pos = xml_content.find("</k1>");
    intrinsic_data.k1 = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

    // k2
    start_pos = xml_content.find("<k2>") + 4;
    end_pos = xml_content.find("</k2>");
    intrinsic_data.k2 = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

    // k3
    start_pos = xml_content.find("<k3>") + 4;
    end_pos = xml_content.find("</k3>");
    intrinsic_data.k3 = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

    // k4
    start_pos = xml_content.find("<k4>") + 4;
    end_pos = xml_content.find("</k4>");
    intrinsic_data.k4 = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

    // model_type
    start_pos = xml_content.find("<model_type>") + 12;
    end_pos = xml_content.find("</model_type>");
    intrinsic_data.model_type = xml_content.substr(start_pos, end_pos - start_pos);

    // p1
    start_pos = xml_content.find("<p1>") + 4;
    end_pos = xml_content.find("</p1>");
    intrinsic_data.p1 = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

    // p2
    start_pos = xml_content.find("<p2>") + 4;
    end_pos = xml_content.find("</p2>");
    intrinsic_data.p2 = std::stof(xml_content.substr(start_pos, end_pos - start_pos));

}


// 根据输入的相机内参数据构造相机矩阵K和畸变向量D
void single_view::construct_matrices() {
    // 构造相机矩阵K
    K << intrinsic_data.fx, 0, intrinsic_data.cx,
         0, intrinsic_data.fy, intrinsic_data.cy,
         0, 0, 1;

    // 构造畸变向量D
    D.resize(4);
    D << intrinsic_data.k1, intrinsic_data.k2, intrinsic_data.k3, intrinsic_data.k4;
}

// 函数：从数据中解析平移向量
Eigen::Vector3d single_view::parse_translation(const std::string& data) {
    Eigen::Vector3d translation;
    std::istringstream iss(data);
    std::string token;
    
    // 跳过第一行（假设为 "translation {"）
    std::getline(iss, token);
    
    // 读取 x、y、z 值
    std::getline(iss, token, ':'); // 读取 "x:"
    std::getline(iss, token, '\n'); // 读取 x 值
    translation[0] = std::stod(token);
    
    std::getline(iss, token, ':'); // 读取 "y:"
    std::getline(iss, token, '\n'); // 读取 y 值
    translation[1] = std::stod(token);
    
    std::getline(iss, token, ':'); // 读取 "z:"
    std::getline(iss, token, '\n'); // 读取 z 值
    translation[2] = std::stod(token);
    
    return translation;
}

// 函数：从数据中解析旋转四元数
Eigen::Quaterniond single_view::parse_rotation(const std::string& data) {
    Eigen::Quaterniond rotation;
    std::istringstream iss(data);
    std::string token;
    
    // 跳过第一行（假设为 "rotation {"）
    std::getline(iss, token);
    
    // 读取 x、y、z、w 值
    std::getline(iss, token, ':'); // 读取 "x:"
    std::getline(iss, token, '\n'); // 读取 x 值
    double x = std::stod(token);
    
    std::getline(iss, token, ':'); // 读取 "y:"
    std::getline(iss, token, '\n'); // 读取 y 值
    double y = std::stod(token);
    
    std::getline(iss, token, ':'); // 读取 "z:"
    std::getline(iss, token, '\n'); // 读取 z 值
    double z = std::stod(token);
    
    std::getline(iss, token, ':'); // 读取 "w:"
    std::getline(iss, token, '\n'); // 读取 w 值
    double w = std::stod(token);
    
    rotation = Eigen::Quaterniond(w, x, y, z); // 构造四元数
    
    return rotation;
}

// 函数：根据平移和旋转构造变换矩阵
Eigen::Matrix4d single_view::construct_transform(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    // 设置旋转矩阵
    T.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
    
    // 设置平移向量
    T.block<3, 1>(0, 3) = translation;
    
    return T;
}

// 函数：从文件中读取数据得到外参矩阵
void single_view::read_file(const std::string& quaternion_file) {
    std::ifstream file(quaternion_file);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + quaternion_file);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    // 查找平移和旋转数据
    size_t translation_pos = buffer.str().find("translation");
    size_t rotation_pos = buffer.str().find("rotation");

    std::string translation_data = buffer.str().substr(translation_pos, rotation_pos - translation_pos);
    std::string rotation_data = buffer.str().substr(rotation_pos);
    // 解析平移向量
    Eigen::Vector3d translation = parse_translation(translation_data);
    // 解析旋转四元数
    Eigen::Quaterniond rotation = parse_rotation(rotation_data);
    // 构造变换矩阵
    T_camera_to_world = construct_transform(translation, rotation);

}

 //每个像素对应的归一化的三维坐标
void single_view::get_pixel_3d_coord(){
    // 生成 X 坐标
    cv::Mat X(perspective_img_h, perspective_img_w, CV_64F);
    for (int i = 0; i < perspective_img_h; ++i) {
        for (int j = 0; j < perspective_img_w; ++j) {
            X.at<double>(i, j) = (((double)j / perspective_img_w) * 2 - 1) * std::tan(M_PI * perspective_FOV_x / 360.0);
        }
    }

    // 生成 Y 坐标
    cv::Mat Y(perspective_img_h, perspective_img_w, CV_64F);
    for (int i = 0; i < perspective_img_h; ++i) {
        for (int j = 0; j < perspective_img_w; ++j) {
            Y.at<double>(i, j) = (((double)i / perspective_img_h) * 2 - 1) * std::tan(M_PI * perspective_FOV_y / 360.0);
        }
    }

    // 生成 Z 坐标
    cv::Mat Z = cv::Mat::ones(perspective_img_h, perspective_img_w, CV_64F);

    // 合并 X, Y, Z 到 XYZ
    std::vector<cv::Mat> channels = { X, Y, Z };
    
    cv::merge(channels, XYZ);

}



void single_view::getLeftRealToVirualMatrix(){
      // 定义旋转矩阵
        Eigen::Matrix3d R_world2standard;
        R_world2standard << 1, 0, 0,
                            0, 0, -1,
                            0, 1, 0;

        Eigen::Matrix3d camera_to_world; // 假设已定义并填充了 camera_to_world 的内容
        // 取前三行三列作为 Eigen::Matrix3d
        camera_to_world = T_camera_to_world.block<3, 3>(0, 0);

        // 计算 R_camera2standard
        Eigen::Matrix3d R_camera2standard = R_world2standard * camera_to_world;

        // 定义 yaw、pitch、roll 的角度
        double yaw = 60.0 / 180.0 * M_PI;
        double pitch = -40.0 / 180.0 * M_PI;
        double roll = 10.0 / 180.0 * M_PI;

        // 定义旋转矩阵 R_yaw、R_pitch、R_roll
        Eigen::Matrix3d R_yaw;
        R_yaw << cos(yaw), 0, -sin(yaw),
                0, 1, 0,
                sin(yaw), 0, cos(yaw);

        Eigen::Matrix3d R_pitch;
        R_pitch << 1, 0, 0,
                0, cos(pitch), sin(pitch),
                0, -sin(pitch), cos(pitch);

        Eigen::Matrix3d R_roll;
        R_roll << cos(roll), -sin(roll), 0,
                sin(roll), cos(roll), 0,
                0, 0, 1;
        // 计算 R_combine
        R_left_combine = (R_pitch * R_yaw * R_camera2standard).inverse();

}



void single_view::getRightRealToVirualMatrix(){
    // 定义并初始化 R_world2standard
    Eigen::Matrix3d R_world2standard;
    R_world2standard << -1, 0, 0,
                        0, 0, -1,
                        0, -1, 0;

    // 假设 camera_to_world 是一个 4x4 矩阵，这里仅使用其 3x3 部分
    Eigen::Matrix3d  camera_to_world; // 您需要根据实际数据初始化该矩阵
    camera_to_world = T_camera_to_world.block<3, 3>(0, 0);

    // 计算 R_camera2standard
    Eigen::Matrix3d R_camera2standard = R_world2standard * camera_to_world;

    // 定义并计算 R_yaw
    double yaw = -60.0 / 180.0 * M_PI;
    Eigen::Matrix3d R_yaw;
    R_yaw << cos(yaw), 0, -sin(yaw),
             0, 1, 0,
             sin(yaw), 0, cos(yaw);

    // 定义并计算 R_pitch
    double pitch = -40.0 / 180.0 * M_PI;
    Eigen::Matrix3d R_pitch;
    R_pitch << 1, 0, 0,
               0, cos(pitch), sin(pitch),
               0, -sin(pitch), cos(pitch);

    // 定义并计算 R_roll
    double roll = 10.0 / 180.0 * M_PI;
    Eigen::Matrix3d R_roll;
    R_roll << cos(roll), -sin(roll), 0,
              sin(roll), cos(roll), 0,
              0, 0, 1;

    // 计算 R_combine
    R_right_combine = (R_pitch * R_yaw * R_camera2standard).inverse();
}


void single_view::getFrontRealToVirualMatrix(){
    // 定义并初始化 R_world2standard
    Eigen::Matrix3d R_world2standard;
    R_world2standard << 0, -1, 0,
                        0, 0, -1,
                        1, 0, 0;

    // 假设 camera_to_world 是一个 4x4 矩阵，这里仅使用其 3x3 部分
    Eigen::Matrix3d camera_to_world; // 您需要根据实际数据初始化该矩阵
    camera_to_world = T_camera_to_world.block<3, 3>(0, 0);

    // 计算 R_camera2standard
    Eigen::Matrix3d R_camera2standard = R_world2standard * camera_to_world;

    // 定义并计算 R_pitch
    double pitch = -10/ 180.0 * M_PI;
    Eigen::Matrix3d R_pitch;
    R_pitch << 1, 0, 0,
               0, cos(pitch), sin(pitch),
               0, -sin(pitch), cos(pitch);


    // 计算 R_combine
    // R_front_combine = (R_pitch * R_camera2standard);
    R_front_combine = (R_pitch * R_camera2standard).inverse();
    // std::cout << "R_front_combine = " << std::endl << R_front_combine << std::endl;

}



void single_view::getBackRealToVirualMatrix(){
    // 定义并初始化 R_world2standard
    Eigen::Matrix3d R_world2standard;
    R_world2standard << 0, 1, 0,
                        0, 0, -1,
                        -1, 0, 0;

    // 假设 camera_to_world 是一个 4x4 矩阵，这里仅使用其 3x3 部分
    Eigen::Matrix3d camera_to_world; // 您需要根据实际数据初始化该矩阵
    camera_to_world = T_camera_to_world.block<3, 3>(0, 0);

    // 计算 R_camera2standard
    Eigen::Matrix3d R_camera2standard = R_world2standard * camera_to_world;

    // 定义并计算 R_pitch
    double pitch = -10 / 180.0 * M_PI;
    Eigen::Matrix3d R_pitch;
    R_pitch << 1, 0, 0,
               0, cos(pitch), sin(pitch),
               0, -sin(pitch), cos(pitch);

    // 计算 R_combine
    R_back_combine = (R_pitch  * R_camera2standard);

}

#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;


void single_view::getFishEye3DCoord(Eigen::Matrix3d & R_left_combine){
        Eigen::Matrix3d R_combine;
        R_combine = R_left_combine;
        // std::cout << "R_combine = " << std::endl << R_combine << std::endl;
        // 将 XYZ 的数据复制到 Eigen::MatrixXd
        Eigen::Matrix<double, 3, Eigen::Dynamic> mat_eigen(3, XYZ.cols * XYZ.rows);
        for (int y = 0; y < XYZ.rows; ++y) {
            for (int x = 0; x < XYZ.cols; ++x) {
                cv::Vec3d pixel = XYZ.at<cv::Vec3d>(y, x);
                mat_eigen.col(y * XYZ.cols + x) << pixel[0], pixel[1], pixel[2];
            }
        }
        // 输出转换后的 Eigen 矩阵
        Eigen::MatrixXd rotated_eigen = (R_combine * mat_eigen).transpose();
        // std::cout << "mat_eigen = " << std::endl << mat_eigen << std::endl;
        // 遍历每一行，对最后一列进行除法操作
        for (int i = 0; i < rotated_eigen.rows(); ++i) {
            rotated_eigen(i, 1) /= rotated_eigen(i, 2); // 每行的最后一列除以其自身值
            rotated_eigen(i, 0) /= rotated_eigen(i, 2); // 每行的最后一列除以其自身值
            rotated_eigen(i, 2) /= rotated_eigen(i, 2); // 每行的最后一列除以其自身值

        }
        XYZ_rotated.create(XYZ.rows, XYZ.cols, CV_64FC3); 
        // 将 Eigen 矩阵数据复制到 cv::Mat 中
        // 假设 mat 是按行存储的，按照图像的行优先顺序进行复制
        for (int y = 0; y < XYZ.rows; ++y) {
            for (int x = 0; x < XYZ.cols; ++x) {
                XYZ_rotated.at<cv::Vec3d>(y, x) = cv::Vec3d(rotated_eigen(y * XYZ.cols + x, 0), rotated_eigen(y * XYZ.cols + x, 1), rotated_eigen(y * XYZ.cols + x, 2));
            }
        }
        // std::cout << "XYZ_rotated = " << std::endl << XYZ_rotated << std::endl;
}






// 计算鱼眼图像的畸变
void single_view::distortFishEye(const cv::Mat& mat, const cv::Vec4d& D, cv::Mat& distorted_mat) {
    int rows = mat.rows;
    int cols = mat.cols;

    distorted_mat.create(rows, cols, CV_64FC1);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            double xx = mat.at<cv::Vec3d>(y, x)[0];
            double yy = mat.at<cv::Vec3d>(y, x)[1];

            double r = std::sqrt(xx * xx + yy * yy);
            double theta = std::atan(r);
            double theta2 = theta * theta;
            double theta4 = theta2 * theta2;
            double theta6 = theta4 * theta2;
            double theta8 = theta4 * theta4;

            double theta_d = theta * (1 + D[0] * theta2 + D[1] * theta4 + D[2] * theta6 + D[3] * theta8);
            double scale = theta_d / r;

            distorted_mat.at<double>(y, x) = scale;

        }
    }
}


void single_view::log_info(){
    std::cout << "\033[31m" << "\n\n###############################################################################################\n"; 

    // 输出变换矩阵 T
    std::cout << "变换矩阵 T:" << std::endl;
    std::cout << T_camera_to_world << std::endl;

    std::cout << "K = " << std::endl << K << std::endl;
    std::cout << "D = " << std::endl << D << std::endl;

    // Print out the read data
    std::cout << "cx: " << intrinsic_data.cx << std::endl;
    std::cout << "cy: " << intrinsic_data.cy << std::endl;
    std::cout << "fx: " << intrinsic_data.fx << std::endl;
    std::cout << "fy: " << intrinsic_data.fy << std::endl;
    std::cout << "image_height: " << intrinsic_data.image_height << std::endl;
    std::cout << "image_width: " << intrinsic_data.image_width << std::endl;
    std::cout << "k1: " << intrinsic_data.k1 << std::endl;
    std::cout << "k2: " << intrinsic_data.k2 << std::endl;
    std::cout << "k3: " << intrinsic_data.k3 << std::endl;
    std::cout << "k4: " << intrinsic_data.k4 << std::endl;
    std::cout << "model_type: " << intrinsic_data.model_type << std::endl;
    std::cout << "p1: " << intrinsic_data.p1 << std::endl;
    std::cout << "p2: " << intrinsic_data.p2 << std::endl;
    std::cout << "###############################################################################################\n " << "\033[0m"; 

}
void single_view::run(const std::string camera_name){

    read_intrinsic(avm_param_path); //get K, D , intrinsic_data
    construct_matrices(); //get K, D
    read_file(avm_transformer_path); //get T_camera_to_world
    get_pixel_3d_coord(); //get XYZ

    if(camera_name == "left_camera"){
        getLeftRealToVirualMatrix(); //get R_left_combine
            high_resolution_clock::time_point beginTime = high_resolution_clock::now();
        getFishEye3DCoord(R_left_combine); //get XYZ_rotated
                high_resolution_clock::time_point endTime = high_resolution_clock::now();
    milliseconds timeInterval = std::chrono::duration_cast<milliseconds>(endTime - beginTime);
    // std::cout <<"left_camera FISHEYE : " <<  timeInterval.count() << "ms\n";

    }
    else if(camera_name == "right_camera"){
        getRightRealToVirualMatrix(); //get R_left_combine
        getFishEye3DCoord(R_right_combine); //get XYZ_rotated
    }
    else{
        std::cout << "Error: camera_name should be left_camera or right_camera" << std::endl;
    }
    cv::Mat distorted_mat;
    cv::Vec4d cvVec_D = cv::Vec4d(D[0], D[1], D[2], D[3]);
    distortFishEye(XYZ_rotated, cvVec_D, distorted_mat); //get distorted_mat
    cv::Mat single_x_dst(XYZ_rotated.rows, XYZ_rotated.cols, CV_64FC1); 
    for (int y = 0; y < XYZ_rotated.rows; ++y) {
        for (int x = 0; x < XYZ_rotated.cols; ++x) {
            single_x_dst.at<double>(y, x) = XYZ_rotated.at<cv::Vec3d>(y, x)[0];
        }
    }
    cv::Mat map1 = single_x_dst.mul(distorted_mat)* K(0,0)+ K(0,2);
    map1.convertTo(map1_float, CV_32FC1); 


    cv::Mat single_y_dst(XYZ_rotated.rows, XYZ_rotated.cols, CV_64FC1);
    for (int y = 0; y < XYZ_rotated.rows; ++y) {
        for (int x = 0; x < XYZ_rotated.cols; ++x) {
            single_y_dst.at<double>(y, x) = XYZ_rotated.at<cv::Vec3d>(y, x)[1];
        }
    }
    cv::Mat map2 = single_y_dst.mul(distorted_mat)* K(1,1)+ K(1,2);
    map2.convertTo(map2_float, CV_32FC1); 
}

// void single_view:: run_front_back(const std::string camera_name){

//     read_intrinsic(avm_param_path); //get K, D , intrinsic_data
//     construct_matrices(); //get K, D
//     read_file(avm_transformer_path); //get T_camera_to_world
//     Eigen::Matrix3d R_combine;

//     Eigen::Matrix3d R_world2standard;
//     if(camera_name == "front_camera"){
//         getFrontRealToVirualMatrix(); 
//         R_combine = R_front_combine;
//         R_world2standard << 0, -1, 0,
//                     0, 0, -1,
//                     1, 0, 0;
//     }
//     else if(camera_name == "back_camera"){
//         getBackRealToVirualMatrix(); 
//         R_combine = R_back_combine;
//         R_world2standard << 0, 1, 0,
//                             0, 0, -1,
//                             -1, 0, 0;
//     }
//     else{
//         std::cout << "Error: camera_name should be front_camera or rear_camera" << std::endl;
//     }

//     // 定义并初始化 n 向量
//     Eigen::Vector3d n;
//     n << 0, 0, 1;
//     // 假设 camera_to_world 是一个 4x4 矩阵，这里仅使用其 3x3 部分
//     Eigen::Matrix3d camera_to_world; // 您需要根据实际数据初始化该矩阵
//     camera_to_world = T_camera_to_world.block<3, 3>(0, 0);

//     // 计算 R_camera2standard
//     Eigen::Matrix3d R_camera2standard = R_world2standard * camera_to_world;

//     n = R_camera2standard.inverse() * n;

//     // 定义 t_real2standard 向量
//     Eigen::Vector3d t_real2standard;
//     t_real2standard << 0, 1, 0;

//     // 定义 d
//     double d = 10.0;
//     // 计算单应性矩阵 H
//     Eigen::Matrix3d H_eigen = K * (R_combine + (t_real2standard / d) * n.transpose()) * K.inverse();
//      // 将Eigen矩阵转换为OpenCV矩阵
//     H_front_or_back.create(3, 3, CV_64F);
//     for (int i = 0; i < 3; ++i)
//         for (int j = 0; j < 3; ++j)
//             H_front_or_back.at<double>(i, j) = H_eigen(i, j);


// }


// void single_view:: run_front_back(const std::string camera_name){

//     read_intrinsic(avm_param_path); //get K, D , intrinsic_data
//     construct_matrices(); //get K, D
//     read_file(avm_transformer_path); //get T_camera_to_world
//     Eigen::Matrix3d R_combine;

//     Eigen::Matrix3d R_world2standard;
//     if(camera_name == "front_camera"){
//         getFrontRealToVirualMatrix(); 
//         R_combine = R_front_combine;
//         R_world2standard << 0, -1, 0,
//                     0, 0, -1,
//                     1, 0, 0;
//     }
//     else if(camera_name == "back_camera"){
//         getBackRealToVirualMatrix(); 
//         R_combine = R_back_combine;
//         R_world2standard << 0, 1, 0,
//                             0, 0, -1,
//                             -1, 0, 0;
//     }
//     else{
//         std::cout << "Error: camera_name should be front_camera or rear_camera" << std::endl;
//     }

//     // 定义并初始化 n 向量
//     Eigen::Vector3d n;
//     n << 0, 0, 1;
//     // 假设 camera_to_world 是一个 4x4 矩阵，这里仅使用其 3x3 部分
//     Eigen::Matrix3d camera_to_world; // 您需要根据实际数据初始化该矩阵
//     camera_to_world = T_camera_to_world.block<3, 3>(0, 0);

//     // 计算 R_camera2standard
//     Eigen::Matrix3d R_camera2standard = R_world2standard * camera_to_world;
    
//     cv::Mat matrix,result;
//     cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << intrinsic_data.fx, 0, intrinsic_data.cx,
//          0, intrinsic_data.fy, intrinsic_data.cy,
//          0, 0, 1);
//     cv::Mat dist_coeffs = (cv::Mat_<double>(1, 4) <<  intrinsic_data.k1, intrinsic_data.k2, intrinsic_data.k3, intrinsic_data.k4);
    
//     // 将 Eigen 矩阵转换为 OpenCV 的 cv::Mat
//     cv::Mat cv_mat(R_camera2standard.rows(), R_camera2standard.cols(), CV_64F, R_camera2standard.data());

//     // 由于 Eigen 矩阵是列主序，而 OpenCV Mat 是行主序，如果你需要在内存布局上保持一致，你可以选择复制
//     cv::Mat cv_mat_copy;
//     cv_mat.copyTo(cv_mat_copy);  // 如果你需要一个独立的副本


//     cv::fisheye::initUndistortRectifyMap(
//         camera_matrix,
//         dist_coeffs,
//         cv_mat_copy,
//         camera_matrix,
//         cv::Size(1920, 1280),
//         CV_16SC2,
//         map1_float,
//         map2_float);




// }


void single_view:: run_front_back(const std::string camera_name){

    read_intrinsic(avm_param_path); //get K, D , intrinsic_data
    construct_matrices(); //get K, D
    read_file(avm_transformer_path); //get T_camera_to_world
    get_pixel_3d_coord(); //get XYZ
    Eigen::Matrix3d R_combine;

    Eigen::Matrix3d R_world2standard;
    if(camera_name == "front_camera"){
        getFrontRealToVirualMatrix(); 
        R_combine = R_front_combine;
        R_world2standard << 0, -1, 0,
                    0, 0, -1,
                    1, 0, 0;
        getFishEye3DCoord(R_front_combine); //get XYZ_rotated
    }
    else if(camera_name == "back_camera"){
        getBackRealToVirualMatrix(); 
        R_combine = R_back_combine;
        R_world2standard << 0, 1, 0,
                            0, 0, -1,
                            -1, 0, 0;
        getFishEye3DCoord(R_back_combine); //get XYZ_rotated
    }
    else{
        std::cout << "Error: camera_name should be front_camera or rear_camera" << std::endl;
    }

   cv::Mat distorted_mat;
    cv::Vec4d cvVec_D = cv::Vec4d(D[0], D[1], D[2], D[3]);
    distortFishEye(XYZ_rotated, cvVec_D, distorted_mat); //get distorted_mat
    cv::Mat single_x_dst(XYZ_rotated.rows, XYZ_rotated.cols, CV_64FC1); 
    for (int y = 0; y < XYZ_rotated.rows; ++y) {
        for (int x = 0; x < XYZ_rotated.cols; ++x) {
            single_x_dst.at<double>(y, x) = XYZ_rotated.at<cv::Vec3d>(y, x)[0];
        }
    }
    cv::Mat map1 = single_x_dst.mul(distorted_mat)* K(0,0)+ K(0,2);
    
    map1.convertTo(map1_float, CV_32FC1); 


    cv::Mat single_y_dst(XYZ_rotated.rows, XYZ_rotated.cols, CV_64FC1);
    for (int y = 0; y < XYZ_rotated.rows; ++y) {
        for (int x = 0; x < XYZ_rotated.cols; ++x) {
            single_y_dst.at<double>(y, x) = XYZ_rotated.at<cv::Vec3d>(y, x)[1];
        }
    }
    cv::Mat map2 = single_y_dst.mul(distorted_mat)* K(1,1)+ K(1,2);
    map2.convertTo(map2_float, CV_32FC1); 
}



