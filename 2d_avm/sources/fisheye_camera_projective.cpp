#include <fisheye_camera_projective.h>

// 初始化静态成员变量
std::map<std::string, cv::Size> settings::project_shapes = {
    {"front", cv::Size(540, 283)},
    {"back", cv::Size(540, 283)},
    {"left", cv::Size(820, 197)},
    {"right", cv::Size(820, 197)}

};

FisheyeCameraProjective::FisheyeCameraProjective(const std::string &camera_param_file, const std::string &camera_name) {
    if (!std::filesystem::exists(camera_param_file)) {
        throw std::invalid_argument("找不到相机参数文件");
    }

    if (std::find(camera_names.begin(),camera_names.end(), camera_name) == camera_names.end()) {
        throw std::invalid_argument("未知的相机名称: " + camera_name);
    }

    camera_file = camera_param_file;
    this->camera_name = camera_name;
    scale_xy = cv::Vec2f(1.0, 1.0);
    shift_xy = cv::Vec2f(0, 0);
    undistort_maps[0] = undistort_maps[1] = cv::Mat();
    project_shape = settings::project_shapes[camera_name];
    LoadCameraParams();
}
FisheyeCameraProjective::FisheyeCameraProjective(CameraParam &camera_param){
     this->camera_name = camera_param.camera_name;
     scale_xy = camera_param.scale_xy;
     shift_xy = camera_param.shift_xy;
     camera_matrix = camera_param.intrinsic_matrix;
     dist_coeffs = camera_param.distortion_coefficients;
     resolution = camera_param.resolution;
     project_matrix = camera_param.projective_matrix;
     project_shape = camera_param.crop_size;
     UpdateUndistortMaps();

}



// 加载相机参数
void FisheyeCameraProjective::LoadCameraParams() {
    cv::FileStorage fs(camera_file, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs["resolution"] >> resolution;
    resolution = resolution.reshape(1, 2);

    cv::Mat scaleMat, shiftMat, projectMat;
    fs["scale_xy"] >> scaleMat;
    if (!scaleMat.empty()) {
        scale_xy = cv::Vec2f(scaleMat);
    }

    fs["shift_xy"] >> shiftMat;
    if (!shiftMat.empty()) {
        shift_xy = cv::Vec2f(shiftMat);
    }

    fs["project_matrix"] >> projectMat;
    if (!projectMat.empty()) {
        project_matrix = projectMat;
    }

    fs.release();
    UpdateUndistortMaps();
}

// 更新去畸变地图
void FisheyeCameraProjective::UpdateUndistortMaps() {
    cv::Mat new_matrix = camera_matrix.clone();
    new_matrix.at<double>(0, 0) *= scale_xy[0];
    new_matrix.at<double>(1, 1) *= scale_xy[1];
    new_matrix.at<double>(0, 2) += shift_xy[0];
    new_matrix.at<double>(1, 2) += shift_xy[1];
    int width = static_cast<int>(resolution.at<int>(0, 0));
    int height = static_cast<int>(resolution.at<int>(0, 1));

    cv::fisheye::initUndistortRectifyMap(
        camera_matrix,
        dist_coeffs,
        cv::Mat::eye(3, 3, CV_64F),
        new_matrix,
        cv::Size(width, height),
        CV_32FC1,
        undistort_maps[0],
        undistort_maps[1]);
}

// 对图像进行去畸变
cv::Mat FisheyeCameraProjective::Undistort(const cv::Mat &image) {
    cv::Mat result;
    cv::remap(image, result, undistort_maps[0], undistort_maps[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return result;
}

// 对图像进行投影
cv::Mat FisheyeCameraProjective::ProjectiveTransform(const cv::Mat &image) {
    cv::Mat result = cv::Mat::ones(project_shape.height, project_shape.width, CV_32FC3);
    cv::warpPerspective(image, result, project_matrix, project_shape);
    return result;
}

// 根据相机方向翻转图像
cv::Mat FisheyeCameraProjective::FlipImage(const cv::Mat &image) {
    cv::Mat result;
    if (camera_name == "front") {
        result = image;
    } else if (camera_name == "back") {
        cv::flip(image, result, -1); //<0：同时沿x轴和y轴翻转（对角线翻转）。
    } else if (camera_name == "left") {
        cv::transpose(image, result);
        cv::flip(result, result, 0); // 0：沿x轴翻转（上下翻转）。
        // 旋转图像（先转置再上下翻转，等效于顺时针旋转90度）
        // cv::rotate(image, result, cv::ROTATE_90_CLOCKWISE);
    } else {
        cv::transpose(image, result);
        cv::flip(result, result, 1); //>0：沿y轴翻转（左右翻转）。
    }
    return result;
}

cv::Mat FisheyeCameraProjective::SaveUndistortPerspectiveRemp(cv::Mat &image1,int ii, std::string remap_path) {
    cv::Mat Tinv;
    cv::invert(project_matrix, Tinv);
    Tinv.convertTo(Tinv, CV_32FC1);
    cv::Mat map_x(project_shape.height, project_shape.width, CV_32FC1);
    cv::Mat map_y(project_shape.height, project_shape.width, CV_32FC1);

    for (int i = 0; i < project_shape.height; ++i) {
        for (int j = 0; j < project_shape.width; ++j) {
            cv::Mat point = (cv::Mat_<float>(3, 1) << j, i, 1);
            cv::Mat proj = Tinv * point;
            float w = proj.at<float>(2, 0);
            map_x.at<float>(i, j) = (proj.at<float>(0, 0) / w);
            map_y.at<float>(i, j) = (proj.at<float>(1, 0) / w);
        }
    }
    cv::Mat new_map_x(project_shape.height, project_shape.width, CV_32FC1);
    cv::Mat new_map_y(project_shape.height, project_shape.width, CV_32FC1);

    cv::remap(undistort_maps[0], new_map_x, map_x, map_y, cv::INTER_LINEAR);
    cv::remap(undistort_maps[1], new_map_y, map_x, map_y, cv::INTER_LINEAR);
    cv::FileStorage fs(remap_path, cv::FileStorage::APPEND);
    fs <<  camera_names[ii]+"_matrix_map_x" << new_map_x;
    fs  << camera_names[ii]+"_matrix_map_y" << new_map_y;
    fs.release();

    // 进行透视变换
    cv::Mat result(image1.size(), image1.type(), cv::Scalar(0, 0, 0));
    cv::remap(image1, result, new_map_x, new_map_y, cv::INTER_LINEAR);

    return result;
}

cv::Mat FisheyeCameraProjective::RunRemap(cv::Mat &image1, cv::Mat &new_map_x, cv::Mat &new_map_y) {
    // 进行透视变换
    cv::Mat result(image1.size(), image1.type(), cv::Scalar(0, 0, 0));
    cv::remap(image1, result, new_map_x, new_map_y, cv::INTER_LINEAR);
    return result;
}
