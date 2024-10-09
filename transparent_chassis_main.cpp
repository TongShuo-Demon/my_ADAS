#include <opencv2/opencv.hpp>
#include <iostream>
#include "transparent_chassis_cpu_api.h"
#include <iomanip>  
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <ctime> 
#include "avm2d_config.pb.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include <google/protobuf/util/json_util.h>



using namespace ParkingPerception::TransparentChassisCpu;
using namespace google::protobuf;
using namespace google::protobuf::util;


// 用于存储CSV列数据的结构体
struct CSVData {
    std::string timestamp;
    std::string poseX;
    std::string poseY;
    std::string headrate;
    std::string VelocityYaw;
    std::string CarSpeed;
    std::string SteeringAngle;
};
// 计算 IQR 并过滤异常值
void remove_outliers(std::vector<cv::Point>& points) {
    if (points.size() < 4) return; // 至少需要四个点

    // 提取 y 值
    std::vector<float> y_values;
    for (const auto& point : points) {
        y_values.push_back(point.y);
    }

    // 排序 y 值
    sort(y_values.begin(), y_values.end());

    // 计算 Q1 和 Q3
    float q1 = y_values[y_values.size() / 4];
    float q3 = y_values[(3 * y_values.size()) / 4];
    float iqr = q3 - q1;

    // 定义异常值的上下限
    float lower_bound = q1 - 1.5 * iqr;
    float upper_bound = q3 + 1.5 * iqr;

    // 去除异常值
    points.erase(remove_if(points.begin(), points.end(),
                            [lower_bound, upper_bound](cv::Point p) {
                                return p.y < lower_bound || p.y > upper_bound;
                            }),
                  points.end());
}

// 反向映射函数
void reverse_mapping(const cv::Mat& map_x, const cv::Mat& map_y, cv::Mat& inv_map_x, cv::Mat& inv_map_y) {
    int h = map_x.rows;
    int w = map_x.cols;

    // 初始化为 -1，表示未映射
    inv_map_x = cv::Mat::ones(h, w, CV_32F) * -1;
    inv_map_y = cv::Mat::ones(h, w, CV_32F) * -1;

    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            // 从 map_x 和 map_y 中获取映射的坐标值
            int x = static_cast<int>(std::round(map_x.at<float>(i, j)));
            int y = static_cast<int>(std::round(map_y.at<float>(i, j)));

            // 检查映射点是否在图像范围内
            if (x >= 0 && x < w && y >= 0 && y < h) {
                // 确保不覆盖已有的映射
                if (inv_map_x.at<float>(y, x) == -1 && inv_map_y.at<float>(y, x) == -1) {
                    inv_map_x.at<float>(y, x) = static_cast<float>(j);
                    inv_map_y.at<float>(y, x) = static_cast<float>(i);
                }
            }
        }
    }
}

// 读取CSV文件并返回指定列的数据
std::vector<CSVData> readCSV(const std::string& filename) {
    std::vector<CSVData> result;

    // 打开CSV文件
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << filename << std::endl;
        return result;
    }

    // 跳过第一行（标题行）
    std::string line;
    if (std::getline(file, line)) {
        // 如果需要可以在这里处理标题行的内容，比如打印标题或者根据标题确定列数
    }

    // 继续逐行读取CSV并提取指定列的数据
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        
        CSVData data;

        std::getline(ss, token, ',');  // 跳过第一列
        std::getline(ss, data.timestamp, ',');  // 假设'Timestamp'在第二列
        std::getline(ss, data.poseX, ',');  // 假设'Pose-X'在第三列
        std::getline(ss, data.poseY, ',');  // 假设'Pose-Y'在第四列
        std::getline(ss, data.headrate, ',');  // 假设'Headrate'在第五列
        std::getline(ss, data.VelocityYaw, ',');  // 假设'Velocity-Yaw'在第六列
        std::getline(ss, token, ',');  // 跳过第七列
        std::getline(ss, token, ',');  // 跳过第八列
        std::getline(ss, data.CarSpeed, ',');  // 假设'CarSpeed'在第九列
        std::getline(ss, data.SteeringAngle, ',');  // 假设'SteeringAngle'在第十列

        result.push_back(data);
    }

    // 关闭文件
    file.close();

    return result;
}

// 找到与给定时间戳最接近的CSV行索引
int findClosestTimeStamp(const std::vector<CSVData>& csvData, const std::string& timestamp) {
    int closestIndex = 0;

    double closestDiff = std::numeric_limits<double>::max();

    for (int i = 0; i < csvData.size(); ++i) {
        double diff = std::abs(std::stod(csvData[i].timestamp) - std::stod(timestamp));

        if (diff < closestDiff) {
            closestDiff = diff;
            closestIndex = i;
        }
    }

    return closestIndex;
}

// 在图像上添加车模，使用OpenCV泊松融合
cv::Mat seamlessCloneWrapper(cv::Mat input_image, cv::Mat rect_region, int x, int y, int aw, int ah) {
    int height = input_image.rows;  // 获取图像的高度
    int width = input_image.cols;  // 获取图像的宽度
    int channels = input_image.channels();  // 获取图像的通道数
    int w = width - 2 * x, h = height - 2 * y;
    cv::resize(rect_region, rect_region, cv::Size(w + aw, h + ah));
    cv::Mat result;
    cv::Point center(x + w / 2, y + h / 2);
    cv::seamlessClone(rect_region, input_image, cv::Mat(), center, result, cv::MIXED_CLONE);
    return result;
}

// 在图像上添加 alpha 通道
cv::Mat addAlphaChannel(cv::Mat img) {
    cv::Mat bgr[3];
    cv::split(img, bgr);
    cv::Mat alpha = cv::Mat::ones(bgr[0].size(), bgr[0].type()) * 255;
    
    cv::Mat img_new;
    cv::merge(std::vector<cv::Mat>{bgr[0], bgr[1], bgr[2], alpha}, img_new);

    return img_new;
}

// 在图像上添加车模，使用透明PNG图像
cv::Mat merge_img_withPNG(cv::Mat jpgImg, cv::Mat pngImg, int x1, int y1, int x2, int y2) {
      // 判断jpg图像是否已经为4通道
    if (jpgImg.channels() == 3) {
        // 添加 alpha 通道
        jpgImg = addAlphaChannel(jpgImg);
    }

    int yy1 = 0, yy2 = pngImg.rows, xx1 = 0, xx2 = pngImg.cols;

    if (x1 < 0) {
        xx1 = -x1;
        x1 = 0;
    }
    if (y1 < 0) {
        yy1 = -y1;
        y1 = 0;
    }
    if (x2 > jpgImg.cols) {
        xx2 = pngImg.cols - (x2 - jpgImg.cols);
        x2 = jpgImg.cols;
    }
    if (y2 > jpgImg.rows) {
        yy2 = pngImg.rows - (y2 - jpgImg.rows);
        y2 = jpgImg.rows;
    }

    cv::Mat alphaPng = pngImg(cv::Rect(xx1, yy1, xx2 - xx1, yy2 - yy1)).col(3) / 255.0;
    cv::Mat alphaJpg = cv::Scalar(1.0, 1.0, 1.0) - alphaPng;

    // for (int c = 0; c < 3; ++c) {
    //     jpgImg(cv::Rect(x1, y1, x2 - x1, y2 - y1)).at<cv::Vec3b>(0, 0)[c] = 
    //         (alpha_jpg.mul(jpgImg(cv::Rect(x1, y1, x2 - x1, y2 - y1)).at<cv::Vec3b>(0, 0)[c]) + 
    //         alpha_png.mul(pngImg(cv::Rect(xx1, yy1, xx2 - xx1, yy2 - yy1)).at<cv::Vec3b>(0, 0)[c]));
    // }

    // 计算要叠加的区域的alpha值
    float alpha_png = pngImg.at<cv::Vec4b>(yy1, xx1)[3] / 255.0f;

    // 开始叠加
    cv::Mat result = jpgImg.clone(); // 为了不修改原始图像，我们进行克隆操作

    for (int c = 0; c < 3; c++) {
        result(cv::Rect(x1, y1, x2 - x1, y2 - y1)).at<cv::Vec3b>(yy1, xx1)[c] = 
            alpha_png * pngImg.at<cv::Vec4b>(yy1, xx1)[c] + 
            (1 - alpha_png) * jpgImg.at<cv::Vec3b>(yy1, xx1)[c];
    }
    // cv::namedWindow("Result", cv::WINDOW_NORMAL);
    // cv::imshow("Result", result);
    // cv::waitKey(0);

    return jpgImg;
}

// 显示进度条的函数
void updateProgress(int progress) {
    std::string progressBar = "[";

    int totalPositions = 20;
    int completedPositions = (progress * totalPositions) / 100;

    for (int i = 0; i < completedPositions; ++i) {
        progressBar += "=";
    }

    for (int i = completedPositions; i < totalPositions; ++i) {
        progressBar += " ";
    }

    progressBar += "] " + std::to_string(progress) + "%";

    std::cout << "\r" << progressBar << std::flush;
}

// 将消息转换为 JSON 字符串
std::string MessageToJson(const Message &message) {
    std::string json_string;
    JsonPrintOptions options;
    Status status = MessageToJsonString(message, &json_string, options);
    if (!status.ok()) {
        std::cerr << "Failed to convert message to JSON: " << status.ToString() << std::endl;
        return "";
    }
    return json_string;
}

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

// 序列化消息到 JSON 文件
void SerializeToJsonFile(const string &filename, const adas::Config  &config) {
    // Convert messages to JSON string
    string config_json = MessageToJson(config);

    // Write JSON string to file
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }
    file << config_json;
    file.close();
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


void drawLineAndExtractPoints( cv::Point startPoint, cv::Point endPoint, std::vector<cv::Point>& linePoints) {
    // Draw the line
    // cv::line(image, startPoint, endPoint, color, thickness);
    
    // Extract points from the line
    int dx = endPoint.x - startPoint.x;
    int dy = endPoint.y - startPoint.y;

    // Calculate the number of steps needed to create points
    int steps = std::max(abs(dx), abs(dy));
    
    for (int i = 0; i <= steps; i++) {
        // Interpolate points on the line
        int x = startPoint.x + i * dx / steps;
        int y = startPoint.y + i * dy / steps;
        linePoints.push_back(cv::Point(x, y)); // Store the point
    }
}





int main()

{
  
  adas::Config  transparent_chassis_config;
  
  ParseFromJsonFile("/home/adairtong/workspace/ADAS/bin/message.json",transparent_chassis_config);  

std::cout << transparent_chassis_config.avm_2d_config().avm2d_remap_path() << std::endl;
  std::cout << "start ----f-------------------------------------------\n";
  // 读取CSV文件并获取所有行数据
  std::vector<CSVData> DR_csvData = readCSV(transparent_chassis_config.transparent_chassis_config().dr_path());
   TransparentChassisData input_config;
   input_config.bev_ratio = transparent_chassis_config.transparent_chassis_config().bev_ratio();
   input_config.bev_center_x = transparent_chassis_config.transparent_chassis_config().bev_center_x();
   input_config.bev_center_y = transparent_chassis_config.transparent_chassis_config().bev_center_y();
   input_config.bev_w = transparent_chassis_config.transparent_chassis_config().bev_w();
   input_config.bev_h = transparent_chassis_config.transparent_chassis_config().bev_h();
   input_config.shift_lr = transparent_chassis_config.transparent_chassis_config().shift_lr();
   input_config.shift_tb = transparent_chassis_config.transparent_chassis_config().shift_tb();
   input_config.filter_kernel_size = transparent_chassis_config.transparent_chassis_config().filter_kernel_size();


  //构造实例
  std::shared_ptr<TransparentChassis> transparent_chassis = CreateTransparentChassis(input_config);

  //初始化
  if (0 != transparent_chassis->init())
  {
    return 0;
  }
  const double L = 2.68;  // 车辆轴距
  const double w =1.585;  // 车辆后轮距
  const float bev_ratio = transparent_chassis->bev_ratio; //拼接图一个像素代表的实际物理距离（米）
  const int bev_width = transparent_chassis->w;  // BEV图像宽度
  const int bev_height = transparent_chassis->h;  // BEV图像高度
  const int shift_lr = transparent_chassis->shift_lr;  // 黑边离图像左右两侧的距离，当前拼接图左右对称(像素)
  const int shift_tb = transparent_chassis->shift_tb;  // 黑边离图像上下两侧的距离，当前拼接图上下对称(像素)
  const int filter_kernel_size = transparent_chassis->filter_kernel_size;  // 平滑卷积核大小
  const int L_pixel = L/bev_ratio;  // 车辆轴距像素值
  const int w_pixel = w/bev_ratio;  // 车辆后轮距像素值

  std::string image_dir = transparent_chassis_config.transparent_chassis_config().input_dir_path(); 
  std::cout << input_config.filter_kernel_size << image_dir << std::endl;

  std::vector<std::filesystem::path> fileList;
    for (const auto & entry: std::filesystem::directory_iterator(image_dir)){
        // std::cout << entry.path() << std::endl;
        fileList.push_back(entry.path());
    }
  sort(fileList.begin(), fileList.end());


  std::string timeStamp;
  int closestIndex;

for(size_t i = 0; i<fileList.size(); ++i){

    cv::Mat img2 = cv::imread(fileList[i].string());
    int height = img2.rows;  // 获取图像的高度
    int width = img2.cols;  // 获取图像的宽度

    timeStamp = fileList[i].filename().string().substr(0,10);  // 获取avm图时间戳
    closestIndex = findClosestTimeStamp(DR_csvData, timeStamp);
    printf("dr_timeStamp: %s, closestIndex: %d\n", DR_csvData[closestIndex].timestamp.c_str(), closestIndex);

    double x = std::stod(DR_csvData[closestIndex].poseX);  // 读取CSV文件中的poseX列数据
    double y = std::stod(DR_csvData[closestIndex].poseY);   // 读取CSV文件中的poseY列数据
    double z = 0.0; // 读取CSV文件中的poseZ列数据
    double roll = 0.0;  
    double pitch = 0.0;
    double yaw = std::stod(DR_csvData[closestIndex].headrate);  // 读取CSV文件中的headrate列数据
    double CarSpeed	= std::stod(DR_csvData[closestIndex].CarSpeed);  // 读取CSV文件中的headrate列数据
    double SteeringAngle = std::stod(DR_csvData[closestIndex].SteeringAngle);  // 读取CSV文件中的FrontWheelAngle列数据

    LocData loc2 = LocData(std::stod(timeStamp), x, y, z, roll, pitch, yaw);
    transparent_chassis->process(img2, loc2);

    // 获取输出图像
    cv::Mat output2;
    transparent_chassis->get_result(output2);
    cv::Mat car_model = cv::imread("/home/adairtong/workspace/ADAS/resources/transparent_chassis/001_bordered_contour.jpg");
    cv::Mat wheel_model = cv::imread("/home/adairtong/workspace/ADAS/resources/transparent_chassis/hcef3/hc_t_wheel.png", cv::IMREAD_UNCHANGED);
    
    cv::Mat final_output;
    int x_offset = 470, y_offset = 470;  // 原始矩形区域的左上角坐标
    int aw = 120, ah = 16; // 基于原始矩形区域，向四周扩充的宽度
    int w_model =width-2*x_offset+2*aw,  h_model = height-2*y_offset+2*ah;  // 调整后车模区域的宽高
    std::cout << "w_model: " << w_model << ", h_model: " << h_model << std::endl;

    cv::resize(car_model, car_model, cv::Size(w_model, h_model));
    cv::resize(wheel_model, wheel_model, cv::Size(w_model, h_model));

    // 使用OpenCV的泊松融合算法拼接图像
    final_output =  seamlessCloneWrapper(output2, car_model, x_offset, y_offset, aw, ah);

    // // 使用透明通道的png图像作为车辆模型
    // final_output = merge_img_withPNG(output2, car_model, x_offset-aw, y_offset-ah, x_offset-aw+w_model, y_offset-ah+h_model);
    // final_output = merge_img_withPNG(output2, wheel_model, x_offset-aw, y_offset-ah, x_offset-aw+w_model, y_offset-ah+h_model);

    // 计算车辆转向半径
    // float R_turning = L_pixel*cos(SteeringAngle)/sin(SteeringAngle);  
    float R_turning = L_pixel*cos(SteeringAngle)/sin(SteeringAngle);  
    // 绘制轨迹线，确定弧线的起始点、结束点、半径、颜色和线条粗细
    // cv::Point center(570-R_turning, 910);
    
    cv::Point center((bev_width / 2) - R_turning, (bev_height / 2) + (L_pixel / 2));
    cv::Point LineL1((bev_width / 2) - w_pixel / 2 , (bev_height / 2) + (L_pixel / 2));
    cv::Point LineL2((bev_width / 2) - w_pixel / 2 , 10);
    cv::Point LineR1((bev_width / 2) + w_pixel / 2 , (bev_height / 2) + (L_pixel / 2));
    cv::Point LineR2((bev_width / 2) + w_pixel / 2 , 10);

    int radius_i=0, radius_o=0;
    std::cout << "SteeringAngle " << SteeringAngle << std::endl;
    std::cout << "R_turning " << R_turning << std::endl;
    // if (R_turning > 0) {
    //     radius_i = (R_turning - w_pixel/2)/cos(SteeringAngle);
    //     radius_o = (R_turning + w_pixel/2)/cos(SteeringAngle); 
    // }
    // else {
    //     radius_i = (-R_turning + w_pixel/2)/cos(SteeringAngle);
    //     radius_o = (-R_turning - w_pixel/2)/cos(SteeringAngle);
    // }

    if(SteeringAngle > 0.4 ){
        radius_i = (R_turning - w_pixel/2)/cos(SteeringAngle)-5;
        radius_o = (R_turning + w_pixel/2)/cos(SteeringAngle)+45; 

    }else if (SteeringAngle >=0 && SteeringAngle <= 0.4)
    {
        radius_i = (R_turning - w_pixel/2)/cos(SteeringAngle);
        radius_o = (R_turning + w_pixel/2)/cos(SteeringAngle); 
    }else if (SteeringAngle < -0.4)
    {
        radius_i = (-R_turning + w_pixel/2)/cos(SteeringAngle)-5;
        radius_o = (-R_turning - w_pixel/2)/cos(SteeringAngle)+45; 
    }else{
        radius_i = (-R_turning + w_pixel/2)/cos(SteeringAngle);
        radius_o = (-R_turning - w_pixel/2)/cos(SteeringAngle); 
    }
    
    
    

    printf("center: (%d, %d), radius_i: %d, radius_o: %d\n", center.x, center.y, radius_i, radius_o);
    // 绘制车辆转向半径
    cv::Scalar color_i(255, 0, 0); // 内侧（左）使用的是蓝色
    cv::Scalar color_o(0, 255, 0); // 外侧（右）使用的是绿色
    cv::Scalar white(255); // 白色，用于掩码
    int thickness = 20;


    std::ostringstream oss;
    oss << transparent_chassis_config.transparent_chassis_config().output_dir_path() << "/" << timeStamp << ".jpg";  // 左对齐用0补全6位
    std::string save_Path = oss.str();
    
    std::cout << "Save output img: " << save_Path << std::endl;
    // 创建一个掩码图像
    cv::Mat mask = cv::Mat::zeros(final_output.size(), CV_8UC3); // 创建黑色掩码图像
    mask = cv::Mat::zeros(final_output.size(), CV_8UC3); // 创建黑色掩码图像

        // 生成椭圆的点集
    std::vector<cv::Point> ellipsePoints_i;
    std::vector<cv::Point> ellipsePoints_o;
    std::vector<cv::Point> ellipsePoints;
    ellipsePoints.clear();
    ellipsePoints_o.clear();
    ellipsePoints_i.clear();
    // if (SteeringAngle > 0.01)
    // {
        
    //     cv::ellipse(final_output, center, cv::Size(radius_i, radius_i), 0, 0, -80, color_i, thickness);
    //     cv::ellipse(final_output, center, cv::Size(radius_o, radius_o), 0, 0, -60, color_o, thickness);
    //         // Extract points from the ellipse
    //     for (int angle = -80; angle <= 0; angle++) {
    //         double radian = angle * CV_PI / 180.0; // Convert angle to radians
    //         int x = center.x + radius_i * cos(radian); // X coordinate
    //         int y = center.y + radius_i * sin(radian); // Y coordinate
    //         ellipsePoints_i.push_back(cv::Point(x, y)); // Store the point
    //     }
    //     for (int angle = -60; angle <= 0; angle++) {
    //         double radian = angle * CV_PI / 180.0; // Convert angle to radians
    //         int x = center.x + radius_o * cos(radian); // X coordinate
    //         int y = center.y + radius_o * sin(radian); // Y coordinate
    //         ellipsePoints_o.push_back(cv::Point(x, y)); // Store the point
    //     }
    // }else if(SteeringAngle > -0.01){
    //     cv::line(final_output, LineL1, LineL2, color_i, thickness);
    //     drawLineAndExtractPoints( LineL1, LineL2, ellipsePoints_i);
    //     cv::line(final_output, LineR1, LineR2, color_i, thickness);
    //     drawLineAndExtractPoints( LineR1, LineR2, ellipsePoints_o);
    // }else {
        
    //     cv::ellipse(final_output, center, cv::Size(radius_i, radius_i), 0, 180, 260, color_i, thickness);
    //     cv::ellipse(final_output, center, cv::Size(radius_o, radius_o), 0, 180, 240, color_o, thickness);
    //     for (int angle = 180; angle <= 260; angle++) {
    //         double radian = angle * CV_PI / 180.0; // Convert angle to radians
    //         int x = center.x + radius_i * cos(radian); // X coordinate
    //         int y = center.y + radius_i * sin(radian); // Y coordinate
    //         ellipsePoints_i.push_back(cv::Point(x, y)); // Store the point
    //     }
    //     for (int angle = 180; angle <= 240; angle++) {
    //         double radian = angle * CV_PI / 180.0; // Convert angle to radians
    //         int x = center.x + radius_o * cos(radian); // X coordinate
    //         int y = center.y + radius_o * sin(radian); // Y coordinate
    //         ellipsePoints_o.push_back(cv::Point(x, y)); // Store the point
    //     }

    // }


    if (radius_i > 0 && radius_o > 0) {
        cv::ellipse(mask, center, cv::Size(radius_i, radius_i), 0, 0, 360, color_i, thickness);
        cv::ellipse(mask, center, cv::Size(radius_o, radius_o), 0, 0, 360, color_o, thickness);
        cv::Rect lowerHalf(0, 2000 / 2, 1140, 1500-2000/2); // 定义下半部分区域
        mask(lowerHalf) = cv::Scalar(0); // 将下半部分设置为黑色

        cv::Mat mask_gray;
        cv::cvtColor(mask, mask_gray, cv::COLOR_BGR2GRAY); // 转换为灰度图像
        cv::imwrite("/home/adairtong/workspace/ADAS/bin/mask.jpg", mask_gray); // 保存掩码图像
        cv::findNonZero(mask_gray, ellipsePoints); // 提取掩码中非零点的坐标
        // 将掩码应用于原图像
        mask.copyTo(final_output, mask); // 只在掩码区域保留图像内容
       
    
        // const int numPoints = 300; // 点的数量
        // double startAngleDegree = -180; // 起始角度（度数）
        // double endAngleDegree = -10;   // 终止角度（度数）

        // // 将角度转换为弧度
        // double startAngleRad = startAngleDegree * M_PI / 180.0;
        // double endAngleRad = endAngleDegree * M_PI / 180.0;
        // // 确保起始角度小于终止角度
        // if (endAngleRad < startAngleRad) {
        //     endAngleRad += 2 * M_PI;
        // }
        // for (int i = 0; i < numPoints; ++i) {
        //     double theta = startAngleRad + (endAngleRad - startAngleRad) * i / numPoints;
        //     double x = center.x + radius_i * cos(theta);
        //     double y = center.y + radius_i * sin(theta);
        //     if (x < 0 || x > 1140 || y < 0 || y > 1500) 
        //         continue;
        //     double x_o = center.x + radius_o * cos(theta);
        //     double y_o = center.y + radius_o * sin(theta);
        //     if(x_o <0 || x_o > 1140 || y_o <0 || y_o > 1500)
        //         continue;
        //     ellipsePoints_i.emplace_back(cv::Point(static_cast<int>(x), static_cast<int>(y)));
        //     ellipsePoints_o.emplace_back(cv::Point(static_cast<int>(x_o), static_cast<int>(y_o)));
        // }

        // for (int i=0; i< ellipsePoints_o.size(); i++) {
        //     cv::circle(final_output, ellipsePoints_o[i], 1, cv::Scalar(0, 255, 0), 5); // 绿色点
        //     cv::circle(final_output, ellipsePoints_i[i], 1, cv::Scalar(0, 0, 255), 5); // 绿色点
        // }
    }
    


    cv::Mat loaded_mat_front_x;
    cv::Mat loaded_mat_front_y;
    // cv::FileStorage fs2(transparent_chassis_config.avm_2d_config().avm2d_remap_path(), cv::FileStorage::READ);
    cv::FileStorage fs2("/home/adairtong/workspace/ADAS/bin/ffff.yml", cv::FileStorage::READ);
    fs2["front_matrix_map_x"] >> loaded_mat_front_x;
    fs2["front_matrix_map_y"] >> loaded_mat_front_y;
    std::cout << "loaded_mat_front_x: " << loaded_mat_front_x.size() <<" loaded_mat_front_x.channels() "<< loaded_mat_front_x.channels() << std::endl;
    std::cout << "loaded_mat_front_y: " << loaded_mat_front_y.size()<<" loaded_mat_front_y.channels() "<< loaded_mat_front_y.channels() << std::endl;
    fs2.release();
  
    cv::Scalar point_color(0, 255, 0); // BGR颜色
    timeStamp;
    string ori_img= "/home/adairtong/workspace/ADAS/resources/ori_data/ori_image/"+timeStamp + "/front.jpg";
    cv::Mat img = cv::imread(ori_img);
    std::cout << "ori_img: " << ori_img << std::endl;
    std::vector<cv::Point> ellipsePoints_i_front;
    std::vector<cv::Point> ellipsePoints_o_front;
    ellipsePoints_o_front.clear();
    ellipsePoints_i_front.clear();

    // for (const auto& point : ellipsePoints_i) {
    //     if (point.x < 0 || point.x > 1140 || point.y < 0 || point.y > 480)
    //         continue;
    //     cv::Vec2s vec = loaded_mat_front_x.at<cv::Vec2s>(point.y, point.x);
    //     int x = vec[0];  // 获取第一个分量 (x)
    //     int y = vec[1];  // 获取第二个分量 (y)
    //     cv::circle(img, cv::Point(static_cast<int>(x)+150, static_cast<int>(y)), 1, color_i, 8);
    //     ellipsePoints_i_front.emplace_back(cv::Point(static_cast<int>(x)+150, static_cast<int>(y)));
    // }

    // for (const auto& point : ellipsePoints_o) {
    //     if (point.x < 0 || point.x > 1140 || point.y < 0 || point.y > 480)
    //         continue;
    //     cv::Vec2s vec = loaded_mat_front_x.at<cv::Vec2s>(point.y, point.x);
    //     int x = vec[0];  // 获取第一个分量 (x)
    //     int y = vec[1];  // 获取第二个分量 (y)
    //     cv::circle(img, cv::Point(static_cast<int>(x)+150, static_cast<int>(y)), 1, color_o, 8);
    //     ellipsePoints_o_front.emplace_back(cv::Point(static_cast<int>(x)+150, static_cast<int>(y)));
    // }

    for (const auto& point : ellipsePoints) {
        if (point.x < 0 || point.x > 1140 || point.y < 0 || point.y > 480)
            continue;
        cv::Vec2s vec = loaded_mat_front_x.at<cv::Vec2s>(point.y, point.x);
        float x = loaded_mat_front_x.at<float>(point.y, point.x); //vec[0];  // 获取第一个分量 (x)
        float y = loaded_mat_front_y.at<float>(point.y, point.x); //vec[1];  // 获取第二个分量 (y)
        // std::cout << "x: " << x << " y: " << y << std::endl;
        cv::circle(img, cv::Point(static_cast<int>(floor(x))+150, static_cast<int>(floor(y))+150), 1, color_o, 8);
        ellipsePoints_o_front.emplace_back(cv::Point(static_cast<int>(x)+150, static_cast<int>(y)));
    }
  


    cv::imwrite(save_Path, final_output);
    cv::imwrite("/home/adairtong/workspace/ADAS/bin/tt/"+timeStamp+ ".jpg", img);

  }

  return 0;
}