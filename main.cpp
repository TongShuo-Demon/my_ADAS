#include <iostream>
#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include  "2d_avm.h"
using namespace std;
namespace fs = std::filesystem;



int main() {

  Avm2d temp;
  temp.InitByConfigFile("/home/adairtong/workspace/ADAS/bin/message.json");

  std::vector<cv::Mat> image_mats;
  string input_dir = "/home/adairtong/workspace/ADAS/resources/ori_data/ori_image";
  std::vector<fs::path> files;
  // 遍历输入目录并将文件路径添加到vector中
  for (auto it = fs::directory_iterator(input_dir); it != fs::directory_iterator(); ++it) {
    const auto& entry = *it;
    if (entry.is_directory()) {
        files.emplace_back(entry.path());
    }
  }

  // 对文件路径进行排序
  std::sort(files.begin(), files.end());
  for (const auto& file_name : files) {
      image_mats.clear();
      for (const auto& name : camera_names) {
          std::string image_path = file_name.string() + "/"+name + ".jpg";
          std::cout << "Processing " << image_path << std::endl;
          cv::Mat image_ori = cv::imread(image_path);
          cv::Rect rect(0, 0, image_ori.cols, 3); // 矩形区域(x, y, width, height)
          image_ori(rect).setTo(cv::Scalar(0, 0, 0));
          image_mats.emplace_back(image_ori);
              
      }
      cv::Mat result;
      temp.Avm2dRun(image_mats,result,true);
      string output_name = file_name.has_parent_path() ? file_name.filename().string() : file_name.string();
      cv::imwrite("/home/adairtong/workspace/ADAS/bin/avm_out/"+output_name+".jpg",result);
  }



  return 0;
}


