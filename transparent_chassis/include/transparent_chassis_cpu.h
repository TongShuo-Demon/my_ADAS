#pragma once

#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace ParkingPerception
{
  namespace TransparentChassisCpu
  {
    struct float3
    {
        float x, y, z;
    };

    struct TransparentChassisData {
        int bev_w =140;
        int bev_h = 1500;
        float bev_ratio = 0.01; //#拼接图一个像素代表的实际物理距离（米）
        float bev_center_x=570; //#定位原点在拼接图上的像素x坐标（像素）
        float bev_center_y=880; //#定位原点在拼接图上的像素y坐标（像素）
        int shift_lr=470;  //#黑边离图像上下两侧的距离，当前拼接图上下对称(像素)
        int shift_tb=470; //#黑边离图像上下两侧的距离，当前拼接图上下对称(像素)
        int filter_kernel_size=75; //#平滑卷积核大小
    };

    struct LocData
    {
      LocData() : time(0.0), x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0){};
      LocData(double t, double xx, double yy, double zz, double r, double p, double y)
          : time(t), x(xx), y(yy), z(zz), roll(r), pitch(p), yaw(y){};
      double time;
      double x;
      double y;
      double z;
      double roll;
      double pitch;
      double yaw;
    };


    class TransparentChassis
    {
    public:
      TransparentChassis(TransparentChassisData config_param);
      ~TransparentChassis();
      int init();
      int process(const cv::Mat &img_now, const LocData &loc_now);
      void get_result(cv::Mat &out);
      //拼接图参数
      int w = 0;
      int h = 0;
      float bev_ratio;
      float bev_center_x;
      float bev_center_y;
      int shift_lr = 0;
      int shift_tb = 0;
      //平滑卷积
      int filter_kernel_size = 0;
      cv::Mat weight; // 融合时权重

    private:
      int load_config();
      void get_warpaffine(const LocData &loc_now);
      void destroy();
      float bicubic(float x);
      void getImpactFactors(float rowU, float colV, float *rowImFac, float *colImFac, int starti, int startj);
      cv::Vec3b bicubic_interpolation(float src_x, float src_y, int width, int height, uint8_t *src, float *rowImFac, float *colImFac);
      cv::Vec3b bilinear_interpolation(float src_x, float src_y, int width, int height, int line_size, uint8_t fill_value, uint8_t *src);
      cv::Vec3b optimized_bilinear_interpolation(float src_x, float src_y, int width, int height, int line_size, uint8_t fill_value, uint8_t *src);
      cv::Vec3b nearest_neighbor_interpolation(float src_x, float src_y, int width, int height, uint8_t *src);
      void process_img_kernel(const cv::Mat &img_pre, cv::Mat &img_fusion, const cv::Mat &img_now, const cv::Mat &weight, int w, int h, const float *affine_now2pre);
      void process_block(const cv::Mat &img_pre, cv::Mat &img_fusion, const cv::Mat &img_now, const cv::Mat &weight, int w, int h, const float *affine_now2pre, int start_row, int end_row);
    
    private:
      //配置文件
      TransparentChassisData config_param_;
      cv::Mat output;                      //输出图像
      cv::Mat img_pre;    // 上一帧图像
      cv::Mat img_fusion; // 上一帧变换到当前时刻的图像
      //仿射变换
      float affine_now2pre[6]; // img_pre -> img_now仿射变换矩阵
      //定位
      LocData loc_data_pre; //上一帧位置
      //是否为首帧
      bool is_first;
    };
  } // namespace TransparentChassisCpu
} // namespace ParkingPerception
