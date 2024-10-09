#include "transparent_chassis_cpu.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <opencv2/core/types.hpp> 
#include <thread>

using namespace cv;
using namespace std;





namespace ParkingPerception
{
    namespace TransparentChassisCpu
    {
        float   TransparentChassis::bicubic(float x)
        {
            float a = -0.75; // opencv取值，默认为-0.5
            float res = 0.0;
            x = abs(x);
            if (x <= 1.0)
                res = (a + 2) * x * x * x - (a + 3) * x * x + 1;
            else if (x < 2.0)
                res = a * x * x * x - 5 * a * x * x + 8 * a * x - 4 * a;
            return res;
        }

        void   TransparentChassis::getImpactFactors(float rowU, float colV, float *rowImFac, float *colImFac, int starti, int startj)
        {
            // 取整
            int row = (int)rowU;
            int col = (int)colV;
            float temp;
            // 计算行系数因子
            for (int i = 0; i < 4; i++)
            {
                if (starti + i <= 0)
                    temp = rowU - row - (starti + i);
                else
                    temp = (starti + i) - (rowU - row);
                rowImFac[i] = bicubic(temp);
                // printf("i:%d,temp:%.1f,rowImFac:%.1f\n", i, temp, rowImFac[i]);
            }
            // 计算列系数因子
            for (int j = 0; j < 4; j++)
            {
                if (startj + j <= 0)
                    temp = colV - col - (startj + j);
                else
                    temp = (startj + j) - (colV - col);
                colImFac[j] = bicubic(temp);
            }
        }

        cv::Vec3b TransparentChassis::bicubic_interpolation(float src_x, float src_y, int width, int height, uint8_t *src, float *rowImFac, float *colImFac)
        {
            // 计算outputMat(row,col)在原图中的位置坐标
            float inputrowf = src_y;
            float inputcolf = src_x;
            // 取整
            int interow = (int)inputrowf;
            int intecol = (int)inputcolf;
            float row_dy = inputrowf - interow;
            float col_dx = inputcolf - intecol;
            int starti = -1, startj = -1;
            // 计算行影响因子，列影响因子
            getImpactFactors(inputrowf, inputcolf, rowImFac, colImFac, starti, startj);
            // 计算输出图像(row,col)的值
            float3 c_3f = {0, 0, 0};
            for (int i = starti; i < starti + 4; i++)
            {
                for (int j = startj; j < startj + 4; j++)
                {
                    uint8_t *src_ptr = src + (interow + i) * 3 * width + (intecol + j) * 3;
                    float weight = rowImFac[i - starti] * colImFac[j - startj];
                    c_3f.x += src_ptr[0] * weight;
                    c_3f.y += src_ptr[1] * weight;
                    c_3f.z += src_ptr[2] * weight;
                }
            }
            c_3f.x = floorf(c_3f.x + 0.5f); // 四舍五入
            c_3f.y = floorf(c_3f.y + 0.5f); // 四舍五入
            c_3f.z = floorf(c_3f.z + 0.5f); // 四舍五入
            return cv::Vec3b(c_3f.x, c_3f.y, c_3f.z);
        }

        cv::Vec3b   TransparentChassis::bilinear_interpolation(float src_x, float src_y, int width, int height, int line_size, uint8_t fill_value, uint8_t *src)
        {
            float c0 = fill_value, c1 = fill_value, c2 = fill_value;
            // 双线性插值
            if (src_x < -1 || src_x >= width || src_y < -1 || src_y >= height)
            {
                // out of range
                // src_x < -1时，其高位high_x < 0，超出范围
                // src_x >= -1时，其高位high_x >= 0，存在取值
            }
            else
            {
                int y_low = floorf(src_y);
                int x_low = floorf(src_x);
                int y_high = y_low + 1;
                int x_high = x_low + 1;

                uint8_t const_values[] = {fill_value, fill_value, fill_value};
                float ly = src_y - y_low;
                float lx = src_x - x_low;
                float hy = 1 - ly;
                float hx = 1 - lx;
                float w1 = hy * hx, w2 = hy * lx, w3 = ly * hx, w4 = ly * lx;
                uint8_t *v1 = const_values;
                uint8_t *v2 = const_values;
                uint8_t *v3 = const_values;
                uint8_t *v4 = const_values;
                if (y_low >= 0)
                {
                    if (x_low >= 0)
                        v1 = src + y_low * line_size + x_low * 3;

                    if (x_high < width)
                        v2 = src + y_low * line_size + x_high * 3;
                }

                if (y_high < height)
                {
                    if (x_low >= 0)
                        v3 = src + y_high * line_size + x_low * 3;

                    if (x_high < width)
                        v4 = src + y_high * line_size + x_high * 3;
                }

                c0 = floorf(w1 * v1[0] + w2 * v2[0] + w3 * v3[0] + w4 * v4[0] + 0.5f); // 四舍五入
                c1 = floorf(w1 * v1[1] + w2 * v2[1] + w3 * v3[1] + w4 * v4[1] + 0.5f); // 四舍五入
                c2 = floorf(w1 * v1[2] + w2 * v2[2] + w3 * v3[2] + w4 * v4[2] + 0.5f); // 四舍五入
            }
            return cv::Vec3b(c0, c1, c2);
        }

        cv::Vec3b   TransparentChassis::optimized_bilinear_interpolation(float src_x, float src_y, int width, int height, int line_size, uint8_t fill_value, uint8_t *src) {
            int x_low = std::max(0, std::min(static_cast<int>(src_x), width - 1));
            int y_low = std::max(0, std::min(static_cast<int>(src_y), height - 1));
            int x_high = std::min(x_low + 1, width - 1);
            int y_high = std::min(y_low + 1, height - 1);

            float lx = src_x - x_low;
            float ly = src_y - y_low;
            float hx = 1 - lx;
            float hy = 1 - ly;

            uint8_t *v1 = src + y_low * line_size + x_low * 3;
            uint8_t *v2 = src + y_low * line_size + x_high * 3;
            uint8_t *v3 = src + y_high * line_size + x_low * 3;
            uint8_t *v4 = src + y_high * line_size + x_high * 3;

            uint8_t c0 = static_cast<uint8_t>(hx * hy * v1[0] + lx * hy * v2[0] + hx * ly * v3[0] + lx * ly * v4[0] + 0.5f);
            uint8_t c1 = static_cast<uint8_t>(hx * hy * v1[1] + lx * hy * v2[1] + hx * ly * v3[1] + lx * ly * v4[1] + 0.5f);
            uint8_t c2 = static_cast<uint8_t>(hx * hy * v1[2] + lx * hy * v2[2] + hx * ly * v3[2] + lx * ly * v4[2] + 0.5f);

            return cv::Vec3b(c0, c1, c2);
        }

        cv::Vec3b   TransparentChassis::nearest_neighbor_interpolation(float src_x, float src_y, int width, int height, uint8_t *src) {
            int x = round(src_x);
            int y = round(src_y);

            // 确保 x 和 y 在图像边界内
            if (x < 0) x = 0;
            if (x >= width) x = width - 1;
            if (y < 0) y = 0;
            if (y >= height) y = height - 1;
            // 计算像素值的指针偏移
            int offset = (y * width + x) * 3;
            // 返回像素值
            return cv::Vec3b(src[offset], src[offset + 1], src[offset + 2]);
        }


        // void process_img_kernel(const Mat &img_pre, Mat &img_fusion, const Mat &img_now, const Mat &weight, int w, int h, const float *affine_now2pre)
        // {
        //     // avm自车黑色区域的位置，平滑卷积核大小
        //     int shift_lr = 470, shift_tb = 470, filter_kernel_size = 75;
        //     // 双线性插值参数
        //     int line_size = w * 3;  // 假设图像的通道数为3
        //     uint8_t fill_value = 0; // 默认填充值
        //     // 双立方插值参数
        //     float rowImFac[4];
        //     float colImFac[4];
        //     // 像素点融合权重
        //     float r;
        //     // 临时变量，存放插值后的像素值            
        //     cv::Vec3b c;            

        //     // 仅对位于平滑后的avm自车黑色区域，进行透明融合
        //     for (int j = (shift_tb - int(filter_kernel_size / 2)); j < (h - shift_tb + int(filter_kernel_size / 2)); ++j)
        //     {
        //         for (int i = (shift_lr - int(filter_kernel_size / 2)); i < (w - shift_lr + int(filter_kernel_size / 2)); ++i)
        //         {
        //             // 确定融合后图像中当前像素的位置
        //             cv::Vec3b &fusion_ptr = img_fusion.at<cv::Vec3b>(j, i);
        //             // 找到权重图和当前图中当前像素的位置
        //             cv::Vec3b weight_ij = weight.at<cv::Vec3b>(j, i);
        //             cv::Vec3b now_ptr = img_now.at<cv::Vec3b>(j, i);

        //             r = weight_ij[0] / 255.0; // 像素点融合权重更新

        //             // 根据当前像素位置和仿射变换矩阵计算出在上一帧图像中的位置坐标
        //             float img_pre_x = affine_now2pre[0] * i + affine_now2pre[1] * j + affine_now2pre[2];
        //             float img_pre_y = affine_now2pre[3] * i + affine_now2pre[4] * j + affine_now2pre[5];

        //             // 判断当前像素是否在上一帧图像中
        //             if (img_pre_x >= 0 && img_pre_x < w && img_pre_y >= 0 && img_pre_y < h)
        //             {
        //                 // 无插值
        //                 c = img_pre.at<cv::Vec3b>(img_pre_y, img_pre_x);
        //                 // 最近邻插值
        //                 // c = nearest_neighbor_interpolation(img_pre_x, img_pre_y, w, h, img_pre.data);
        //                 // 双线性插值
        //                 // c = bilinear_interpolation(img_pre_x, img_pre_y, w, h, 3 * w, 0, img_pre.data);
        //                 // c = optimized_bilinear_interpolation(img_pre_x, img_pre_y, w, h, 3 * w, 0, img_pre.data);
        //                 // 双立方插值
        //                 // c = bicubic_interpolation(img_pre_x, img_pre_y, w, h, img_pre.data, (float *)rowImFac, (float *)colImFac);
        //                 // Lanczos插值
        //                 // c = lanczos_interpolation(img_pre, img_pre_x, img_pre_y, 3); // 调用lanczos_interpolation函数，进行lanczos插值
        //             }
        //             else
        //             {
        //                 // 超出范围，设置为黑色
        //                 c = {0, 0, 0};
        //             }
        //             // 将插值后的像素值赋给融合图像的对应位置
        //             fusion_ptr = c;

        //             if (weight_ij[0] == 0)
        //             { // 权重为0即中间黑框部分，仅使用pre2now，不做融合
        //                 now_ptr = fusion_ptr;
        //             }
        //             else
        //             {
        //                 fusion_ptr[0] = (1 - r) * fusion_ptr[0] + r * now_ptr[0];
        //                 fusion_ptr[1] = (1 - r) * fusion_ptr[1] + r * now_ptr[1];
        //                 fusion_ptr[2] = (1 - r) * fusion_ptr[2] + r * now_ptr[2];
        //             }
        //         }
        //     }
        // }

        // void process_img_kernel(const cv::Mat &img_pre, cv::Mat &img_fusion, const cv::Mat &img_now, const cv::Mat &weight, int w, int h, const float *affine_now2pre)
        // {
        //     int shift_lr = 470, shift_tb = 470, filter_kernel_size = 75;
        //     float rowImFac[4];
        //     float colImFac[4];
        //     float r;
        //     cv::Vec3b c;

        //     // 使用指针访问像素
        //     for (int j = (shift_tb - int(filter_kernel_size / 2)); j < (h - shift_tb + int(filter_kernel_size / 2)); ++j)
        //     {
        //         cv::Vec3b* fusion_row = img_fusion.ptr<cv::Vec3b>(j);
        //         const cv::Vec3b* weight_row = weight.ptr<cv::Vec3b>(j);
        //         const cv::Vec3b* now_row = img_now.ptr<cv::Vec3b>(j);

        //         for (int i = (shift_lr - int(filter_kernel_size / 2)); i < (w - shift_lr + int(filter_kernel_size / 2)); ++i)
        //         {
        //             cv::Vec3b &fusion_ptr = fusion_row[i];
        //             const cv::Vec3b &weight_ij = weight_row[i];
        //             const cv::Vec3b &now_ptr = now_row[i];

        //             r = weight_ij[0] / 255.0;

        //             float img_pre_x = affine_now2pre[0] * i + affine_now2pre[1] * j + affine_now2pre[2];
        //             float img_pre_y = affine_now2pre[3] * i + affine_now2pre[4] * j + affine_now2pre[5];

        //             if (img_pre_x >= 0 && img_pre_x < w && img_pre_y >= 0 && img_pre_y < h)
        //             {
        //                 c = img_pre.at<cv::Vec3b>(img_pre_y, img_pre_x);
        //             }
        //             else
        //             {
        //                 c = {0, 0, 0};
        //             }

        //             fusion_ptr = c;

        //             if (weight_ij[0] != 0)
        //             {
        //                 fusion_ptr[0] = (1 - r) * fusion_ptr[0] + r * now_ptr[0];
        //                 fusion_ptr[1] = (1 - r) * fusion_ptr[1] + r * now_ptr[1];
        //                 fusion_ptr[2] = (1 - r) * fusion_ptr[2] + r * now_ptr[2];
        //             }
        //         }
        //     }
        // }

        void   TransparentChassis::process_block(const cv::Mat &img_pre, cv::Mat &img_fusion, const cv::Mat &img_now, const cv::Mat &weight, int w, int h, const float *affine_now2pre, int start_row, int end_row)
        {
            int shift_lr = 470, shift_tb = 470, filter_kernel_size = 75;
            int line_size = w * 3;
            uint8_t fill_value = 0;
            float rowImFac[4];
            float colImFac[4];
            float r;
            cv::Vec3b c;

            for (int j = start_row; j < end_row; ++j)
            {
                cv::Vec3b* fusion_row = img_fusion.ptr<cv::Vec3b>(j);
                const cv::Vec3b* weight_row = weight.ptr<cv::Vec3b>(j);
                const cv::Vec3b* now_row = img_now.ptr<cv::Vec3b>(j);

                for (int i = (shift_lr - int(filter_kernel_size / 2)); i < (w - shift_lr + int(filter_kernel_size / 2)); ++i)
                {
                    cv::Vec3b &fusion_ptr = fusion_row[i];
                    const cv::Vec3b &weight_ij = weight_row[i];
                    const cv::Vec3b &now_ptr = now_row[i];

                    r = weight_ij[0] / 255.0;

                    float img_pre_x = affine_now2pre[0] * i + affine_now2pre[1] * j + affine_now2pre[2];
                    float img_pre_y = affine_now2pre[3] * i + affine_now2pre[4] * j + affine_now2pre[5];

                    if (img_pre_x >= 0 && img_pre_x < w && img_pre_y >= 0 && img_pre_y < h)
                    {
                        // c = img_pre.at<cv::Vec3b>(img_pre_y, img_pre_x);
                        c = optimized_bilinear_interpolation(img_pre_x, img_pre_y, w, h, 3 * w, 0, img_pre.data);
                        // c = bicubic_interpolation(img_pre_x, img_pre_y, w, h, img_pre.data, (float *)rowImFac, (float *)colImFac);
                    }
                    else
                    {
                        c = {0, 0, 0};
                    }

                    fusion_ptr = c;

                    if (weight_ij[0] != 0)
                    {
                        fusion_ptr[0] = (1 - r) * fusion_ptr[0] + r * now_ptr[0];
                        fusion_ptr[1] = (1 - r) * fusion_ptr[1] + r * now_ptr[1];
                        fusion_ptr[2] = (1 - r) * fusion_ptr[2] + r * now_ptr[2];
                    }
                }
            }
        }

        void   TransparentChassis::process_img_kernel(const cv::Mat &img_pre, cv::Mat &img_fusion, const cv::Mat &img_now, const cv::Mat &weight, int w, int h, const float *affine_now2pre)
        {
            int shift_lr = 470, shift_tb = 470, filter_kernel_size = 75;
            int num_blocks = 16; // 可以根据需要调整块的数量
            int block_size = (h - 2 * shift_tb + filter_kernel_size) / num_blocks; //39

            std::vector<std::thread> threads;

            for (int block = 0; block < num_blocks; ++block)
            {
                int start_row = shift_tb - int(filter_kernel_size / 2) + block * block_size;//470-37=433
                int end_row = start_row + block_size;//433+39=472
                if (block == num_blocks - 1)
                {
                    end_row = h - shift_tb + int(filter_kernel_size / 2);
                }
                // threads.emplace_back(process_block, std::ref(img_pre), std::ref(img_fusion), std::ref(img_now), std::ref(weight), w, h, affine_now2pre, start_row, end_row);
                threads.emplace_back(std::bind(&TransparentChassis::process_block, this, std::ref(img_pre), std::ref(img_fusion), std::ref(img_now), std::ref(weight), w, h, affine_now2pre, start_row, end_row));

            }

            for (auto &thread : threads)
            {
                thread.join();
            }
        }   

        TransparentChassis::TransparentChassis(TransparentChassisData config_param) : config_param_(config_param)
        {
        }

        TransparentChassis::~TransparentChassis()
        {
            destroy();
        }

        int TransparentChassis::init()
        {
            if (0 != load_config())
            {
                std::cout << "[TransparentChassis]->[init] Failed to load config file." << std::endl;
                return -1;
            }

            // 初始化融合权重
            cv::Mat weight_tmp(cv::Size(w, h), CV_8UC3, cv::Scalar(255, 255, 255));
            cv::rectangle(
                weight_tmp, cv::Point(shift_lr - int(filter_kernel_size / 2), shift_tb - int(filter_kernel_size / 2)),
                cv::Point(w - shift_lr + int(filter_kernel_size / 2), h - shift_tb + int(filter_kernel_size / 2)),
                cv::Scalar{0}, -1); // weight中间部分置黑，中间只用前一帧变换过来的图,外围只用前一帧变换后的图，中间平滑过渡
                                                                      
            cv::blur(weight_tmp, weight, cv::Size(filter_kernel_size, filter_kernel_size)); // 卷积核大小，根据透明部分是否有波纹修改，越大波纹越不明显，但是会变模糊
            // 初始化仿射变换矩阵
            float affine_now2pre[6];
            // 初始化首帧
            is_first = true;
            // 融合结果分配内存
            output.create(h, w, CV_8UC3);
            std::cout << "[TransparentChassis]->[init] Init success." << std::endl;
            return 0;
        }

        int TransparentChassis::load_config()
        {
 
            w = config_param_.bev_w;
            h = config_param_.bev_h;
            if (w == 0 || h == 0)
            {
                std::cout << "[TransparentChassis]->[load_config] Img_params size error!!!" << std::endl;
                return -1;
            }
            bev_ratio = config_param_.bev_ratio;
            bev_center_x = config_param_.bev_center_x;
            bev_center_y = config_param_.bev_center_y;
            shift_lr = config_param_.shift_lr;
            shift_tb = config_param_.shift_tb;
            if (shift_lr == 0 || shift_tb == 0)
            {
                std::cout << "[TransparentChassis]->[load_config] shift_lr or shift_tb error!!!" << std::endl;
                return -1;
            }
            filter_kernel_size = config_param_.filter_kernel_size;
            if (filter_kernel_size == 0)
            {
                std::cout << "[TransparentChassis]->[load_config] filter_kernel_size error!!!" << std::endl;
                return -1;
            }

            return 0;
        }

        int TransparentChassis::process(const cv::Mat &img_now, const LocData &loc_now)
        {
            // 判断输入图像是否正常
            if (img_now.empty())
            {
                std::cout << "[TransparentChassis]->[process] Input img is empty!!!" << std::endl;
                return -1;
            }
            if (img_now.rows != h || img_now.cols != w)
            {
                std::cout << "[TransparentChassis]->[process] Input img size error!!!" << std::endl;
                return -1;
            }

            // 如果为首帧，初始化img_pre_device和loc_data_pre
            if (is_first)
            {
                // 确保 img_now 是三通道图像
                assert(img_now.channels() == 3);
                img_pre = img_now.clone();
                loc_data_pre = loc_now;
                is_first = false;
                output = img_now.clone();
                return 0;
            }
            // 计算从上一帧到当前帧的仿射变换
            get_warpaffine(loc_now);
            // 图像变换融合
            Mat img_fusion = img_now.clone();
            process_img_kernel(img_pre, img_fusion, img_now, weight, w, h, affine_now2pre);
            // 当前帧融合结果用于下一帧融合
            img_pre = img_fusion.clone();
            loc_data_pre = loc_now;

            // 输出结果
            output = img_fusion.clone();

            return 0;
        }

        void TransparentChassis::get_result(cv::Mat &out)
        {
            out = output.clone();
        }

        void TransparentChassis::get_warpaffine(const LocData &loc_now)
        {
            // 前一帧的car->global
            double x_pre = loc_data_pre.x;
            double y_pre = loc_data_pre.y;
            double theta_pre = loc_data_pre.yaw;
            cv::Mat RT_c2g_pre = cv::Mat::eye(4, 4, CV_64F);
            RT_c2g_pre.at<double>(0, 0) = cos(theta_pre);
            RT_c2g_pre.at<double>(0, 1) = -sin(theta_pre);
            RT_c2g_pre.at<double>(0, 2) = 0;
            RT_c2g_pre.at<double>(0, 3) = x_pre;
            RT_c2g_pre.at<double>(1, 0) = sin(theta_pre);
            RT_c2g_pre.at<double>(1, 1) = cos(theta_pre);
            RT_c2g_pre.at<double>(1, 2) = 0;
            RT_c2g_pre.at<double>(1, 3) = y_pre;
            // 当前帧的car->global
            double x_now = loc_now.x;
            double y_now = loc_now.y;
            double theta_now = loc_now.yaw;
            cv::Mat RT_c2g_now = cv::Mat::eye(4, 4, CV_64F);
            RT_c2g_now.at<double>(0, 0) = cos(theta_now);
            RT_c2g_now.at<double>(0, 1) = -sin(theta_now);
            RT_c2g_now.at<double>(0, 2) = 0;
            RT_c2g_now.at<double>(0, 3) = x_now;
            RT_c2g_now.at<double>(1, 0) = sin(theta_now);
            RT_c2g_now.at<double>(1, 1) = cos(theta_now);
            RT_c2g_now.at<double>(1, 2) = 0;
            RT_c2g_now.at<double>(1, 3) = y_now;
            // img->car
            cv::Mat RT_i2c = cv::Mat::eye(4, 4, CV_64F);
            RT_i2c.at<double>(0, 0) = 0;
            RT_i2c.at<double>(0, 1) = -1 * bev_ratio;
            RT_i2c.at<double>(0, 2) = 0;
            RT_i2c.at<double>(0, 3) = (bev_center_y - 12) * bev_ratio; // Y
            RT_i2c.at<double>(1, 0) = -1 * bev_ratio;
            RT_i2c.at<double>(1, 1) = 0;
            RT_i2c.at<double>(1, 2) = 0;
            RT_i2c.at<double>(1, 3) = bev_center_x * bev_ratio; // X
            RT_i2c.at<double>(2, 0) = 0;
            RT_i2c.at<double>(2, 1) = 0;
            RT_i2c.at<double>(2, 2) = -1 * bev_ratio;
            RT_i2c.at<double>(2, 3) = 0;
            // 上一帧图到当前图的变换
            cv::Mat RT_pre2now = RT_i2c.inv() * RT_c2g_now.inv() * RT_c2g_pre * RT_i2c;
            cv::Mat RT_now2pre = RT_pre2now.inv();
            // 计算仿射变换矩阵
            affine_now2pre[0] = float(RT_now2pre.at<double>(0, 0));
            affine_now2pre[1] = float(RT_now2pre.at<double>(0, 1));
            affine_now2pre[2] = float(RT_now2pre.at<double>(0, 3));
            affine_now2pre[3] = float(RT_now2pre.at<double>(1, 0));
            affine_now2pre[4] = float(RT_now2pre.at<double>(1, 1));
            affine_now2pre[5] = float(RT_now2pre.at<double>(1, 3));
        }

        void TransparentChassis::destroy()
        {
            // 在纯 CPU 代码中，不需要手动释放内存
        }
    }
}
