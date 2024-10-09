#include "image_processing.h"

// 定义命名空间
namespace ImageProcessing {


// 计算灰度图像在掩码下的平均值
double get_mean_statistisc(const cv::Mat &gray, const cv::Mat &mask) {
    CV_Assert(gray.size() == mask.size() && gray.type() == CV_8UC1 && mask.type() == CV_8UC1);
    double total = 0;
    for (int i = 0; i < gray.rows; ++i) {
        const uchar *grayRow = gray.ptr<uchar>(i);
        const uchar *maskRow = mask.ptr<uchar>(i);
        for (int j = 0; j < gray.cols; ++j) {
            if (maskRow[j] != 0) {
                total += grayRow[j];
            }
        }
    }
    return total;
}

double mean_luminance_ratio(const cv::Mat &grayA, const cv::Mat &grayB, const cv::Mat &mask) {
    CV_Assert(grayA.size() == grayB.size() && grayA.size() == mask.size() && grayA.type() == CV_8UC1 && grayB.type() == CV_8UC1 && mask.type() == CV_8UC1);

    double totalA = get_mean_statistisc(grayA, mask);
    double totalB = get_mean_statistisc(grayB, mask);
    return totalA / totalB;
}

// 定义调整亮度的函数
cv::Mat adjust_luminance(const cv::Mat &gray, double factor) {
    CV_Assert(gray.type() == CV_8UC1);
    cv::Mat adjusted;
    cv::multiply(gray, factor, adjusted);
    cv::Mat result;
    cv::convertScaleAbs(adjusted, result);
    return result;
}

cv::Mat get_mask(const cv::Mat &img) {
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat mask = cv::Mat_<uchar>(img.rows, img.cols);
    cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY);
    return mask;
}

cv::Mat get_overlap_region_mask(const cv::Mat &imA, const cv::Mat &imB) {
    cv::Mat overlap;
    cv::bitwise_and(imA, imB, overlap);
    cv::Mat mask = get_mask(overlap);
    cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)), cv::Point(-1, -1), 2);
    return mask;
}

std::vector<cv::Point> get_outmost_polygon_boundary(const cv::Mat &img) {
    cv::Mat mask = get_mask(img);
    cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Get the contour with the largest area
    double maxArea = 0;
    std::vector<cv::Point> largestContour;
    for (const auto &contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea) {
            maxArea = area;
            largestContour = contour;
        }
    }

    // Polygon approximation
    std::vector<cv::Point> polygon;
    cv::approxPolyDP(largestContour, polygon, 0.009 * cv::arcLength(largestContour, true), true);
    return polygon;
}

std::pair<cv::Mat, cv::Mat> get_weight_mask_matrix(const cv::Mat &imA, const cv::Mat &imB, int dist_threshold) {
    cv::Mat overlapMask = get_overlap_region_mask(imA, imB);
    cv::Mat overlapMaskInv;
    cv::bitwise_not(overlapMask, overlapMaskInv);

    cv::Mat imA_diff, imB_diff;
    cv::bitwise_and(imA, imA, imA_diff, overlapMaskInv);
    cv::bitwise_and(imB, imB, imB_diff, overlapMaskInv);

    cv::Mat G = cv::Mat_<float>(overlapMask.rows, overlapMask.cols);
    G = get_mask(imA);
    G.convertTo(G, CV_32F, 1.0 / 255.0);

    std::vector<cv::Point> polyA = get_outmost_polygon_boundary(imA_diff);
    std::vector<cv::Point> polyB = get_outmost_polygon_boundary(imB_diff);

    for (int y = 0; y < G.rows; y++) {
        for (int x = 0; x < G.cols; x++) {
            if (static_cast<int>(overlapMask.at<uchar>(y, x)) == 255) {
                float distToB = cv::pointPolygonTest(polyB, cv::Point(x, y), true);
                if (distToB < dist_threshold) {
                    float distToA = cv::pointPolygonTest(polyA, cv::Point(x, y), true);
                    distToB *= distToB;
                    distToA *= distToA;
                    G.at<float>(y, x) = distToB / (distToA + distToB + 1e-5f);
                }
            }
        }
    }
    return std::make_pair(G, overlapMask);
}

// 白平衡调整函数
cv::Mat make_white_balance_u(const cv::Mat &image) {
    std::vector<cv::Mat> channels;
    cv::split(image, channels); // 分割成 B, G, R 通道

    double m1 = cv::mean(channels[0])[0]; // 计算 B 通道均值
    double m2 = cv::mean(channels[1])[0]; // 计算 G 通道均值
    double m3 = cv::mean(channels[2])[0]; // 计算 R 通道均值

    double K = (m1 + m2 + m3) / 3.0; // 计算所有通道均值的均值
    double c1 = K / m1;
    double c2 = K / m2;
    double c3 = K / m3;

    // 调整每个通道的亮度
    channels[0] = adjust_luminance(channels[0], c1);
    channels[1] = adjust_luminance(channels[1], c2);
    channels[2] = adjust_luminance(channels[2], c3);

    // 合并通道
    cv::Mat balanced_image;
    cv::merge(channels, balanced_image);
    return balanced_image;
}

}