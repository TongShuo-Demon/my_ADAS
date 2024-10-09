#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen>
#include <iostream>
#include <thread>
#include "image_processing.h"
#include "birdview.h"

class BirdView {
public:
    BirdView(int xl, int xr, int yt, int yb,
             int total_h, int total_w);

    cv::Mat FI(const cv::Mat &front_image) const {
        return front_image(cv::Rect(0, 0, xl, front_image.rows));
    }
    cv::Mat FII(const cv::Mat &front_image) const {
        return front_image(cv::Rect(xr, 0, front_image.cols - xr, front_image.rows));
    }
    cv::Mat FM(const cv::Mat &front_image) const {
        return front_image(cv::Rect(xl, 0, xr - xl, front_image.rows));
    }

    cv::Mat BIII(const cv::Mat &back_image) const {
        return back_image(cv::Rect(0, 0, xl, back_image.rows));
    }
    cv::Mat BIV(const cv::Mat &back_image) const {
        return back_image(cv::Rect(xr, 0, back_image.cols - xr, back_image.rows));
    }
    cv::Mat BM(const cv::Mat &back_image) const {
        return back_image(cv::Rect(xl, 0, xr - xl, back_image.rows));
    }

    cv::Mat LI(const cv::Mat &left_image) const {
        return left_image(cv::Rect(0, 0, left_image.cols, yt));
    }
    cv::Mat LIII(const cv::Mat &left_image) const {
        return left_image(cv::Rect(0, yb, left_image.cols, left_image.rows - yb));
    }
    cv::Mat LM(const cv::Mat &left_image) const {
        return left_image(cv::Rect(0, yt, left_image.cols, yb - yt));
    }

    cv::Mat RII(const cv::Mat &right_image) const {
        return right_image(cv::Rect(0, 0, right_image.cols, yt));
    }
    cv::Mat RIV(const cv::Mat &right_image) const {
        return right_image(cv::Rect(0, yb, right_image.cols, right_image.rows - yb));
    }
    cv::Mat RM(const cv::Mat &right_image) const {
        return right_image(cv::Rect(0, yt, right_image.cols, yb - yt));
    }

    cv::Mat getFL() {
        return image(cv::Rect(0, 0, xl, yt));
    }
    cv::Mat getF() {
        return image(cv::Rect(xl, 0, xr - xl, yt));
    }
    cv::Mat getFR() {
        return image(cv::Rect(xr, 0, image.cols - xr, yt));
    }

    cv::Mat getBL() {
        return image(cv::Rect(0, yb, xl, image.rows - yb));
    }
    cv::Mat getB() {
        return image(cv::Rect(xl, yb, xr - xl, image.rows - yb));
    }
    cv::Mat getBR() {
        return image(cv::Rect(xr, yb, image.cols - xr, image.rows - yb));
    }

    cv::Mat getL() {
        return image(cv::Rect(0, yt, xl, yb - yt));
    }
    cv::Mat getR() {
        return image(cv::Rect(xr, yt, image.cols - xr, yb - yt));
    }
    cv::Mat getC() {
        return image(cv::Rect(xl, yt, xr - xl, yb - yt));
    }

    void update_frames(const std::vector<cv::Mat> &images) {
        frames = images;
    }

    cv::Mat merge(const cv::Mat &imA, const cv::Mat &imB, int k);     // cv::Mat G
    cv::Mat merge(const cv::Mat &imA, const cv::Mat &imB, cv::Mat G); // cv::Mat G
    void mergeParts(const cv::Mat &front, const cv::Mat &back, const cv::Mat &left, const cv::Mat &right,
                    const std::vector<cv::Mat> &weights_in, cv::Mat &mergedFL, cv::Mat &mergedFR,
                    cv::Mat &mergedBL, cv::Mat &mergedBR);

    void stitch_all_parts();
    void stitch_all_parts(std::vector<cv::Mat> &weights_in);

    void copy_car_image(const cv::Mat &car_image);

    void make_luminance_balance(const std::vector<cv::Mat> &images);

    std::pair<cv::Mat, cv::Mat> get_weights_and_masks(const std::vector<cv::Mat> &images);
    void save_weights_and_masks(const std::string &filename);
    void load_weights_and_masks(const std::string &filename);
    void make_white_balance();
    cv::Mat image;

private:
    int xl;
    int xr;
    int yt;
    int yb;

    std::vector<cv::Mat> frames;
    std::vector<cv::Mat> weights;
    std::vector<cv::Mat> masks;
    cv::Mat car_image;
};
