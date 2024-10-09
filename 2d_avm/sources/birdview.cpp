#include <birdview.h>

BirdView::BirdView(
    int xl, int xr, int yt, int yb,
    int total_h, int total_w) {
    this->xl = xl;
    this->xr = xr;
    this->yt = yt;
    this->yb = yb;
    image = cv::Mat(total_h, total_w, CV_8UC3, cv::Scalar(0, 0, 0));
}

cv::Mat BirdView::merge(const cv::Mat &imA, const cv::Mat &imB, int k) {
    // 获取权重矩阵
    cv::Mat G = weights[k];
    // 检查输入图像和权重矩阵的大小和类型是否一致
    if (imA.size() != imB.size() || imA.size() != G.size() || imA.type() != imB.type()) {
        std::cout << "imA.size:" << imA.size() << " imB.size:" << imB.size() << " G.size:" << G.size() << " imA.type:" << imA.type() << " imB.type:" << imB.type() << std::endl;
        throw std::invalid_argument("Input images and weight matrix must have the same size and compatible types.");
    }
    // 将输入图像转换为浮点类型进行计算
    cv::Mat merged_float, imA_float, imB_float;
    cv::Mat ones(G.rows, G.cols, CV_32FC3, cv::Scalar(1, 1, 1));
    cv::Mat dst_float, out_a, out_b;
    cv::subtract(ones, G, dst_float);
    // std::cout << "dst_float.size:" << dst_float.size() << " dst_float.type:" << dst_float << std::endl;
    // 合并图像
    cv::multiply(imA, G, out_a);
    cv::multiply(imB, dst_float, out_b);
    merged_float = out_a + out_b;
    return merged_float;
}


cv::Mat BirdView::merge(const cv::Mat &imA, const cv::Mat &imB, cv::Mat G) {
    // 将输入图像转换为浮点类型进行计算
    cv::Mat merged_float(G.rows, G.cols, CV_32FC3, cv::Scalar(1, 1, 1)), imA_float, imB_float;
    cv::Mat ones(G.rows, G.cols, CV_32FC3, cv::Scalar(1, 1, 1));
    cv::Mat dst_float, out_a, out_b;
    cv::subtract(ones, G, dst_float);
    // 合并图像
    cv::multiply(imA, G, out_a);
    cv::multiply(imB, dst_float, out_b);
    merged_float = out_a + out_b;

    return merged_float;
}

void BirdView::stitch_all_parts() {
    cv::Mat front = frames[0];
    cv::Mat back = frames[1];
    cv::Mat left = frames[2];
    cv::Mat right = frames[3];
    front.convertTo(front, CV_32FC3);
    back.convertTo(back, CV_32FC3);
    left.convertTo(left, CV_32FC3);
    right.convertTo(right, CV_32FC3);

    // Copy parts to image
    FM(front).copyTo(getF());
    BM(back).copyTo(getB());
    LM(left).copyTo(getL());
    RM(right).copyTo(getR());

    // Merge and copy blended parts to image
    cv::Mat mergedFL = merge(FI(front), LI(left), 0);
    cv::Mat mergedFR = merge(FII(front), RII(right), 1);
    cv::Mat mergedBL = merge(BIII(back), LIII(left), 2);
    cv::Mat mergedBR = merge(BIV(back), RIV(right), 3);
    mergedFL.copyTo(getFL());
    mergedFR.copyTo(getFR());
    mergedBL.copyTo(getBL());
    mergedBR.copyTo(getBR());
}

void BirdView::copy_car_image(const cv::Mat &car_image) {
    cv::Rect roi(xl, yt, xr - xl, yb - yt); // 创建感兴趣区域
    cv::Mat destination_roi = image(roi);   // 获取感兴趣区域的引用
    car_image.copyTo(destination_roi);      // 将汽车图像复制到感兴趣区域
}

void BirdView::make_luminance_balance(const std::vector<cv::Mat> &images) {
    const cv::Mat front = images[0];
    const cv::Mat back = images[1];  
    const cv::Mat left = images[2]; 
    const cv::Mat right = images[3];

    cv::Mat Fb, Fg, Fr, Bb, Bg, Br, Lb, Lg, Lr, Rb, Rg, Rr;
    // 拆分通道
    std::vector<cv::Mat> frontChannels, backChannels, leftChannels, rightChannels;
    cv::split(front, frontChannels);
    cv::split(back, backChannels);
    cv::split(left, leftChannels);
    cv::split(right, rightChannels);
    Fb = frontChannels[0];
    Fg = frontChannels[1];
    Fr = frontChannels[2];
    Bb = backChannels[0];
    Bg = backChannels[1];
    Br = backChannels[2];
    Lb = leftChannels[0];
    Lg = leftChannels[1];
    Lr = leftChannels[2];
    Rb = rightChannels[0];
    Rg = rightChannels[1];
    Rr = rightChannels[2];

    // 计算均值比率
    double a1 = ImageProcessing::mean_luminance_ratio(RII(Rb), FII(Fb), masks[1]);
    double a2 = ImageProcessing::mean_luminance_ratio(RII(Rg), FII(Fg), masks[1]);
    double a3 = ImageProcessing::mean_luminance_ratio(RII(Rr), FII(Fr), masks[1]);

    double b1 = ImageProcessing::mean_luminance_ratio(BIV(Bb), RIV(Rb), masks[3]);
    double b2 = ImageProcessing::mean_luminance_ratio(BIV(Bg), RIV(Rg), masks[3]);
    double b3 = ImageProcessing::mean_luminance_ratio(BIV(Br), RIV(Rr), masks[3]);

    double c1 = ImageProcessing::mean_luminance_ratio(LIII(Lb), BIII(Bb), masks[2]);
    double c2 = ImageProcessing::mean_luminance_ratio(LIII(Lg), BIII(Bg), masks[2]);
    double c3 = ImageProcessing::mean_luminance_ratio(LIII(Lr), BIII(Br), masks[2]);

    double d1 = ImageProcessing::mean_luminance_ratio(FI(Fb), LI(Lb), masks[0]);
    double d2 = ImageProcessing::mean_luminance_ratio(FI(Fg), LI(Lg), masks[0]);
    double d3 = ImageProcessing::mean_luminance_ratio(FI(Fr), LI(Lr), masks[0]);
    // 计算调整系数
    double t1 = pow((a1 * b1 * c1 * d1), 0.25);
    double t2 = pow((a2 * b2 * c2 * d2), 0.25);
    double t3 = pow((a3 * b3 * c3 * d3), 0.25);

    double x1 = t1 / sqrt(d1 / a1);
    double x2 = t2 / sqrt(d2 / a2);
    double x3 = t3 / sqrt(d3 / a3);

    x1 = (x1 >= 1) ? x1 * exp((1 - x1) * 0.5) : x1 * exp((1 - x1) * 0.8);
    x2 = (x2 >= 1) ? x2 * exp((1 - x2) * 0.5) : x2 * exp((1 - x2) * 0.8);
    x3 = (x3 >= 1) ? x3 * exp((1 - x3) * 0.5) : x3 * exp((1 - x3) * 0.8);

    Fb = ImageProcessing::adjust_luminance(Fb, x1);
    Fg = ImageProcessing::adjust_luminance(Fg, x2);
    Fr = ImageProcessing::adjust_luminance(Fr, x3);

    float y1 = t1 / std::sqrt(b1 / c1);
    float y2 = t2 / std::sqrt(b2 / c2);
    float y3 = t3 / std::sqrt(b3 / c3);
    y1 = (y1 >= 1) ? y1 * exp((1 - y1) * 0.5) : y1 * exp((1 - y1) * 0.8);
    y2 = (y2 >= 1) ? y2 * exp((1 - y2) * 0.5) : y2 * exp((1 - y2) * 0.8);
    y3 = (y3 >= 1) ? y3 * exp((1 - y3) * 0.5) : y3 * exp((1 - y3) * 0.8);
    Bb = ImageProcessing::adjust_luminance(Bb, y1);
    Bg = ImageProcessing::adjust_luminance(Bg, y2);
    Br = ImageProcessing::adjust_luminance(Br, y3);

    float z1 = t1 / std::sqrt(c1 / d1);
    float z2 = t2 / std::sqrt(c2 / d2);
    float z3 = t3 / std::sqrt(c3 / d3);
    z1 = (z1 >= 1) ? z1 * exp((1 - z1) * 0.5) : z1 * exp((1 - z1) * 0.8);
    z2 = (z2 >= 1) ? z2 * exp((1 - z2) * 0.5) : z2 * exp((1 - z2) * 0.8);
    z3 = (z3 >= 1) ? z3 * exp((1 - z3) * 0.5) : z3 * exp((1 - z3) * 0.8);
    Lb = ImageProcessing::adjust_luminance(Lb, z1);
    Lg = ImageProcessing::adjust_luminance(Lg, z2);
    Lr = ImageProcessing::adjust_luminance(Lr, z3);

    float w1 = t1 / std::sqrt(a1 / b1);
    float w2 = t2 / std::sqrt(a2 / b2);
    float w3 = t3 / std::sqrt(a3 / b3);
    w1 = (w1 >= 1) ? w1 * exp((1 - w1) * 0.5) : w1 * exp((1 - w1) * 0.8);
    w2 = (w2 >= 1) ? w2 * exp((1 - w2) * 0.5) : w2 * exp((1 - w2) * 0.8);
    w3 = (w3 >= 1) ? w3 * exp((1 - w3) * 0.5) : w3 * exp((1 - w3) * 0.8);
    Rb = ImageProcessing::adjust_luminance(Rb, w1);
    Rg = ImageProcessing::adjust_luminance(Rg, w2);
    Rr = ImageProcessing::adjust_luminance(Rr, w3);

    frontChannels[0] = Fb;
    frontChannels[1] = Fg;
    frontChannels[2] = Fr;
    backChannels[0] = Bb;
    backChannels[1] = Bg;
    backChannels[2] = Br;
    leftChannels[0] = Lb;
    leftChannels[1] = Lg;
    leftChannels[2] = Lr;
    rightChannels[0] = Rb;
    rightChannels[1] = Rg;
    rightChannels[2] = Rr;

    // 合并通道
    cv::merge(frontChannels, frames[0]);
    cv::merge(backChannels, frames[1]);
    cv::merge(leftChannels, frames[2]);
    cv::merge(rightChannels, frames[3]);
}

std::pair<cv::Mat, cv::Mat> BirdView::get_weights_and_masks(const std::vector<cv::Mat> &images) {
    const cv::Mat front = images[0]; //[1140 x 480]
    const cv::Mat back = images[1];  //[1140 x 510]
    const cv::Mat left = images[2];  // [470 x 1500]
    const cv::Mat right = images[3]; //[1140 x 480]
    cv::Mat G0, M0, G1, M1, G2, M2, G3, M3;

    std::tie(G0, M0) = ImageProcessing::get_weight_mask_matrix(FI(front), LI(left));
    std::tie(G1, M1) = ImageProcessing::get_weight_mask_matrix(FII(front), RII(right));
    std::tie(G2, M2) = ImageProcessing::get_weight_mask_matrix(BIII(back), LIII(left)); // 470,510-----470,510
    std::tie(G3, M3) = ImageProcessing::get_weight_mask_matrix(BIV(back), RIV(right));  // 470,510-----470,510
    weights = {cv::Mat::zeros(G0.size(), CV_32F),
               cv::Mat::zeros(G0.size(), CV_32F),
               cv::Mat::zeros(G0.size(), CV_32F),
               cv::Mat::zeros(G0.size(), CV_32F)};
    masks = {cv::Mat::zeros(M0.size(), CV_32F),
             cv::Mat::zeros(M0.size(), CV_32F),
             cv::Mat::zeros(M0.size(), CV_32F),
             cv::Mat::zeros(M0.size(), CV_32F)};

    cv::Mat GArrays[] = {G0, G1, G2, G3};
    cv::Mat MArrays[] = {M0, M1, M2, M3};

    for (int i = 0; i < 4; ++i) {
        cv::merge(std::vector<cv::Mat>(3, GArrays[i]), weights[i]);     
        masks[i] = MArrays[i] / 255.0f;
    }

    return std::make_pair(weights[0], masks[0]); // Just returning G0 and M0 as an example
}

void BirdView::make_white_balance() {
    // 实现
    image = ImageProcessing::make_white_balance_u(image);
}

void convertTo32FC3(cv::Mat &image) {
    image.convertTo(image, CV_32FC3);
}

void BirdView::mergeParts(const cv::Mat &front, const cv::Mat &back, const cv::Mat &left, const cv::Mat &right,
                          const std::vector<cv::Mat> &weights_in, cv::Mat &mergedFL, cv::Mat &mergedFR,
                          cv::Mat &mergedBL, cv::Mat &mergedBR) {
    // 并行处理四个 merge 操作
    std::thread t1([&]() { mergedFL = merge(FI(front), LI(left), weights_in[0]); });
    std::thread t2([&]() { mergedFR = merge(FII(front), RII(right), weights_in[1]); });
    std::thread t3([&]() { mergedBL = merge(BIII(back), LIII(left), weights_in[2]); });
    std::thread t4([&]() { mergedBR = merge(BIV(back), RIV(right), weights_in[3]); });

    t1.join();
    t2.join();
    t3.join();
    t4.join();
}

void BirdView::stitch_all_parts(std::vector<cv::Mat> &weights_in) {
    cv::Mat front = frames[0];
    cv::Mat back = frames[1];
    cv::Mat left = frames[2];
    cv::Mat right = frames[3];
    // 使用线程进行并行处理
    std::vector<std::thread> threads;
    threads.emplace_back(convertTo32FC3, std::ref(front));
    threads.emplace_back(convertTo32FC3, std::ref(back));
    threads.emplace_back(convertTo32FC3, std::ref(left));
    threads.emplace_back(convertTo32FC3, std::ref(right));

    // 等待所有线程完成
    for (auto &thread : threads) {
        thread.join();
    }

    // Copy parts to image
    FM(front).copyTo(getF());
    BM(back).copyTo(getB());
    LM(left).copyTo(getL());
    RM(right).copyTo(getR());

    // Merge and copy blended parts to image
    // 合并图像并拷贝到相应位置
    cv::Mat mergedFL, mergedFR, mergedBL, mergedBR;
    mergeParts(front, back, left, right, weights_in, mergedFL, mergedFR, mergedBL, mergedBR);
    mergedFL.copyTo(getFL());
    mergedFR.copyTo(getFR());
    mergedBL.copyTo(getBL());
    mergedBR.copyTo(getBR());
}

void BirdView::save_weights_and_masks(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        throw std::runtime_error("Could not open file for writing weights and masks.");
    }

    for (int i = 0; i < weights.size(); ++i) {
        fs << "weight_" + std::to_string(i) << weights[i];
        fs << "mask_" + std::to_string(i) << masks[i];
    }

    fs.release();
}

void BirdView::load_weights_and_masks(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Could not open file for reading weights and masks.");
    }

    weights.clear();
    masks.clear();

    for (int i = 0; i < 4; ++i) // Assuming there are 4 weights and masks
    {
        cv::Mat weight, mask;
        fs["weight_" + std::to_string(i)] >> weight;
        fs["mask_" + std::to_string(i)] >> mask;
        weights.push_back(weight);
        masks.push_back(mask);
    }

    fs.release();
}
