#include "algorithm/laser_extractor.h"
#include <algorithm>
#include <cmath>

LaserExtractor::LaserExtractor(int threshold, int gaussKernelSize)
    : m_threshold(threshold), m_gaussKernelSize(gaussKernelSize) {}

// 提取单帧图像中的激光中心点
std::vector<Point2D> LaserExtractor::extractCenter(const cv::Mat& image) {
    std::vector<Point2D> laserPoints;
    if (image.empty()) return laserPoints;

    // 先在 HSV 空间生成候选掩膜
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255), mask);

    // 在绿色通道上应用掩膜，抑制非候选区域
    cv::Mat channels[3];
    cv::split(image, channels);
    cv::Mat greenMasked;
    channels[1].copyTo(greenMasked, mask);

    // 进行高斯平滑，减少噪声对峰值定位的影响
    cv::Mat blurred;
    int ksize = m_gaussKernelSize | 1;
    cv::GaussianBlur(greenMasked, blurred, cv::Size(ksize, ksize), 0);

    // 按行做峰值定位，并在局部窗口内计算灰度重心
    const int halfWindow = 5;
    laserPoints.reserve(blurred.rows);

    for (int row = 0; row < blurred.rows; row++) {
        const uchar* rowPtr = blurred.ptr<uchar>(row);

        int maxVal = 0;
        int maxCol = -1;
        for (int col = 0; col < blurred.cols; col++) {
            if (rowPtr[col] > maxVal) {
                maxVal = rowPtr[col];
                maxCol = col;
            }
        }

        // 峰值过低时认为该行无有效激光信号
        if (maxVal < m_threshold) continue;

        int colStart = std::max(0, maxCol - halfWindow);
        int colEnd   = std::min(blurred.cols - 1, maxCol + halfWindow);

        // 使用局部自适应阈值过滤弱响应像素
        int adaptiveThresh = std::max(m_threshold, maxVal / 3);

        double weightedSum = 0.0;
        double intensitySum = 0.0;

        for (int col = colStart; col <= colEnd; col++) {
            int val = rowPtr[col];
            if (val > adaptiveThresh) {
                weightedSum += col * (double)val;
                intensitySum += (double)val;
            }
        }

        // 信号强度满足条件时输出该行的亚像素中心
        if (intensitySum > adaptiveThresh * 3.0) {
            double subPixelU = weightedSum / intensitySum;
            laserPoints.push_back({subPixelU, static_cast<double>(row)});
        }
    }

    return laserPoints;
}

// 将提取到的中心点绘制到图像上用于调试显示
void LaserExtractor::drawCenters(cv::Mat& image,
                                 const std::vector<Point2D>& points,
                                 const cv::Scalar& color) {
    for (const auto& pt : points) {
        cv::circle(image,
                   cv::Point(static_cast<int>(pt.u), static_cast<int>(pt.v)),
                   1, color, -1);
    }
}
