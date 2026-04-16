/**
 * @file laser_extractor.h
 * @brief 激光条纹中心提取模块接口
 *
 * 提供:
 * - Point2D 二维点定义
 * - LaserExtractor 提取接口
 * - 调试绘制接口
 */

#ifndef LASER_EXTRACTOR_H  // 头文件防重复包含宏开始
#define LASER_EXTRACTOR_H  // 定义头文件防重复包含宏

#include <vector>           // std::vector 容器
#include <opencv2/opencv.hpp>  // OpenCV 基础类型与图像接口

/**
 * @brief 2D 像素点 (亚像素精度)
 */
struct Point2D {  // 激光中心在图像中的二维坐标
    double u;     // 列坐标，单位像素，原点在图像左上角
    double v;     // 行坐标，单位像素，支持亚像素小数
};

/**
 * @brief 激光条纹中心提取类
 *
 * 当前实现: 灰度重心法 (Gravity Center Method)
 * 针对 532nm 绿色激光优化：提取绿色通道
 * 当前采样策略为“逐行重心”，即每行最多输出一个中心点
 *
 * 后续可替换为 Steger 算法，只需修改 extractCenter() 内部实现
 */
class LaserExtractor {  // 激光中心提取模块
public:  // 对外公开接口
    /**
     * @param threshold 灰度阈值，低于此值的像素不参与计算 (默认30)
     * @param gaussKernelSize 高斯滤波核大小 (默认5)
     */
    LaserExtractor(int threshold = 30, int gaussKernelSize = 5);  // 构造函数并设置参数

    /**
     * @brief 从图像中提取激光条纹的亚像素中心坐标
     * @param image 输入图像 (BGR 彩色, 已去畸变)
     * @return 提取到的 2D 中心点集合
     *
     * 说明:
     * - 当前实现采用“逐行重心”策略，即每一行最多输出一个中心点
     * - 当激光线在图像中近竖直时，逐行策略通常比逐列策略更稳定
     */
    std::vector<Point2D> extractCenter(const cv::Mat& image);  // 提取当前帧激光中心点

    /**
     * @brief 在图像上绘制提取到的激光中心 (调试用)
     * @param image 输入/输出图像
     * @param points 提取到的中心点
     * @param color  标记颜色 (默认绿色)
     */
    static void drawCenters(cv::Mat& image, const std::vector<Point2D>& points,  // 在图像上画点
                            const cv::Scalar& color = cv::Scalar(0, 255, 0));    // 设定绘制颜色

private:  // 私有成员
    int m_threshold;        // 灰度阈值，抑制背景噪声
    int m_gaussKernelSize;  // 高斯核尺寸，必须为奇数
};

#endif // LASER_EXTRACTOR_H  // 头文件结束
