/**
 * @file camera_capture.h
 * @brief 摄像头采集与去畸变模块接口
 *
 * 定义摄像头初始化、原始取帧与去畸变取帧能力。
 */

#ifndef CAMERA_CAPTURE_H  // 头文件防重复包含宏开始
#define CAMERA_CAPTURE_H  // 定义头文件防重复包含宏

#include <string>            // std::string
#include <opencv2/opencv.hpp>  // OpenCV 视频与矩阵接口

/**
 * @brief USB 摄像头控制类
 *
 * 封装 OpenCV VideoCapture，提供:
 * - 关闭自动曝光
 * - 清空 USB 缓冲区
 * - 图像去畸变
 *
 * 使用约束:
 * - 本类按单线程访问设计，不保证多线程并发安全
 * - 去畸变依赖 config.yaml 中的 Camera_Matrix/Distortion_Coeffs
 */
class CameraCapture {  // 摄像头采集与去畸变封装
public:  // 公共接口区
    CameraCapture();   // 构造函数
    ~CameraCapture();  // 析构函数

    /**
     * @brief 初始化摄像头
     * @param cameraIndex 摄像头索引 (如笔记本自带=0, USB外接=1或2)
     * @param configPath  config.yaml 路径，用于加载内参与畸变系数
     * @return true 成功
     */
    bool init(int cameraIndex, const std::string& configPath);  // 初始化硬件与标定参数

    /**
     * @brief 获取去畸变的最新清晰帧 (清空 USB 缓冲后)
     * @param discardFrames 丢弃的缓冲帧数 (默认3)
     * @return 去畸变后的图像; 如果失败返回空 Mat
     *
     * 说明:
     * - 会先丢弃若干旧帧，减少 USB 摄像头缓存延迟
     * - 若标定未加载，会直接返回原始帧
     */
    cv::Mat getLatestClearFrame(int discardFrames = 3);  // 获取最新且尽量低延迟的帧

    /**
     * @brief 获取原始帧 (标定拍照时使用，不做去畸变)
     */
    cv::Mat getRawFrame();  // 获取原始图像

    /**
     * @brief 检查摄像头是否已打开
     */
    bool isOpened() const;  // 返回摄像头打开状态

    /**
     * @brief 获取内参矩阵 (供其他模块使用)
     */
    const cv::Mat& getCameraMatrix() const { return m_K; }  // 返回内参矩阵引用

    /**
     * @brief 获取畸变系数
     */
    const cv::Mat& getDistCoeffs() const { return m_distCoeffs; }  // 返回畸变系数引用

private:  // 私有成员区
    cv::VideoCapture m_cap;  // OpenCV 摄像头句柄
    cv::Mat m_K;             // 3x3 内参矩阵
    cv::Mat m_distCoeffs;    // 畸变系数
    bool m_calibLoaded;      // 标定数据是否已加载

    /**
     * @brief 从 YAML 加载内参和畸变系数
     */
    bool loadCalibration(const std::string& configPath);  // 读取配置文件中的标定参数
};

#endif // CAMERA_CAPTURE_H  // 头文件结束
