#include "hardware/camera_capture.h"
#include <iostream>

CameraCapture::CameraCapture()
    : m_calibLoaded(false) {}

// 析构时释放摄像头句柄
CameraCapture::~CameraCapture() {
    if (m_cap.isOpened()) {
        m_cap.release();
    }
}

// 初始化摄像头参数，并尝试加载标定数据
bool CameraCapture::init(int cameraIndex, const std::string& configPath) {
    m_cap.open(cameraIndex);
    if (!m_cap.isOpened()) {
        std::cerr << "[CameraCapture] Failed to open camera index "
                  << cameraIndex << std::endl;
        return false;
    }

    // 配置分辨率与曝光相关参数
    m_cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    m_cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    m_cap.set(cv::CAP_PROP_EXPOSURE, -6);
    m_cap.set(cv::CAP_PROP_AUTO_WB, 0);

    std::cout << "[CameraCapture] Camera opened (index=" << cameraIndex << ")"
              << std::endl;

    // 标定数据加载失败时允许继续采集原图
    if (!configPath.empty()) {
        m_calibLoaded = loadCalibration(configPath);
        if (!m_calibLoaded) {
            std::cerr << "[CameraCapture] Warning: calibration not loaded, "
                      << "undistortion will be skipped." << std::endl;
        }
    }

    return true;
}

// 从配置文件读取相机内参与畸变系数
bool CameraCapture::loadCalibration(const std::string& configPath) {
    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[CameraCapture] Cannot open config: " << configPath << std::endl;
        return false;
    }

    fs["Camera_Matrix"] >> m_K;
    fs["Distortion_Coeffs"] >> m_distCoeffs;
    fs.release();

    if (m_K.empty() || m_distCoeffs.empty()) {
        std::cerr << "[CameraCapture] Calibration data incomplete!" << std::endl;
        return false;
    }

    std::cout << "[CameraCapture] Calibration loaded." << std::endl;
    std::cout << "  K = " << m_K << std::endl;
    return true;
}

// 获取最新帧：先丢弃缓存帧，再读取并按需去畸变
cv::Mat CameraCapture::getLatestClearFrame(int discardFrames) {
    for (int i = 0; i < discardFrames; i++) {
        m_cap.grab();
    }

    cv::Mat raw;
    if (!m_cap.read(raw) || raw.empty()) {
        std::cerr << "[CameraCapture] Failed to read frame!" << std::endl;
        return cv::Mat();
    }

    if (m_calibLoaded) {
        cv::Mat undistorted;
        cv::undistort(raw, undistorted, m_K, m_distCoeffs);
        return undistorted;
    }

    return raw;
}

// 直接读取原始帧，不做去畸变处理
cv::Mat CameraCapture::getRawFrame() {
    cv::Mat frame;
    m_cap.read(frame);
    return frame;
}

// 查询摄像头是否处于打开状态
bool CameraCapture::isOpened() const {
    return m_cap.isOpened();
}
