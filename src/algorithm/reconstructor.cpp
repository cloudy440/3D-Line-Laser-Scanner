#include "algorithm/reconstructor.h"
#include <opencv2/core.hpp>
#include <iostream>
#include <cmath>

Reconstructor::Reconstructor() : m_paramsLoaded(false) {}

// 从配置文件加载内参、外参与激光平面参数
bool Reconstructor::loadParams(const std::string& configPath) {
    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[Reconstructor] Cannot open config: " << configPath << std::endl;
        return false;
    }

    // 读取并拷贝相机内参矩阵
    cv::Mat K_cv;
    fs["Camera_Matrix"] >> K_cv;
    if (K_cv.empty()) {
        std::cerr << "[Reconstructor] Camera_Matrix not found!" << std::endl;
        return false;
    }
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            m_K(r, c) = K_cv.at<double>(r, c);

    // 读取并拷贝相机到基座的齐次变换
    cv::Mat T_CB_cv;
    fs["T_CB"] >> T_CB_cv;
    if (T_CB_cv.empty()) {
        std::cerr << "[Reconstructor] T_CB not found!" << std::endl;
        return false;
    }
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            m_T_CB(r, c) = T_CB_cv.at<double>(r, c);

    // 读取并拷贝转子坐标系下的激光平面参数
    cv::Mat pi_R_cv;
    fs["LaserPlane_R"] >> pi_R_cv;
    if (pi_R_cv.empty()) {
        std::cerr << "[Reconstructor] LaserPlane_R not found!" << std::endl;
        return false;
    }
    for (int i = 0; i < 4; i++)
        m_pi_R(i) = pi_R_cv.at<double>(i, 0);

    // 预计算内参逆矩阵并标记参数就绪
    fs.release();

    m_K_inv = m_K.inverse();
    m_paramsLoaded = true;

    std::cout << "[Reconstructor] Parameters loaded." << std::endl;
    std::cout << "  K =\n" << m_K << std::endl;
    std::cout << "  T_CB =\n" << m_T_CB << std::endl;
    std::cout << "  pi_R = " << m_pi_R.transpose() << std::endl;

    return true;
}

// 供调试使用：直接设置重建所需参数
void Reconstructor::setParams(const Eigen::Matrix3d& K,
                              const Eigen::Matrix4d& T_CB,
                              const Eigen::Vector4d& pi_R) {
    m_K = K;
    m_K_inv = K.inverse();
    m_T_CB = T_CB;
    m_pi_R = pi_R;
    m_paramsLoaded = true;
}

std::vector<Eigen::Vector3d> Reconstructor::processFrame(
    const std::vector<Point2D>& laserCenters,
    double theta_deg) {

    // 参数未加载或输入为空时直接返回空结果
    std::vector<Eigen::Vector3d> points3D;
    if (!m_paramsLoaded || laserCenters.empty()) return points3D;

    constexpr double kPi = 3.14159265358979323846;
    double theta = theta_deg * kPi / 180.0;

    // 根据当前电机角度构造转子到基座的旋转变换
    Eigen::Matrix4d T_BR = Eigen::Matrix4d::Identity();
    T_BR(0, 0) =  std::cos(theta);
    T_BR(0, 1) = -std::sin(theta);
    T_BR(1, 0) =  std::sin(theta);
    T_BR(1, 1) =  std::cos(theta);

    Eigen::Matrix4d T_total = m_T_CB * T_BR;
    // 平面参数属于对偶空间，需使用逆转置进行坐标变换
    Eigen::Vector4d pi_C = T_total.inverse().transpose() * m_pi_R;

    Eigen::Vector3d n_c = pi_C.head<3>();
    double d_c = pi_C(3);

    points3D.reserve(laserCenters.size());

    // 对每个像素点做反投影并与激光平面求交
    for (const auto& pt : laserCenters) {
        Eigen::Vector3d p(pt.u, pt.v, 1.0);

        Eigen::Vector3d dir = m_K_inv * p;

        double denominator = n_c.dot(dir);

        // 跳过近平行射线，避免深度数值不稳定
        if (std::abs(denominator) < 1e-1) continue;

        double lambda = -d_c / denominator;

        if (lambda < 0) continue;

        Eigen::Vector3d P_c = lambda * dir;
        points3D.push_back(P_c);
    }

    return points3D;
}
