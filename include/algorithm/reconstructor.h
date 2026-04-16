/**
 * @file reconstructor.h
 * @brief 线面交会重建模块接口
 *
 * 定义 2D 激光中心点到 3D 点云切片的核心类与参数接口。
 */

#ifndef RECONSTRUCTOR_H  // 头文件防重复包含宏开始
#define RECONSTRUCTOR_H  // 定义头文件防重复包含宏

#include <vector>           // std::vector 容器
#include <string>           // std::string
#include <Eigen/Dense>      // Eigen 矩阵向量类型
#include "algorithm/laser_extractor.h"  // Point2D 定义

/**
 * @brief 线面交会 3D 重建器
 *
 * 核心数学:
 *   射线: P_c = λ · K_inv · [u, v, 1]^T
 *   动态平面: π_C(θ) = (T_CB · T_BR(θ))^{-T} · π_R
 *   交会: λ = -d_c / (n_c^T · K_inv · p)
 *
 * 坐标系约定:
 * - C: 相机坐标系 (OpenCV: X→东/右, Y→下, Z→北/前)
 * - B: 电机基座坐标系 (ENU: X→东, Y→北, Z→天)
 * - R: 转子坐标系 (ENU, 与 B 同轴, 绕 Z/天轴 旋转; 激光平面静态定义所在坐标系)
 */
class Reconstructor {  // 从 2D 激光点重建 3D 点的算法类
public:  // 对外公开接口
    Reconstructor();  // 默认构造函数

    /**
     * @brief 从 config.yaml 加载标定参数
     * @return true 成功
     */
    bool loadParams(const std::string& configPath);  // 从配置文件读取 K/T_CB/pi_R

    /**
     * @brief 手动设置参数 (调试用)
     */
    void setParams(const Eigen::Matrix3d& K,     // 相机内参矩阵
                   const Eigen::Matrix4d& T_CB,  // 基座到相机变换
                   const Eigen::Vector4d& pi_R); // 转子系激光平面

    /**
     * @brief 处理一帧数据，输出 3D 点云切片
     * @param laserCenters 该帧提取到的 2D 激光中心点
     * @param theta_deg    步进电机当前绝对角度 (度)
     * @return 3D 点集合 (相机坐标系下)
     */
    std::vector<Eigen::Vector3d> processFrame(          // 核心重建入口
        const std::vector<Point2D>& laserCenters,       // 输入 2D 点序列
        double theta_deg);                               // 输入当前旋转角度

private:  // 私有数据成员
    Eigen::Matrix3d m_K;        // 相机内参矩阵 K
    Eigen::Matrix3d m_K_inv;    // K 的逆矩阵，初始化后缓存复用
    Eigen::Matrix4d m_T_CB;     // B->C 齐次变换矩阵
    Eigen::Vector4d m_pi_R;     // R 坐标系下光平面参数 [nx, ny, nz, d]

    bool m_paramsLoaded;        // 参数是否已成功加载标志
};

#endif // RECONSTRUCTOR_H  // 头文件结束
