/**
 * @file viewer.h
 * @brief 2D/3D 可视化模块接口
 *
 * 定义显示初始化、实时更新、点云保存和窗口状态查询接口。
 */

#ifndef VIEWER_H  // 头文件防重复包含宏开始
#define VIEWER_H  // 定义头文件防重复包含宏

#include <vector>             // std::vector
#include <string>             // std::string
#include <opencv2/opencv.hpp> // OpenCV 图像窗口接口
#include <Eigen/Dense>        // Eigen 向量类型
#include "algorithm/laser_extractor.h"  // Point2D 定义

#if LL3D_HAS_PCL  // 仅在启用 PCL 时包含 3D 可视化头
#include <pcl/point_cloud.h>                     // PCL 点云容器
#include <pcl/point_types.h>                     // PCL 点类型
#include <pcl/visualization/pcl_visualizer.h>    // PCL 可视化窗口
#endif  // LL3D_HAS_PCL 条件结束

/**
 * @brief 双视窗渲染器
 *
 * - 2D 窗口 (OpenCV): 实时摄像头画面 + 激光中心标记
 * - 3D 窗口 (PCL):    实时点云生长渲染 (条件编译)
 */
class Viewer {  // 负责二维与三维显示
public:  // 公共接口区
    Viewer();   // 构造函数
    ~Viewer();  // 析构函数

    /**
     * @brief 初始化显示窗口
     */
    void init();  // 初始化窗口与点云容器

    /**
     * @brief 更新显示内容
     * @param frame     当前帧图像 (将在上面绘制标记)
     * @param points2D  当前帧提取到的 2D 激光中心
     * @param points3D  当前帧计算出的 3D 点
     */
    void update(cv::Mat& frame,                            // 输入输出图像帧
                const std::vector<Point2D>& points2D,     // 当前帧二维点
                const std::vector<Eigen::Vector3d>& points3D);  // 当前帧三维点

    /**
     * @brief 保存累积的点云到文件
     * @param filename 输出文件名 (如 "scan_result.pcd")
     * @return true 成功
     */
    bool saveCloud(const std::string& filename);  // 保存累积点云

    /**
     * @brief 检查 3D 视窗是否被关闭
     */
    bool isStopped() const;  // 返回 3D 窗口关闭状态

private:  // 私有成员区
#if LL3D_HAS_PCL  // 仅在 PCL 可用时定义 3D 数据成员
    pcl::visualization::PCLVisualizer::Ptr m_pclViewer;  // 3D 可视化窗口实例
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;         // 累积点云缓冲
    bool m_cloudInitialized;                             // 是否已向窗口注册点云对象
#endif  // LL3D_HAS_PCL 条件结束
};

#endif // VIEWER_H  // 头文件结束
