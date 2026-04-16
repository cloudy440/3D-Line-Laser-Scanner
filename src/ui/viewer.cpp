#include "ui/viewer.h"
#include <iostream>

#if LL3D_HAS_PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#endif

Viewer::Viewer()
#if LL3D_HAS_PCL
    : m_cloudInitialized(false)
#endif
{}

Viewer::~Viewer() {}

// 初始化 2D/3D 可视化窗口与点云容器
void Viewer::init() {
    cv::namedWindow("2D Laser Tracking", cv::WINDOW_AUTOSIZE);

#if LL3D_HAS_PCL
    m_pclViewer.reset(new pcl::visualization::PCLVisualizer("3D Point Cloud Live"));
    m_pclViewer->setBackgroundColor(0.05, 0.05, 0.1);
    m_pclViewer->addCoordinateSystem(50.0);
    m_pclViewer->initCameraParameters();

    m_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudInitialized = false;

    std::cout << "[Viewer] 2D + 3D windows initialized." << std::endl;
#else
    std::cout << "[Viewer] 2D window initialized. (PCL not available, 3D disabled)"
              << std::endl;
#endif
}

// 更新 2D 跟踪画面，并在可用时刷新 3D 点云显示
void Viewer::update(cv::Mat& frame,
                    const std::vector<Point2D>& points2D,
                    const std::vector<Eigen::Vector3d>& points3D) {
    LaserExtractor::drawCenters(frame, points2D);
    cv::imshow("2D Laser Tracking", frame);
    cv::waitKey(1);

#if LL3D_HAS_PCL
    for (const auto& pt : points3D) {
        pcl::PointXYZ pclPt;
        pclPt.x = static_cast<float>(pt.x());
        pclPt.y = static_cast<float>(pt.y());
        pclPt.z = static_cast<float>(pt.z());
        m_cloud->push_back(pclPt);
    }

    // 首次添加点云对象，后续仅更新数据
    if (!m_cloudInitialized && !m_cloud->empty()) {
        m_pclViewer->addPointCloud<pcl::PointXYZ>(m_cloud, "scan_cloud");
        m_pclViewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scan_cloud");
        m_cloudInitialized = true;
    } else if (m_cloudInitialized) {
        m_pclViewer->updatePointCloud(m_cloud, "scan_cloud");
    }

    m_pclViewer->spinOnce(10);
#endif
}

// 按扩展名将累计点云保存为 PCD 或 PLY
bool Viewer::saveCloud(const std::string& filename) {
#if LL3D_HAS_PCL
    if (!m_cloud || m_cloud->empty()) {
        std::cerr << "[Viewer] No point cloud data to save!" << std::endl;
        return false;
    }

    int result;
    if (filename.size() >= 4 &&
        filename.substr(filename.size() - 4) == ".ply") {
        result = pcl::io::savePLYFileBinary(filename, *m_cloud);
    } else {
        result = pcl::io::savePCDFileBinary(filename, *m_cloud);
    }

    if (result == 0) {
        std::cout << "[Viewer] Point cloud saved to: " << filename
                  << " (" << m_cloud->size() << " points)" << std::endl;
        return true;
    } else {
        std::cerr << "[Viewer] Failed to save point cloud!" << std::endl;
        return false;
    }
#else
    std::cerr << "[Viewer] Cannot save: PCL not available." << std::endl;
    return false;
#endif
}

// 查询 3D 窗口是否已被用户关闭
bool Viewer::isStopped() const {
#if LL3D_HAS_PCL
    return m_pclViewer && m_pclViewer->wasStopped();
#else
    return false;
#endif
}
