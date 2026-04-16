#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>

#include "hardware/serial_comm.h"
#include "hardware/camera_capture.h"
#include "algorithm/laser_extractor.h"
#include "algorithm/reconstructor.h"
#include "ui/viewer.h"

struct AppConfig {
    std::string configPath = "../config.yaml";
    bool simulate = false;
};

// 解析命令行参数，决定配置路径与是否进入模拟模式
AppConfig parseArgs(int argc, char* argv[]) {
    AppConfig cfg;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--simulate") {
            cfg.simulate = true;
        } else if (arg == "--config" && i + 1 < argc) {
            cfg.configPath = argv[++i];
        }
    }
    return cfg;
}

struct ScanParams {
    std::string serialPort = "COM10";
    int serialBaud = 115200;
    int cameraIndex = 1;
    double stepAngle = 0.1125;
    double halfAngle = 60.0;
};

// 从配置文件读取扫描参数，缺失时使用默认值
ScanParams loadScanParams(const std::string& configPath) {
    ScanParams sp;
    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[Main] Warning: cannot open " << configPath
                  << ", using defaults." << std::endl;
        return sp;
    }
    if (!fs["Serial_Port"].empty())       fs["Serial_Port"] >> sp.serialPort;
    if (!fs["Serial_BaudRate"].empty())   fs["Serial_BaudRate"] >> sp.serialBaud;
    if (!fs["Camera_Index"].empty())      fs["Camera_Index"] >> sp.cameraIndex;
    if (!fs["StepAngle_Deg"].empty())     fs["StepAngle_Deg"] >> sp.stepAngle;
    if (!fs["Scan_Half_Angle"].empty())   fs["Scan_Half_Angle"] >> sp.halfAngle;
    fs.release();
    return sp;
}

// 程序入口：初始化模块并执行整段扫描流程
int main(int argc, char* argv[]) {
    std::cout << "=====================================" << std::endl;
    std::cout << "  LineLaserScanner3D  " << std::endl;
    std::cout << "=====================================" << std::endl;

    AppConfig appCfg = parseArgs(argc, argv);
    if (appCfg.simulate) {
        std::cout << "[Mode] SIMULATE - no serial connection" << std::endl;
    } else {
        std::cout << "[Mode] HARDWARE - connecting to STM32" << std::endl;
    }

    ScanParams sp = loadScanParams(appCfg.configPath);

    // 计算对称扫描角范围 [theta, 180 - theta]
    double theta = sp.halfAngle;
    double startAngle = theta;
    double endAngle   = 180.0 - theta;
    double scanRange  = endAngle - startAngle;

    std::cout << "[Config] Step angle: " << sp.stepAngle << " deg" << std::endl;
    std::cout << "[Config] Half angle (theta): " << theta << " deg" << std::endl;
    std::cout << "[Config] Scan range: " << startAngle << " -> " << endAngle << " deg" << std::endl;

    // 初始化串口通信（仅硬件模式需要）
    SerialComm serial;
    if (!appCfg.simulate) {
        if (!serial.init(sp.serialPort, sp.serialBaud)) {
            std::cerr << "[Main] Serial init failed! Use --simulate for testing." << std::endl;
            return -1;
        }
    }

    // 初始化相机、激光中心提取器、重建器与可视化器
    CameraCapture camera;
    if (!camera.init(sp.cameraIndex, appCfg.configPath)) {
        std::cerr << "[Main] Camera init failed!" << std::endl;
        return -1;
    }

    LaserExtractor extractor(8, 5);

    Reconstructor reconstructor;
    if (!reconstructor.loadParams(appCfg.configPath)) {
        std::cerr << "[Main] Reconstructor param load failed!" << std::endl;
        return -1;
    }

    Viewer viewer;
    viewer.init();

    // 先移动到扫描起点，并累计位移用于结束后复位
    int stepsToStart = static_cast<int>(std::round(theta / sp.stepAngle));
    int totalMoved = 0;

    if (!appCfg.simulate && stepsToStart > 0) {
        std::cout << "[Main] Moving to start position: " << theta << " deg ("
                  << stepsToStart << " steps CCW)..." << std::endl;
        try {
            serial.sendMoveCommand(stepsToStart);
            totalMoved += stepsToStart;
        } catch (const std::exception& e) {
            std::cerr << "[Main] Move to start failed: " << e.what() << std::endl;
            return -1;
        }
    }

    // 扫描主循环：步进、采图、提取、重建、显示
    double currentAngle = startAngle;
    int scanSteps = static_cast<int>(std::round(scanRange / sp.stepAngle));
    int currentStep = 0;

    std::cout << "\n========== SCANNING START ==========" << std::endl;
    std::cout << "Total scan steps: " << scanSteps << std::endl;

    while (currentStep < scanSteps) {
        if (!appCfg.simulate) {
            try {
                currentAngle = serial.moveStepAndWait();
                totalMoved++;
            } catch (const std::exception& e) {
                std::cerr << "[Main] Step failed: " << e.what() << std::endl;
                break;
            }
        } else {
            currentAngle += sp.stepAngle;
        }

        cv::Mat frame = camera.getLatestClearFrame(3);
        if (frame.empty()) {
            std::cerr << "[Main] Empty frame, skipping..." << std::endl;
            continue;
        }

        std::vector<Point2D> points2D = extractor.extractCenter(frame);
        std::vector<Eigen::Vector3d> points3D =
            reconstructor.processFrame(points2D, currentAngle);

        viewer.update(frame, points2D, points3D);

        currentStep++;
        if (currentStep % 10 == 0 || currentStep == scanSteps) {
            int progress = static_cast<int>(100.0 * currentStep / scanSteps);
            std::cout << "\r[Progress] " << progress << "% ("
                      << currentStep << "/" << scanSteps << ") "
                      << "Angle: " << currentAngle << " deg, "
                      << "Points this frame: " << points3D.size()
                      << std::flush;
        }

        if (viewer.isStopped()) {
            std::cout << "\n[Main] 3D viewer closed, stopping scan." << std::endl;
            break;
        }
    }

    std::cout << "\n========== SCANNING COMPLETE ==========" << std::endl;

    // 根据累计步数反向回零，恢复到初始姿态
    if (!appCfg.simulate && totalMoved > 0) {
        std::cout << "[Main] Returning to 0 deg (" << totalMoved << " steps CW)..." << std::endl;
        try {
            serial.sendMoveCommand(-totalMoved);
        } catch (const std::exception& e) {
            std::cerr << "[Main] Return to 0 failed: " << e.what() << std::endl;
        }
    }

    // 保存点云并等待用户关闭窗口退出
    std::string outputFile = "scan_result.pcd";
    viewer.saveCloud(outputFile);

    std::cout << "\nPress any key in the 2D window to exit..." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();

    std::cout << "[Main] Program exited safely." << std::endl;
    return 0;
}