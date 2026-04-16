/**
 * @file calibrator_main.cpp
 * @brief LineLaserScanner3D 离线标定工具
 *
 * 模式 A (capture):   拍摄棋盘格图片 (内参标定用)
 * 模式 B (calibrate): 计算内参和畸变系数 -> 输出 config.yaml
 * 模式 C (excapture): 拍摄带激光线的棋盘格图片 (外参标定用)
 * 模式 D (excalib):   计算外参 T_CB 和光平面 π_R -> 更新 config.yaml
 *
 * 用法:
 *   CalibratorTool capture   --camera 1 --outdir ./calib_images
 *   CalibratorTool calibrate --imgdir ./calib_images --board 9x6 --square 25.0
 *   CalibratorTool excapture --camera 1 --outdir ./excalib_images
 *   CalibratorTool excalib   --imgdir ./excalib_images --board 9x6
 * --square 25.0 --dist 150
 */

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "algorithm/laser_extractor.h"
#include "hardware/serial_comm.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

// ============================================================
// 模式 A: 拍摄标定图片
// ============================================================
int captureImages(int cameraIndex,
                  const std::string &outDir) { // 拍摄并保存标定图像
  fs::create_directories(outDir);              // 若目录不存在则递归创建

  cv::VideoCapture cap(cameraIndex); // 打开指定序号摄像头
  if (!cap.isOpened()) {             // 检查摄像头是否成功打开
    std::cerr << "[Capture] Cannot open camera " << cameraIndex
              << std::endl; // 输出错误信息
    return -1;              // 打开失败返回错误码
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);  // 期望宽度 640
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // 期望高度 480

  std::cout << "[Capture] Camera opened. Controls:"
            << std::endl;                                   // 提示控制方式
  std::cout << "  SPACE = save current frame" << std::endl; // 空格保存
  std::cout << "  Q/ESC = finish and exit" << std::endl;    // Q 或 ESC 退出
  std::cout << "  Aim for 15-20 images from different angles."
            << std::endl; // 建议拍摄数量

  int count = 0; // 已保存图像计数
  cv::Mat frame; // 当前帧缓存

  while (true) {  // 主采集循环
    cap >> frame; // 读取一帧图像
    if (frame.empty())
      break; // 读取失败或流结束则退出循环

    std::string text = "Saved: " + std::to_string(count); // 叠加显示已保存数量
    cv::putText(frame, text, cv::Point(10, 30), // 在左上角绘制计数文本
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0),
                2); // 设置字体样式

    cv::imshow("Calibration Capture", frame);      // 显示实时画面
    char key = static_cast<char>(cv::waitKey(30)); // 读取键盘输入(约 30ms 刷新)

    if (key == ' ') {                             // 空格键: 保存当前帧
      std::string filename = outDir + "/calib_" + // 构造输出文件名前缀
                             std::to_string(count) + ".png"; // 拼接编号与后缀
      cv::imwrite(filename, frame);                          // 保存图像到磁盘
      std::cout << "  Saved: " << filename << std::endl;     // 打印保存路径
      count++;                                               // 计数递增
    } else if (key == 'q' || key == 27) { // q 或 ESC: 退出采集
      break;                              // 跳出循环
    }
  }

  cap.release();           // 释放摄像头资源
  cv::destroyAllWindows(); // 关闭全部 OpenCV 窗口
  std::cout << "[Capture] Total " << count << " images saved to " << outDir
            << std::endl; // 输出采集统计
  return 0;               // 正常结束
}

// ============================================================
// 模式 B: 执行标定
// ============================================================
int runCalibration(const std::string &imgDir,       // 标定图目录
                   int boardW, int boardH,          // 棋盘格内角点宽高
                   float squareSize,                // 单格物理尺寸(mm)
                   const std::string &outputYaml) { // 输出配置文件
  cv::Size boardSize(boardW, boardH);               // OpenCV 棋盘格尺寸对象

  std::vector<std::string> imageFiles; // 存放所有候选图像路径
  for (const auto &entry : fs::directory_iterator(imgDir)) { // 遍历目录项
    std::string ext = entry.path().extension().string();     // 读取文件扩展名
    if (ext == ".png" || ext == ".jpg" || ext == ".bmp") { // 仅处理常见图像格式
      imageFiles.push_back(entry.path().string());         // 记录图像完整路径
    }
  }

  if (imageFiles.empty()) { // 没有找到图像则报错
    std::cerr << "[Calibrate] No images found in " << imgDir
              << std::endl; // 输出错误信息
    return -1;              // 返回失败
  }
  std::cout << "[Calibrate] Found " << imageFiles.size() << " images."
            << std::endl; // 输出图像数量

  std::vector<cv::Point3f> objCorners; // 单张图对应的棋盘角点世界坐标模板
  for (int r = 0; r < boardH; r++) {   // 遍历棋盘行
    for (int c = 0; c < boardW; c++) { // 遍历棋盘列
      objCorners.emplace_back(c * squareSize, r * squareSize,
                              0.0f); // Z=0 平面上生成角点
    }
  }

  std::vector<std::vector<cv::Point3f>> objectPoints; // 所有图像的 3D 角点
  std::vector<std::vector<cv::Point2f>> imagePoints;  // 所有图像的 2D 角点
  cv::Size imageSize;                                 // 标定图尺寸

  for (const auto &imgFile : imageFiles) { // 遍历每张标定图
    cv::Mat img = cv::imread(imgFile);     // 读取图像
    if (img.empty())
      continue;             // 读取失败则跳过
    imageSize = img.size(); // 记录图像尺寸

    cv::Mat gray;                                // 灰度图缓存
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY); // 转灰度用于角点检测

    std::vector<cv::Point2f> corners; // 当前图像检测到的角点
    bool found = cv::findChessboardCorners(
        gray, boardSize, corners, // 检测棋盘角点
        cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE); // 启用常用增强选项

    if (found) { // 成功检测到角点
      cv::cornerSubPix(
          gray, corners, cv::Size(11, 11),
          cv::Size(-1, -1), // 亚像素优化角点位置
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                           30, 0.001)); // 最大迭代 30，精度阈值 0.001

      objectPoints.push_back(objCorners); // 追加该图对应 3D 角点模板
      imagePoints.push_back(corners);     // 追加该图对应 2D 角点

      cv::drawChessboardCorners(img, boardSize, corners,
                                found);    // 可视化绘制检测结果
      cv::imshow("Detected Corners", img); // 显示当前检测图
      cv::waitKey(200);                    // 短暂停留便于观察

      std::cout << "  [OK] " << imgFile << std::endl; // 打印成功图像
    } else {                                          // 未检测到角点
      std::cout << "  [SKIP] " << imgFile << " (corners not found)"
                << std::endl; // 打印跳过信息
    }
  }
  cv::destroyAllWindows(); // 关闭角点预览窗口

  if (objectPoints.size() < 5) { // 有效图像过少时拒绝标定
    std::cerr << "[Calibrate] Not enough valid images! Need at least 5, got "
              << objectPoints.size() << std::endl; // 输出当前有效数量
    return -1;                                     // 返回失败
  }

  cv::Mat K, distCoeffs;             // 相机内参矩阵与畸变系数
  std::vector<cv::Mat> rvecs, tvecs; // 每张图的位姿结果(旋转/平移)

  double rmsError = cv::calibrateCamera( // 调用 OpenCV 相机标定
      objectPoints, imagePoints, imageSize, K, distCoeffs, rvecs, tvecs);

  std::cout << "\n=====================================" << std::endl; // 分隔线
  std::cout << "  Calibration Results" << std::endl;                   // 标题
  std::cout << "=====================================" << std::endl;   // 分隔线
  std::cout << "RMS Reprojection Error: " << rmsError
            << std::endl;                                // 输出重投影误差
  std::cout << "Camera Matrix (K):\n" << K << std::endl; // 输出内参矩阵
  std::cout << "Distortion Coefficients:\n"
            << distCoeffs << std::endl; // 输出畸变参数

  if (rmsError > 1.0) { // 经验阈值: RMS > 1 像素通常偏大
    std::cerr << "\n[WARNING] RMS error > 1.0 pixel! "
              << "Consider re-taking calibration images."
              << std::endl; // 给出重拍建议
  }

  cv::FileStorage fsOut(outputYaml,
                        cv::FileStorage::WRITE); // 以写模式打开 YAML
  if (!fsOut.isOpened()) {                       // 打开失败
    std::cerr << "[Calibrate] Cannot write to " << outputYaml
              << std::endl; // 输出错误
    return -1;              // 返回失败
  }

  fsOut << "Camera_Matrix" << K;              // 写入相机内参矩阵
  fsOut << "Distortion_Coeffs" << distCoeffs; // 写入畸变参数

  cv::Mat T_CB = cv::Mat::eye(4, 4, CV_64F); // 初始化占位外参为单位阵
  T_CB.at<double>(0, 3) = 100.0;             // 示例平移(单位与系统约定一致)
  fsOut << "T_CB" << T_CB;                   // 写入外参占位值

  cv::Mat pi_R =
      (cv::Mat_<double>(4, 1) << 1.0, 0.0, 0.0, -50.0); // 写入占位激光平面参数
  fsOut << "LaserPlane_R" << pi_R;                      // 保存激光平面

  fsOut << "StepAngle_Deg" << 0.1125;   // 步进角度
  fsOut << "Scan_Start_Angle" << 0.0;   // 扫描起始角
  fsOut << "Scan_End_Angle" << 90.0;    // 扫描结束角
  fsOut << "Serial_Port" << "COM3";     // 默认串口
  fsOut << "Serial_BaudRate" << 115200; // 默认波特率
  fsOut << "Camera_Index" << 2;         // 默认摄像头序号

  fsOut.release(); // 关闭并落盘 YAML 文件

  std::cout << "\n[Calibrate] Config saved to: " << outputYaml
            << std::endl; // 输出保存路径
  std::cout << "[Calibrate] Next step: Run extrinsic calibration to update "
               "T_CB and LaserPlane_R."
            << std::endl; // 提示后续步骤

  return 0; // 标定成功
}

// ============================================================
// 模式 C: 拍摄外参标定图片
//          启动时自动转到 90°(正北), 拍照完成后自动复位到 0°
// ============================================================
int captureExtrinsicImages(int cameraIndex, const std::string &outDir,
                           const std::string &port, int baud) {
  fs::create_directories(outDir);

  // ---- 连接串口并移动电机到 90° ----
  SerialComm serial;
  bool hasSerial = false;
  if (!port.empty()) {
    if (serial.init(port, baud)) {
      hasSerial = true;
      // 3200 steps/rev, 90° = 800 steps CCW
      int stepsTo90 = 800;
      std::cout << "[ExCapture] Moving laser to 90 deg (North)..." << std::endl;
      try {
        serial.sendMoveCommand(stepsTo90);
        std::cout << "[ExCapture] Laser at 90 deg. Ready for capture."
                  << std::endl;
      } catch (const std::exception &e) {
        std::cerr << "[ExCapture] Move failed: " << e.what() << std::endl;
        return -1;
      }
    } else {
      std::cerr << "[ExCapture] Serial init failed, continuing without motor."
                << std::endl;
    }
  } else {
    std::cout << "[ExCapture] No --port specified, skipping motor control."
              << std::endl;
  }

  cv::VideoCapture cap(cameraIndex);
  if (!cap.isOpened()) {
    std::cerr << "[ExCapture] Cannot open camera " << cameraIndex << std::endl;
    return -1;
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  // 曝光控制: 关闭自动曝光, 手动设较低值以便激光线更突出
  // 值的含义因摄像头型号而异: 一般负数或小值 = 低曝光
  // 如效果不好, 可调整 -6 为其他值 (范围大约 -13 ~ 0)
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE,
          0.25);                      // 0.25 = 手动模式 (MSMF/DirectShow)
  cap.set(cv::CAP_PROP_EXPOSURE, -4); // 曝光值, 越小越暗，范围约 -13 ~ 0

  std::cout << "\n[ExCapture] === Extrinsic Calibration Capture ==="
            << std::endl;
  std::cout << "  Laser should be pointing NORTH (90 deg)." << std::endl;
  std::cout << "  Place chessboard TILTED in laser path." << std::endl;
  std::cout << "  Controls: SPACE = save | Q/ESC = finish" << std::endl;
  std::cout << "  Take 8-15 images at DIFFERENT board positions/angles.\n"
            << std::endl;

  int count = 0;
  cv::Mat frame;
  while (true) {
    cap >> frame;
    if (frame.empty())
      break;

    std::string text = "ExCalib Saved: " + std::to_string(count);
    cv::putText(frame, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
    cv::imshow("Extrinsic Capture (move board, keep motor fixed)", frame);
    char key = static_cast<char>(cv::waitKey(30));

    if (key == ' ') {
      std::string filename =
          outDir + "/excalib_" + std::to_string(count) + ".png";
      cv::imwrite(filename, frame);
      std::cout << "  Saved: " << filename << std::endl;
      count++;
    } else if (key == 'q' || key == 27) {
      break;
    }
  }
  cap.release();
  cv::destroyAllWindows();
  std::cout << "[ExCapture] Total " << count << " images saved to " << outDir
            << std::endl;

  // ---- 复位电机回 0° ----
  if (hasSerial) {
    std::cout << "[ExCapture] Returning laser to 0 deg..." << std::endl;
    try {
      serial.sendMoveCommand(-800); // 800 steps CW 复位
      std::cout << "[ExCapture] Laser back at 0 deg." << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "[ExCapture] Return failed: " << e.what() << std::endl;
    }
  }

  return 0;
}

// ============================================================
// 模式 D: 外参标定 (棋盘格+激光线 -> 解算 T_CB 和 π_R)
// ============================================================
int runExtrinsicCalibration(const std::string &imgDir, int boardW, int boardH,
                            float squareSize, const std::string &configPath,
                            double camToMotorDist_mm) {
  cv::Size boardSize(boardW, boardH);

  // ----- 1. 加载已有内参 -----
  cv::Mat K, distCoeffs;
  {
    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "[ExCalib] Cannot open " << configPath << std::endl;
      return -1;
    }
    fs["Camera_Matrix"] >> K;
    fs["Distortion_Coeffs"] >> distCoeffs;
    fs.release();
    if (K.empty()) {
      std::cerr << "[ExCalib] Camera_Matrix not found! Run 'calibrate' first."
                << std::endl;
      return -1;
    }
  }
  cv::Mat K_inv = K.inv();
  std::cout << "[ExCalib] Loaded intrinsics: K =\n" << K << std::endl;

  // ----- 2. 收集图像 -----
  std::vector<std::string> imageFiles;
  for (const auto &entry : fs::directory_iterator(imgDir)) {
    std::string ext = entry.path().extension().string();
    if (ext == ".png" || ext == ".jpg" || ext == ".bmp")
      imageFiles.push_back(entry.path().string());
  }
  if (imageFiles.empty()) {
    std::cerr << "[ExCalib] No images found in " << imgDir << std::endl;
    return -1;
  }
  std::cout << "[ExCalib] Found " << imageFiles.size() << " images."
            << std::endl;

  // ----- 3. 棋盘格世界坐标 -----
  std::vector<cv::Point3f> objCorners;
  for (int r = 0; r < boardH; r++)
    for (int c = 0; c < boardW; c++)
      objCorners.emplace_back(c * squareSize, r * squareSize, 0.0f);

  // ----- 4. 逐图处理: solvePnP + 手动标注激光线 + 反投影到棋盘格面 -----
  std::vector<Eigen::Vector3d> allLaserPts3D;
  int validImages = 0;

  // 鼠标回调: 收集用户点击的两个端点
  struct ClickData {
    std::vector<cv::Point2d> pts;   // 已点击的点 (最多 2 个)
    cv::Mat displayImg;             // 用于叠加绘制的图像副本
    cv::Mat baseImg;                // 原始底图 (棋盘格标注后)
  };

  for (const auto &imgFile : imageFiles) {
    cv::Mat img = cv::imread(imgFile);
    if (img.empty())
      continue;

    // 去畸变
    cv::Mat undist;
    cv::undistort(img, undist, K, distCoeffs);

    // 检测棋盘格角点
    cv::Mat gray;
    cv::cvtColor(undist, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, boardSize, corners,
                                           cv::CALIB_CB_ADAPTIVE_THRESH |
                                               cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!found) {
      std::cout << "  [SKIP] " << imgFile << " (no corners)" << std::endl;
      continue;
    }
    cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30,
                         0.001));

    // solvePnP 得到棋盘格位姿 (去畸变图无需再传畸变参数)
    cv::Mat rvec, tvec;
    cv::solvePnP(objCorners, corners, K, cv::Mat(), rvec, tvec);

    // 棋盘格平面方程 (相机坐标系下)
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Vec3d n_cam(R.at<double>(0, 2), R.at<double>(1, 2),
                    R.at<double>(2, 2));
    cv::Vec3d p_cam(tvec.at<double>(0), tvec.at<double>(1),
                    tvec.at<double>(2));
    double d_plane = n_cam.dot(p_cam);

    // ---- 手动标注激光线: 用户点击两个端点 ----
    cv::Mat vis = undist.clone();
    cv::drawChessboardCorners(vis, boardSize, corners, found);

    ClickData clickData;
    clickData.baseImg = vis.clone();
    clickData.displayImg = vis.clone();

    std::string winName = "ExCalib: Click 2 endpoints of laser line (S=skip, R=reset)";
    cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);

    // 设置鼠标回调
    cv::setMouseCallback(winName, [](int event, int x, int y, int, void* userdata) {
      auto* data = static_cast<ClickData*>(userdata);
      if (event == cv::EVENT_LBUTTONDOWN && data->pts.size() < 2) {
        data->pts.push_back(cv::Point2d(x, y));

        // 刷新显示
        data->displayImg = data->baseImg.clone();
        for (size_t i = 0; i < data->pts.size(); i++) {
          cv::Point pt(static_cast<int>(data->pts[i].x),
                       static_cast<int>(data->pts[i].y));
          // 绘制十字标记
          cv::drawMarker(data->displayImg, pt, cv::Scalar(0, 0, 255),
                         cv::MARKER_CROSS, 20, 2);
        }
        // 如果已有两个点, 画连线
        if (data->pts.size() == 2) {
          cv::line(data->displayImg,
                   cv::Point(static_cast<int>(data->pts[0].x), static_cast<int>(data->pts[0].y)),
                   cv::Point(static_cast<int>(data->pts[1].x), static_cast<int>(data->pts[1].y)),
                   cv::Scalar(0, 255, 255), 1);
          cv::putText(data->displayImg, "Press ENTER to confirm, R to reset",
                      cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cv::Scalar(0, 255, 0), 2);
        }
      }
    }, &clickData);

    std::cout << "  [" << imgFile << "] Click 2 endpoints of laser line..." << std::endl;

    bool accepted = false;
    bool skipped = false;
    while (true) {
      cv::imshow(winName, clickData.displayImg);
      int key = cv::waitKey(30) & 0xFF;

      if (key == 's' || key == 'S') {  // 跳过此图
        skipped = true;
        break;
      }
      if (key == 'r' || key == 'R') {  // 重置点击
        clickData.pts.clear();
        clickData.displayImg = clickData.baseImg.clone();
      }
      if ((key == 13 || key == 10) && clickData.pts.size() == 2) {  // Enter 确认
        accepted = true;
        break;
      }
      if (key == 27) {  // ESC 退出全部
        cv::destroyAllWindows();
        std::cout << "  [ExCalib] Cancelled by user." << std::endl;
        return -1;
      }
    }
    cv::destroyWindow(winName);

    if (skipped) {
      std::cout << "  [SKIP] " << imgFile << " (user skipped)" << std::endl;
      continue;
    }

    // ---- 在两点之间等距插值 100 个点 ----
    const int numInterp = 100;
    std::vector<Point2D> points2D;
    points2D.reserve(numInterp);
    for (int i = 0; i < numInterp; i++) {
      double t = static_cast<double>(i) / (numInterp - 1);
      double u = clickData.pts[0].x * (1.0 - t) + clickData.pts[1].x * t;
      double v = clickData.pts[0].y * (1.0 - t) + clickData.pts[1].y * t;
      points2D.push_back({u, v});
    }

    // ---- 反投影: 2D 点 -> 棋盘格面 -> 3D 点 ----
    int ptsThisImage = 0;
    for (const auto &pt : points2D) {
      cv::Mat pixel = (cv::Mat_<double>(3, 1) << pt.u, pt.v, 1.0);
      cv::Mat ray = K_inv * pixel;
      cv::Vec3d dir(ray.at<double>(0), ray.at<double>(1), ray.at<double>(2));

      double denom = n_cam.dot(dir);
      if (std::abs(denom) < 1e-8)
        continue;
      double lambda = d_plane / denom;
      if (lambda < 0)
        continue;

      Eigen::Vector3d P3d(lambda * dir[0], lambda * dir[1], lambda * dir[2]);
      allLaserPts3D.push_back(P3d);
      ptsThisImage++;
    }

    std::cout << "  [OK] " << imgFile << " -> " << ptsThisImage
              << " laser 3D points" << std::endl;
    validImages++;
  }
  cv::destroyAllWindows();

  if (validImages < 3 || allLaserPts3D.size() < 50) {
    std::cerr
        << "[ExCalib] Not enough data! Need >= 3 valid images and >= 50 points."
        << " Got " << validImages << " images, " << allLaserPts3D.size()
        << " points." << std::endl;
    return -1;
  }
  std::cout << "\n[ExCalib] Total: " << validImages << " valid images, "
            << allLaserPts3D.size() << " 3D laser points." << std::endl;

  // ----- 5. SVD 平面拟合 -> π_C (相机坐标系下的激光面) -----
  // 将所有 3D 点去均值后做 SVD, 最小奇异值对应的右奇异向量即为平面法向量
  Eigen::Vector3d centroid(0, 0, 0);
  for (const auto &p : allLaserPts3D)
    centroid += p;
  centroid /= (double)allLaserPts3D.size();

  Eigen::MatrixXd A(allLaserPts3D.size(), 3);
  for (size_t i = 0; i < allLaserPts3D.size(); i++) {
    A(i, 0) = allLaserPts3D[i].x() - centroid.x();
    A(i, 1) = allLaserPts3D[i].y() - centroid.y();
    A(i, 2) = allLaserPts3D[i].z() - centroid.z();
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  Eigen::Vector3d normal =
      svd.matrixV().col(2); // 最小奇异值对应的右奇异向量 = 法向量

  // 确保法向量指向 +X_cam 方向 (东, 即激光面法向量应朝东)
  if (normal(0) < 0)
    normal = -normal;

  double d_c = -(normal.dot(centroid)); // n·P + d = 0  =>  d = -n·centroid
  Eigen::Vector4d pi_C(normal(0), normal(1), normal(2), d_c);

  double maxErr = 0, sumErr = 0;
  for (const auto &p : allLaserPts3D) {
    double err = std::abs(normal.dot(p) + d_c);
    maxErr = std::max(maxErr, err);
    sumErr += err;
  }
  double avgErr = sumErr / allLaserPts3D.size();

  std::cout << "\n=====================================" << std::endl;
  std::cout << "  Laser Plane Fitting Results" << std::endl;
  std::cout << "=====================================" << std::endl;
  std::cout << "  pi_C (camera coords) = [" << pi_C.transpose() << "]"
            << std::endl;
  std::cout << "  Normal direction: [" << normal.transpose() << "]"
            << std::endl;
  std::cout << "  Plane offset d = " << d_c << " mm" << std::endl;
  std::cout << "  Fit error: avg = " << avgErr << " mm, max = " << maxErr
            << " mm" << std::endl;

  if (avgErr > 2.0) {
    std::cerr << "\n[WARNING] Average fitting error > 2mm! Check image quality."
              << std::endl;
  }

  // ----- 6. 构建 T_CB (已知几何关系) -----
  // R_CB: Base(ENU) -> Camera(X-East, Y-Down, Z-North)
  //   X_cam = X_base   => R row 0 = [1, 0, 0]
  //   Y_cam = -Z_base  => R row 1 = [0, 0, -1]
  //   Z_cam = Y_base   => R row 2 = [0, 1, 0]
  // t_CB: 在相机坐标系下, 电机原点在 (+d, 0, 0)
  Eigen::Matrix4d T_CB = Eigen::Matrix4d::Identity();
  T_CB(0, 0) = 1;
  T_CB(0, 1) = 0;
  T_CB(0, 2) = 0;
  T_CB(0, 3) = camToMotorDist_mm;
  T_CB(1, 0) = 0;
  T_CB(1, 1) = 0;
  T_CB(1, 2) = -1;
  T_CB(1, 3) = 0;
  T_CB(2, 0) = 0;
  T_CB(2, 1) = 1;
  T_CB(2, 2) = 0;
  T_CB(2, 3) = 0;

  // ----- 7. 计算 π_R -----
  // 标定拍照时激光处于 θ=90° (excapture 将电机转到正北)
  // π_C 是 θ=90° 时相机坐标系下的激光平面
  // 正确公式: π_R = (T_CB · T_BR(90°))^T · π_C
  Eigen::Matrix4d T_BR_calib = Eigen::Matrix4d::Identity();
  // Rz(90°): cos90=0, sin90=1
  T_BR_calib(0, 0) =  0;  T_BR_calib(0, 1) = -1;
  T_BR_calib(1, 0) =  1;  T_BR_calib(1, 1) =  0;

  Eigen::Matrix4d T_CR = T_CB * T_BR_calib;       // 转子(θ=90°) → 相机
  Eigen::Vector4d pi_R = T_CR.transpose() * pi_C;  // π_R = T_CR^T · π_C
  // 归一化法向量部分
  double normR = pi_R.head<3>().norm();
  pi_R /= normR;

  std::cout << "\n  T_CB =\n" << T_CB << std::endl;
  std::cout << "  pi_R (rotor coords) = [" << pi_R.transpose() << "]"
            << std::endl;
  std::cout
      << "  Expected: pi_R ~ [1, 0, 0, -delta] (delta = laser offset from axis)"
      << std::endl;

  // ----- 8. 更新 config.yaml (保留已有内参和其他参数) -----
  // 先读取所有现有参数, 然后用标定结果覆盖 T_CB 和 LaserPlane_R
  cv::Mat existK, existDist;
  double stepAngle = 0.1125, startAngle = 0.0, endAngle = 90.0;
  std::string serialPort = "COM10";
  int serialBaud = 115200, cameraIdx = 1;
  {
    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    if (fs.isOpened()) {
      fs["Camera_Matrix"] >> existK;
      fs["Distortion_Coeffs"] >> existDist;
      if (!fs["StepAngle_Deg"].empty())
        fs["StepAngle_Deg"] >> stepAngle;
      if (!fs["Scan_Start_Angle"].empty())
        fs["Scan_Start_Angle"] >> startAngle;
      if (!fs["Scan_End_Angle"].empty())
        fs["Scan_End_Angle"] >> endAngle;
      if (!fs["Serial_Port"].empty())
        fs["Serial_Port"] >> serialPort;
      if (!fs["Serial_BaudRate"].empty())
        fs["Serial_BaudRate"] >> serialBaud;
      if (!fs["Camera_Index"].empty())
        fs["Camera_Index"] >> cameraIdx;
      fs.release();
    }
  }
  if (existK.empty())
    existK = K;
  if (existDist.empty())
    existDist = distCoeffs;

  // 写入更新后的配置文件
  cv::FileStorage fsOut(configPath, cv::FileStorage::WRITE);
  fsOut << "Camera_Matrix" << existK;        // 保留原有内参
  fsOut << "Distortion_Coeffs" << existDist; // 保留原有畸变参数

  // T_CB: Eigen -> cv::Mat
  cv::Mat T_CB_cv(4, 4, CV_64F);
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++)
      T_CB_cv.at<double>(r, c) = T_CB(r, c);
  fsOut << "T_CB" << T_CB_cv; // 写入标定后的外参

  // pi_R: Eigen -> cv::Mat
  cv::Mat pi_R_cv(4, 1, CV_64F);
  for (int i = 0; i < 4; i++)
    pi_R_cv.at<double>(i, 0) = pi_R(i);
  fsOut << "LaserPlane_R" << pi_R_cv; // 写入标定后的光平面

  fsOut << "StepAngle_Deg" << stepAngle;
  fsOut << "Scan_Start_Angle" << startAngle;
  fsOut << "Scan_End_Angle" << endAngle;
  fsOut << "Serial_Port" << serialPort;
  fsOut << "Serial_BaudRate" << serialBaud;
  fsOut << "Camera_Index" << cameraIdx;
  fsOut.release();

  std::cout << "\n[ExCalib] Config updated: " << configPath << std::endl;
  std::cout << "[ExCalib] Extrinsic calibration complete!" << std::endl;
  return 0;
}

// ============================================================
// 命令行解析与入口
// ============================================================
void printUsage() {
  std::cout << "Usage:" << std::endl;
  std::cout << "  CalibratorTool capture   --camera <idx> --outdir <dir>"
            << std::endl;
  std::cout << "  CalibratorTool calibrate --imgdir <dir> --board <WxH> "
            << "--square <mm> [--output <yaml>]" << std::endl;
  std::cout << "  CalibratorTool excapture --camera <idx> --outdir <dir>"
            << std::endl;
  std::cout << "  CalibratorTool excalib   --imgdir <dir> --board <WxH> "
            << "--square <mm> --dist <mm> [--config <yaml>]" << std::endl;
  std::cout << "\nExamples:" << std::endl;
  std::cout << "  CalibratorTool capture   --camera 1 --outdir ./calib_images"
            << std::endl;
  std::cout << "  CalibratorTool calibrate --imgdir ./calib_images --board 9x6 "
            << "--square 25.0" << std::endl;
  std::cout << "  CalibratorTool excapture --camera 1 --outdir ./excalib_images"
            << std::endl;
  std::cout
      << "  CalibratorTool excalib   --imgdir ./excalib_images --board 9x6 "
      << "--square 25.0 --dist 150" << std::endl;
}

int main(int argc, char *argv[]) {
  std::cout << "=====================================" << std::endl;
  std::cout << "  LineLaserScanner3D Calibrator Tool" << std::endl;
  std::cout << "=====================================" << std::endl;

  if (argc < 2) {
    printUsage();
    return 0;
  }

  std::string mode = argv[1];

  if (mode == "capture") {
    int camera = 1;
    std::string outDir = "./calib_images";
    for (int i = 2; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "--camera" && i + 1 < argc)
        camera = std::stoi(argv[++i]);
      if (arg == "--outdir" && i + 1 < argc)
        outDir = argv[++i];
    }
    return captureImages(camera, outDir);

  } else if (mode == "calibrate") {
    std::string imgDir = "./calib_images";
    int boardW = 9, boardH = 6;
    float squareSize = 25.0f;
    std::string output = "config.yaml";
    for (int i = 2; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "--imgdir" && i + 1 < argc)
        imgDir = argv[++i];
      if (arg == "--board" && i + 1 < argc) {
        std::string s = argv[++i];
        sscanf_s(s.c_str(), "%dx%d", &boardW, &boardH);
      }
      if (arg == "--square" && i + 1 < argc)
        squareSize = std::stof(argv[++i]);
      if (arg == "--output" && i + 1 < argc)
        output = argv[++i];
    }
    return runCalibration(imgDir, boardW, boardH, squareSize, output);

  } else if (mode == "excapture") {
    int camera = 1;
    std::string outDir = "./excalib_images";
    std::string port = "COM10";
    int baud = 115200;
    for (int i = 2; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "--camera" && i + 1 < argc)
        camera = std::stoi(argv[++i]);
      if (arg == "--outdir" && i + 1 < argc)
        outDir = argv[++i];
      if (arg == "--port" && i + 1 < argc)
        port = argv[++i];
      if (arg == "--baud" && i + 1 < argc)
        baud = std::stoi(argv[++i]);
    }
    return captureExtrinsicImages(camera, outDir, port, baud);

  } else if (mode == "excalib") {
    std::string imgDir = "./excalib_images";
    int boardW = 9, boardH = 6;
    float squareSize = 25.0f;
    std::string config = "config.yaml";
    double dist = 150.0;
    for (int i = 2; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "--imgdir" && i + 1 < argc)
        imgDir = argv[++i];
      if (arg == "--board" && i + 1 < argc) {
        std::string s = argv[++i];
        sscanf_s(s.c_str(), "%dx%d", &boardW, &boardH);
      }
      if (arg == "--square" && i + 1 < argc)
        squareSize = std::stof(argv[++i]);
      if (arg == "--config" && i + 1 < argc)
        config = argv[++i];
      if (arg == "--dist" && i + 1 < argc)
        dist = std::stod(argv[++i]);
    }
    return runExtrinsicCalibration(imgDir, boardW, boardH, squareSize, config,
                                   dist);

  } else {
    std::cerr << "Unknown mode: " << mode << std::endl;
    printUsage();
    return -1;
  }
}
