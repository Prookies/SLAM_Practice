#include "slamBase.hpp"

PointCloud::Ptr image2PointCloud(Mat rgb, Mat depth,
                                 CAMERA_INTRINSIC_PARAMETERS camera) {
  PointCloud::Ptr cloud(new PointCloud);
  for (int m = 0; m < depth.rows; m++)
    for (int n = 0; n < depth.cols; n++) {
      // 获取深度图中(m,n)处的值
      ushort d = depth.ptr<ushort>(m)[n];
      // d 可能没有值，若如此，跳过此点
      if (d == 0)
        continue;
      // d 存在值，则向点云增加一个点
      PointT p;
      // 计算这个点的空间坐标
      p.z = double(d) / camera.scale;
      p.x = (n - camera.cx) * p.z / camera.fx;
      p.y = (m - camera.cy) * p.z / camera.fy;

      // 从rgb图像中获取它的颜色
      p.b = rgb.ptr<uchar>(m)[n * 3];
      p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
      p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

      // 把p加入到点云中
      cloud->points.push_back(p);
    }
  // 设置并保存点云
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;
  return cloud;
}

PointCloud::Ptr pointCloudFusion(PointCloud::Ptr &original, FRAME &newFrame,
                                 Eigen::Isometry3d T,
                                 CAMERA_INTRINSIC_PARAMETERS camera) {
  // ---------- 开始你的代码  ------------- -//
  // 简单的点云叠加融合
  PointCloud::Ptr tmp(new PointCloud());
  tmp = image2PointCloud(newFrame.rgb, newFrame.depth, camera);
  //  cv::Mat im_d = newFrame.depth;
  //  cv::Mat im_rgb = newFrame.rgb;
  //  for (int m = 0; m < im_d.rows; m++)
  //    for (int n = 0; n < im_d.cols; n++) {
  //      ushort d = im_d.ptr<ushort>(m)[n];
  //      if (d == 0)
  //        continue;
  //      PointT p;
  //      p.z = double(d) / camera.scale;
  //      p.x = (n - camera.cx) * p.z / camera.fx;
  //      p.y = (m - camera.cy) * p.z / camera.fy;

  //      p.b = im_rgb.ptr<uchar>(m)[n * 3];
  //      p.g = im_rgb.ptr<uchar>(m)[n * 3 + 1];
  //      p.r = im_rgb.ptr<uchar>(m)[n * 3 + 2];

  //      tmp->points.push_back(p);
  //    }
  PointCloud::Ptr cloud(new PointCloud());
  pcl::transformPointCloud(*tmp, *cloud, T.matrix());

  *original += *cloud;

  return original;
  // ---------- 结束你的代码  ------------- -//
}

void readCameraTrajectory(string camTransFile,
                          vector<Eigen::Isometry3d> &poses) {
  ifstream fcamTrans(camTransFile);
  if (!fcamTrans.is_open()) {
    cerr << "trajectory is empty!" << endl;
    return;
  }

  // ---------- 开始你的代码  ------------- -//
  // 参考作业8 绘制轨迹
  std::string sPose_line;
  Eigen::Quaterniond q;
  Eigen::Vector3d t;

  while (std::getline(fcamTrans, sPose_line) && !sPose_line.empty()) {
    // char c = sPose_line.at(0);
    if (sPose_line[0] == '#')
      continue;
    std::istringstream ssPoseData(sPose_line);
    ssPoseData >> t.x() >> t.y() >> t.z() >> q.x() >> q.y() >> q.z() >> q.w();
    cout << "q = " << q.coeffs().transpose() << endl;
    cout << "t = " << t.transpose() << endl;
    // 一定要对变换矩阵进行初始化，很重要
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(q);
    T.pretranslate(t);
    cout << "T = " << T.matrix() << endl;
    poses.push_back(T);
  }
  // ---------- 结束你的代码  ------------- -//
}
