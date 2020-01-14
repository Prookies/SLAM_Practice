/****************************
 * 题目：SLAM问题的目标之一就是精确的估计相机运动的轨迹（姿态），如果我们将相机运动的轨迹绘制出来，就可以直观的观察它的运动是否符合预期。
 * 给定一个轨迹文件trajectory.txt，该文件的每一行由若干个数据组成，格式为 [time,
*tx, ty, tz, qx, qy, qz, qw],
 * 其中 time 为时间，tx,ty,tz 为平移部分，qx,qy,qz,qw
*是四元数表示的旋转部分，请完成数据读取部分的代码，绘制部分代码已经给出。
 *
* 本程序学习目标：
 * 熟悉李代数库Sophus安装及基本操作
 * 熟悉数据流读取基本操作
 * 需要安装pangolin，注意异常数据处理
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.10
****************************/

#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <pangolin/pangolin.h>
#include <sophus/se3.h>
#include <string>

using namespace std;

// path to trajectory file
static string trajectory_file = "../trajectory.txt";
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void load_trajectory(
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &poses);

int main(int argc, char **argv) {
  // 由于vector的元素是Eigen中的类型，Sophus使用了Eigen。所以其需要强调元素的内存分配和管理
  vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

  /// implement pose reading code
  // 开始你的代码
  load_trajectory(poses);
  // 结束你的代码

  // draw trajectory in pangolin
  DrawTrajectory(poses);
  return 0;
}

// 无需改动以下绘图程序
void DrawTrajectory(
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
  if (poses.empty()) {
    cerr << "Trajectory is empty!" << endl;
    return;
  }

  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glLineWidth(2);
    for (size_t i = 0; i < poses.size() - 1; i++) {
      glColor3f(1 - (float)i / poses.size(), 0.0f, (float)i / poses.size());
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);
  }
}

// 加载位姿数据
void load_trajectory(
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &poses) {
  LOG(INFO) << "load trajectory file: " << trajectory_file << endl;
  std::ifstream fs_trajectory;
  fs_trajectory.open(trajectory_file.c_str());
  if (!fs_trajectory.is_open()) {
    cerr << "Failed to open imu file! " << trajectory_file << endl;
  }

  std::string sPose_line;
  double dTime_stamp = 0.0;
  Eigen::Vector4d q_R;
  Eigen::Vector3d t;

  while (std::getline(fs_trajectory, sPose_line) && !sPose_line.empty()) {
    char c = sPose_line.at(0);
    if (c < '0' || c > '9')
      continue;

    std::istringstream ssPoseData(sPose_line);
    ssPoseData >> dTime_stamp >> t.x() >> t.y() >> t.z() >> q_R.x() >>
        q_R.y() >> q_R.z() >> q_R.w();
    //    LOG(INFO) << " t = " << t.transpose() << "; q_R = " << q_R.transpose()
    //              << endl;
    Sophus::SE3 SE3_qt(Eigen::Quaterniond(q_R), t);
    //    LOG(INFO) << "SE3 = " << endl << SE3_qt << endl;
    poses.push_back(SE3_qt);
  }
}
