/****************************
 * 题目：相机视场角比较小（比如手机摄像头）时，一般可以近似为针孔相机成像，三维世界中的直线成像也是直线。
 * 但是很多时候需要用到广角甚至鱼眼相机，此时会产生畸变，三维世界中的直线在图像里会弯曲。因此，需要做去畸变。
 * 给定一张广角畸变图像，以及相机的内参，请完成图像去畸变过程
 *
* 本程序学习目标：
 * 掌握图像去畸变原理
 *
 * 时间：2018.10
****************************/
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
static string image_file = "../test.png"; // 请确保路径正确

int main(int argc, char **argv) {

  double k1 = -0.28340811, k2 = 0.07395907; // 畸变参数
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

  cv::Mat image = cv::imread(image_file, CV_8UC1); // 图像是灰度图
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1); // 去畸变以后的图
  cv::imshow("image distorted", image);
  // 计算去畸变后图像的内容
  for (int v = 0; v < rows; v++)
    for (int u = 0; u < cols; u++) {

      double u_distorted = 0, v_distorted = 0;
      // 开始代码，注意(u,v)要先转化为归一化坐标
      // 获得真实的x和y归一化坐标
      double x_ud = (u - cx) / fx;
      double y_ud = (v - cy) / fy;
      // 计算r^2
      double r2 = x_ud * x_ud + y_ud * y_ud;
      double r4 = r2 * r2;
      // 根据畸变参数和公式计算有畸变的x和y的归一化坐标
      double x_d = x_ud * (1 + k1 * r2 + k2 * r4);
      double y_d = y_ud * (1 + k1 * r2 + k2 * r4);
      // 再经过相机内参得到的u和v对应的有畸变的图像中的u和v坐标值，也就是下面的u_distorted和v_distorted
      u_distorted = fx * x_d + cx;
      v_distorted = fy * y_d + cy;
      // 结束代码
      // 基于畸变图像image，获得去畸变图像image_undistort的像素值，采用最近邻插值
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols &&
          v_distorted < rows) {
        image_undistort.at<uchar>(v, u) =
            image.at<uchar>((int)v_distorted, (int)u_distorted);
      } else {
        image_undistort.at<uchar>(v, u) = 0;
      }
    }

  // 使用opencv库
  cv::Mat image_undistort_cv;
  cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat D = (cv::Mat_<double>(1, 4) << k1, k2, 0, 0);

  cv::undistort(image, image_undistort_cv, K, D);
  //  cv::cvUndistort2(image, image_undistort_cv, K, D);

  cv::imshow("cv image undistorted", image_undistort_cv);

  cv::imshow("image undistorted", image_undistort);
  cv::waitKey();

  return 0;
}
