#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(0.1, 0.35, 0.2, 0.3).normalized();
    cout << "旋转矩阵：r = " << endl
         << q.toRotationMatrix() << endl;
    cout << "旋转矩阵转置后：rt = " << endl
         << q.toRotationMatrix().transpose() << endl;
    cout << "旋转矩阵的逆矩阵 r.inv = " << endl
         << q.inverse().toRotationMatrix() << endl;
    cout << "旋转矩阵乘以自身的转置： r*rt = " << endl
         << q.toRotationMatrix()*q.inverse().toRotationMatrix() << endl;

    cout << "四元数的逆乘以自身的旋转矩阵：q.inv*q = " << endl
         << (q.inverse()*q).toRotationMatrix() << endl;
}
