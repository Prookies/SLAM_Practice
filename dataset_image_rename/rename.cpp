#include <iostream>
#include <opencv2/opencv.hpp>
// 对文件夹进行操作的库
#include <unistd.h>
//#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>


using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    Mat out_image;
    // 使用Opencv的String类，
    String src_path = "/home/lab202/Dataset/TUM/rgbd_dataset_freiburg1_xyz/rgb/";
    String dst_path = "/home/lab202/Dataset/TUM/rgbd_dataset_freiburg1_xyz/output/";
    // String rgb_file = "/home/lab202/Dataset/TUM/rgbd_dataset_freiburg1_xyz/rgb.txt";

    // 判断目标文件夹是否存在，不存在则创建该文件夹
    int file_state = access(dst_path.c_str(), F_OK);
    cout << "当前文件件的状态为:" << file_state << endl;
    if(file_state == -1)
    {
        int create_state = mkdir(dst_path.c_str(), S_IRWXU);
        if(!create_state)
            cout << "MAKE SUCCESSFULLY" << endl;
        else
            cout << "MAKE ERRORLY" << endl;
    }


    vector<String> img_names;
    // 该函数可以获得src_path路径下的文件名，并把文件名存储在file_names中
    // 此外还可以通过后缀对该目录下的文件进行筛选
    glob(src_path, img_names, false);

    cout << "当前文件下的图像数量：" << img_names.size() << endl;
    for(size_t i=0; i<img_names.size(); i++)
    {
        // 取出当前图像命名
        String img_name = img_names[i];
        // 判断文件名的后缀
        // 如果没有找到，则会返回标记npos
        if(img_name.find(".png") != String::npos)
        {
            // 读取图像
            out_image = imread(img_name, CV_LOAD_IMAGE_UNCHANGED);
            if(out_image.empty())
            {
                cout << "CAN NOT READ THE IMAGE!" << endl;
                return -1;
            }
            // 格式化字符串
            String img_new_name = format("%04d", i) + ".png";
            // cout << img_new_name << endl;
            bool bimg_write = imwrite(dst_path+img_new_name, out_image);
            // 应该再判断一下是否写入成功
            if(!bimg_write)
            {
                cout << "CAN NOT WRITE THE IMAGE TO THE PATH!" << endl;
            }
            cout << dst_path+img_new_name << endl;
        }
        else{
            cout << "NOT A PNG IMAGE!" << endl;
        }
    }
    return 0;
}
