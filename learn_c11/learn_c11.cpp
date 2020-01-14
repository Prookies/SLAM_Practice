#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

bool cmp(Point2i pt1, Point2i pt2)
{
    if(pt1.x == pt2.x)
        return pt1.y < pt2.y;
    else
        return pt1.x < pt2.x;
}

int main()
{
    vector<Point2i> vec;

    vec.push_back(Point2i(2,1));
    vec.push_back(Point2i(3,3));
    vec.push_back(Point2i(2,3));
    vec.push_back(Point2i(3,2));
    vec.push_back(Point2i(3,1));
    vec.push_back(Point2i(1,3));
    vec.push_back(Point2i(1,1));
    vec.push_back(Point2i(2,2));
    vec.push_back(Point2i(1,2));

    cout << "Before sort: " << endl;
    for(auto v: vec){
        cout << v << endl;
    }

    // sort(vec.begin(), vec.end(), cmp);

    sort(vec.begin(), vec.end(), [=](Point2i pt1, Point2i pt2)->bool{return ((pt1.x==pt2.x)?(pt1.y < pt2.y): ( pt1.x<pt2.x));});

    cout << "After sort: " << endl;
    for(auto v: vec)
    {
        cout << v << endl;
    }

    return 0;

}
