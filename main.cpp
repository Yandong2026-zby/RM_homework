#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#define USE_CAMERA 0  

using namespace std;
using namespace cv;

Mat getImageFromLocal()
{
    Mat src = imread("test.jpg");
    if(src.empty())
    {
        cout << "找不到 test.jpg！图片放错文件夹了！" << endl;
    }
    return src;
}

int main()
{
    Mat frame = getImageFromLocal();
    if(frame.empty())
    {
        return -1;
    }

    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    
    Scalar lower_red = Scalar(0,80,40);
    Scalar upper_red = Scalar(15, 255,255);
    Mat mask;
    inRange(hsv, lower_red, upper_red, mask);

    
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Point2f> armor;

    for(auto &c : contours)
    {
        
        if(contourArea(c) < 2000) continue;

        vector<Point> approx;
        approxPolyDP(c, approx, arcLength(c, true)*0.04, true);

        if(approx.size() == 4)
        {
            for(auto p : approx)
                armor.push_back(Point2f(p.x, p.y));
            break;
        }
    }

    if(armor.size() != 4)
    {
        cout << "没找到装甲板！HSV颜色阈值不合适，需要调整！" << endl;
        imshow("mask",mask);
        waitKey(0);
        return -1;
    }
    vector<Point3f> world = {
        Point3f(-63.25, -68.5, 0),
        Point3f( 63.25, -68.5, 0),
        Point3f( 63.25,  68.5, 0),
        Point3f(-63.25,  68.5, 0)
    };
    

    
    Mat cam = (Mat_<double>(3,3) << 800,0,320, 0,800,240, 0,0,1);
    Mat dist = Mat::zeros(5,1,CV_64F);

    Mat rvec,tvec;

    solvePnP(world, armor, cam, dist, rvec, tvec);
    drawFrameAxes(frame, cam, dist, rvec, tvec, 50);

  
    for(auto p : armor) circle(frame,p,5,Scalar(0,255,0),2);

  
    cout << "=====作业PnP输出结果=====" << endl;
    cout << "平移向量XYZ(单位mm)：" << tvec << endl;
    cout << "旋转向量（姿态角度）：" << rvec << endl;

    imshow("最终作业效果",frame);
    imshow("颜色筛选蒙版mask",mask);

    waitKey(0);
    destroyAllWindows();
    return 0;
}
