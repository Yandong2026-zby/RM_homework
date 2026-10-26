#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;

// 灯条结构体
struct Bar
{
    Rect rect;
};

int main()
{
    // 读取图片，把图片放到build文件夹内，命名armor3.jpg
    Mat frame = imread("armor3.jpg");
    if(frame.empty())
    {
        cout << "图片读取失败! 检查路径" << endl;
        return -1;
    }

    Mat hsv, mask;
    // ========= HSV红色阈值，两段红色合并 + 形态学降噪 =========
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    Mat mask1, mask2;
    // 低红 H:0~10
    inRange(hsv, Scalar(0, 111, 121), Scalar(10, 255, 255), mask1);
    // 高红 H:160~180
    inRange(hsv, Scalar(159,111,121), Scalar(180,255,255), mask2);
    mask = mask1 | mask2;

    // 形态学开运算，去除mask微小白色噪点
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    // =========轮廓查找、灯条筛选【修正长宽比逻辑】=========
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Bar> bars;
    for(auto &cont : contours)
    {
        Rect r = boundingRect(cont);
       float wh_ratio = (float)r.width / r.height;

// 【新增调试打印】把程序找到的所有轮廓的尺寸打印出来
cout << ">>> 发现轮廓: 宽=" << r.width << " 高=" << r.height << " 比例=" << wh_ratio << endl;

// 【临时注释掉过滤条件】先不要过滤，让它全部通过
 if(r.width < 15 || r.height < 50) continue; 
 if(wh_ratio > 0.2 && wh_ratio < 0.6)
        {
            Bar b;
            b.rect = r;
            bars.push_back(b);
            rectangle(frame, r, Scalar(255,0,0), 2);
        }
    }

    // =========变量声明=========
    vector<Point2f> armor_points;
    Point2f lu, ru, rd, ld;
    bool find_armor = false;

    // 两层for循环配对左右灯条
    for(int i = 0; i < bars.size(); i++)
    {
        for(int j = i + 1; j < bars.size(); j++)
        {
            if(i == j) continue;
            Bar &bar1 = bars[i];
            Bar &bar2 = bars[j];

            float h_diff = fabs(bar1.rect.height - bar2.rect.height);
            float y_diff = fabs(bar1.rect.y - bar2.rect.y);
            float dx = fabs(bar1.rect.x - bar2.rect.x);

            // 【新增调试打印】把任意两个轮廓的配对距离数据打印出来
cout << "  [尝试配对] 高度差=" << h_diff << " Y轴差=" << y_diff << " X距离=" << dx << endl;

// 【临时注释掉配对条件】让它强制配对，看看能不能连起来
 if(h_diff < 50 && y_diff < 50 && dx > 400 && dx < 1200)
            {
                Bar bar_left = bar1.rect.x < bar2.rect.x ? bar1 : bar2;
                Bar bar_right = bar1.rect.x < bar2.rect.x ? bar2 : bar1;

                // 左右灯条中心点
                float left_cx = bar_left.rect.x + bar_left.rect.width / 2.0f;
                float left_cy = bar_left.rect.y + bar_left.rect.height / 2.0f;
                float right_cx = bar_right.rect.x + bar_right.rect.width / 2.0f;
                float right_cy = bar_right.rect.y + bar_right.rect.height / 2.0f;

                // 装甲板半长，根据图片调整
                float armor_half = 250.0f;

                // 计算灯条连线的垂直方向
                float dx_seg = right_cx - left_cx;
                float dy_seg = right_cy - left_cy;
                float len = sqrt(dx_seg*dx_seg + dy_seg*dy_seg);
                float nx = -dy_seg / len;
                float ny = dx_seg / len;

                lu = Point2f(left_cx + nx * armor_half, left_cy + ny * armor_half);
                ld = Point2f(left_cx - nx * armor_half, left_cy - ny * armor_half);
                ru = Point2f(right_cx + nx * armor_half, right_cy + ny * armor_half);
                rd = Point2f(right_cx - nx * armor_half, right_cy - ny * armor_half);

                armor_points.clear();
                armor_points.push_back(lu);
                armor_points.push_back(ru);
                armor_points.push_back(rd);
                armor_points.push_back(ld);

                find_armor = true;
                cout << "✅配对成功!" << endl;
                cout << "lu:"<<lu.x<<","<<lu.y<<" ru:"<<ru.x<<","<<ru.y<<endl;
                cout << "rd:"<<rd.x<<","<<rd.y<<" ld:"<<ld.x<<","<<ld.y<<endl;
                break;
            }
        }
        if(find_armor) break;
    }

    if(!find_armor || armor_points.size() != 4)
    {
        cout << "❌没有找到装甲板! HSV颜色阈值不合适，需要调整! " << endl;
        imshow("mask", mask);
        waitKey(0);
        return -1;
    }

    // =========绘图代码=========
    circle(frame, lu, 5, Scalar(0, 255, 0), -1);
    circle(frame, ru, 5, Scalar(0, 255, 0), -1);
    circle(frame, rd, 5, Scalar(0, 255, 0), -1);
    circle(frame, ld, 5, Scalar(0, 255, 0), -1);

    line(frame, lu, ru, Scalar(0, 255, 0), 3);
    line(frame, ru, rd, Scalar(0, 255, 0), 3);
    line(frame, rd, ld, Scalar(0, 255, 0), 3);
    line(frame, ld, lu, Scalar(0, 255, 0), 3);

    putText(frame, "ARMOR", lu, FONT_HERSHEY_SIMPLEX,0.7,Scalar(0,255,0),2);

    // =========solvePnP部分=========
    vector<Point3f> world_points = {
        Point3f(-65, -25, 0),
        Point3f( 65, -25, 0),
        Point3f( 65, 25, 0),
        Point3f(-65, 25, 0)
    };
 
    //自动计算图片中心点坐标
    double cx = frame.cols / 2.0;
    double cy = frame.rows / 2.0;
    
    Mat camera_matrix = (Mat_<double>(3,3) <<
        1500, 0, cx,
        0, 1500, cy,
        0, 0, 1
    );
    Mat dist_coeffs = Mat::zeros(5,1,CV_64F);
    Mat rvec, tvec;
    if(armor_points.size() == 4)
    {
    bool pnp_ok = solvePnP(world_points, armor_points, camera_matrix, dist_coeffs, rvec, tvec);
    if(pnp_ok)
    {
        cout << "✅ PnP解算成功！"<<endl;
        cout << "平移向量 tvec(x,y,z): " << tvec << endl;
        cout << "旋转向量 rvec: " << rvec << endl;
        // 在画面绘制3D坐标轴，直观看到姿态
        drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 300);
    }
    else
    {
        cout << "❌ PnP解算失败，角点异常！" << endl;
    }
    }
    else
    {
       cout << "❌ 未找到有效的装甲板，无法进行 PnP 解算。" << endl;
    }

    imshow("result", frame);
    imshow("mask", mask);
    waitKey(0);
    return 0;
}
