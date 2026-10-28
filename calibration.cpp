#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <filesystem>

using namespace cv;
using namespace std;

int main()
{
    // 棋盘内角点数量：横向9，纵向6（与 calib/chessboard_9x6_25mm 一致）
    Size boardSize(9, 6);
    float squareSize = 0.025; // 棋盘每个方格边长，单位米，按你实物修改

    vector<vector<Point3f>> objPoints;
    vector<vector<Point2f>> imgPoints;

    vector<Point3f> obj;
    for(int y=0; y<boardSize.height; y++)
        for(int x=0; x<boardSize.width; x++)
            obj.emplace_back(x*squareSize, y*squareSize, 0);

    // 读取calib文件夹里相机采集的标定照片（bmp；png 是打印模板，不能参与标定）
    string imgPath = "./calib/";
    vector<String> imgFiles;
    glob(imgPath + "*.bmp", imgFiles);

    Size imageSize;
    for(auto &file : imgFiles)
    {
        Mat img = imread(file);
        if(img.empty())
        {
            cout << file << " 读取失败，跳过" << endl;
            continue;
        }
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        vector<Point2f> corners;

        bool found = findChessboardCorners(gray, boardSize, corners,
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

        if(found)
        {
            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
                TermCriteria(TermCriteria::EPS+TermCriteria::MAX_ITER, 30, 0.001));
            drawChessboardCorners(img, boardSize, corners, found);
            imshow("chessboard", img);
            waitKey(300);
            objPoints.push_back(obj);
            imgPoints.push_back(corners);
            imageSize = img.size(); // 用实际图像分辨率，别写死
            cout << file << " 识别成功" << endl;
        }
        else
        {
            cout << file << " 识别失败！这张照片要重拍" << endl;
        }
    }
    destroyAllWindows();

    // 有效视图太少时直接退出，否则 calibrateCamera 会抛 nimages > 0 断言
    if(imgPoints.size() < 4)
    {
        cerr << "有效标定图只有 " << imgPoints.size() << " 张，至少需要 4 张，标定中止。" << endl;
        return -1;
    }

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(objPoints, imgPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "有效视图 " << imgPoints.size() << " 张，图像尺寸 " << imageSize << endl;
    cout << "重投影误差 RMS = " << rms << " 像素" << endl;
    cout << "====相机内参矩阵====" << endl;
    cout << cameraMatrix << endl;
    cout << "====畸变系数====" << endl;
    cout << distCoeffs << endl;

    // 把参数保存到yaml文件，后面PnP代码直接加载
    FileStorage fs("camera_param.yaml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    // 记录标定分辨率：换分辨率使用时必须按比例缩放内参
    fs << "imageSize" << imageSize;
    fs.release();
    cout << "参数已保存 camera_param.yaml" << endl;
    return 0;
}