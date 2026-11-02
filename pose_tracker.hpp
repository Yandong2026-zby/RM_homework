#pragma once

#include <algorithm>
#include <opencv2/opencv.hpp>

// 匀速运动模型卡尔曼：状态 [x,y,z,vx,vy,vz]，观测 [x,y,z]
class CVKalman
{
public:
    CVKalman(float processNoise, float measureNoise)
        : kf_(6, 3, 0)
    {
        cv::setIdentity(kf_.measurementMatrix);
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(processNoise));
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(measureNoise));
    }

    bool ready() const { return ready_; }
    void invalidate() { ready_ = false; }

    void init(const cv::Vec3d& z)
    {
        kf_.statePost = cv::Mat::zeros(6, 1, CV_32F);
        for (int i = 0; i < 3; ++i)
            kf_.statePost.at<float>(i) = (float)z[i];
        kf_.errorCovPost = cv::Mat::eye(6, 6, CV_32F) * 1e3f;
        ready_ = true;
    }

    cv::Vec3d predict(double dt)
    {
        setDt(dt);
        return toVec(kf_.predict());
    }

    cv::Vec3d correct(const cv::Vec3d& z)
    {
        cv::Mat m(3, 1, CV_32F);
        for (int i = 0; i < 3; ++i)
            m.at<float>(i) = (float)z[i];
        return toVec(kf_.correct(m));
    }

private:
    void setDt(double dt)
    {
        dt = std::clamp(dt, 1e-3, 0.5);   // 防止卡顿时速度估计爆炸
        cv::setIdentity(kf_.transitionMatrix);
        for (int i = 0; i < 3; ++i)
            kf_.transitionMatrix.at<float>(i, i + 3) = (float)dt;
    }

    static cv::Vec3d toVec(const cv::Mat& m)
    {
        return cv::Vec3d(m.at<float>(0), m.at<float>(1), m.at<float>(2));
    }

    cv::KalmanFilter kf_;
    bool ready_ = false;
};

// 位置 + 姿态两个滤波器，负责“跟丢后继续预测”
class PoseTracker
{
public:
    void update(const cv::Vec3d& t, const cv::Vec3d& r, double dt,
                cv::Vec3d& predT, cv::Vec3d& predR)
    {
        if (!pos_.ready())
        {
            pos_.init(t);
            rot_.init(r);
            predT = t;
            predR = r;
            lost_ = 0;
            return;
        }
        predT = pos_.predict(dt);   // 先验估计
        predR = rot_.predict(dt);
        pos_.correct(t);
        rot_.correct(r);
        lost_ = 0;
    }

    // 没有检测结果时只预测，返回 false 表示跟丢太久
    bool predictOnly(double dt, cv::Vec3d& predT, cv::Vec3d& predR)
    {
        if (!pos_.ready()) return false;
        predT = pos_.predict(dt);
        predR = rot_.predict(dt);
        return ++lost_ <= kMaxLost;
    }

    bool ready() const { return pos_.ready(); }

    void reset()
    {
        pos_.invalidate();
        rot_.invalidate();
        lost_ = 0;
    }

private:
    static constexpr int kMaxLost = 15;   // 最多靠预测续 15 帧
    CVKalman pos_{1e2f, 1e2f};    // 位置：mm
    CVKalman rot_{1e-4f, 1e-4f};  // 姿态：rad
    int lost_ = 0;
};

// 用指定颜色画三根轴，用来区分实测和预测
inline void DrawPoseAxes(cv::Mat& img, const cv::Mat& K, const cv::Mat& D,
                         const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                         double len, const cv::Scalar& color, int thickness = 2)
{
    std::vector<cv::Point3f> axes = {
        {0.f, 0.f, 0.f},
        {(float)len, 0.f, 0.f},
        {0.f, (float)len, 0.f},
        {0.f, 0.f, (float)len}};
    std::vector<cv::Point2f> proj;
    cv::projectPoints(axes, rvec, tvec, K, D, proj);
    for (int i = 1; i < 4; ++i)
        cv::line(img, proj[0], proj[i], color, thickness, cv::LINE_AA);
}