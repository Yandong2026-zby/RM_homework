#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"

using namespace cv;
using namespace std;

// ============================================================
// 装甲板拍摄程序
//   ./capture [输出目录] [曝光us] [增益dB]
// 按键:
//   s / 空格  立即保存一帧
//   a         自动拍摄开关(识别到装甲板并稳定几帧后自动存)
//   m         显示/隐藏 HSV mask
//   [  ]      曝光减 / 加
//   q / ESC   退出
// 输出:
//   <输出目录>/full/   完整原图
//   <输出目录>/armor/  裁出来的装甲板区域(放大 20% 边距)
// ============================================================

// ---------------- 红色灯条 HSV 阈值 ----------------
// 红色在 HSV 里横跨 0 度和 180 度两端，必须分两段查再合并
static const Scalar LOW_RED_LO(0, 111, 121), LOW_RED_HI(10, 255, 255);
static const Scalar HIGH_RED_LO(159, 111, 121), HIGH_RED_HI(180, 255, 255);

struct Bar
{
    Rect rect;
    float cx() const { return rect.x + rect.width * 0.5f; }
    float cy() const { return rect.y + rect.height * 0.5f; }
};

// ================= 海康相机封装 =================
class HikCamera
{
public:
    ~HikCamera() { Close(); }

    bool Open(unsigned int index, double exposure_us, double gain_db)
    {
        if (MV_CC_Initialize() != MV_OK)
        {
            printf("MV_CC_Initialize 失败\n");
            return false;
        }
        initialized_ = true;

        MV_CC_DEVICE_INFO_LIST list;
        memset(&list, 0, sizeof(list));
        if (MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list) != MV_OK)
        {
            printf("枚举设备失败\n");
            return false;
        }
        if (list.nDeviceNum == 0)
        {
            printf("没找到相机，检查 USB3 线 / 是否被 MVS 客户端占用\n");
            return false;
        }
        if (index >= list.nDeviceNum)
        {
            printf("设备序号 %u 越界(共 %u 个)\n", index, list.nDeviceNum);
            return false;
        }

        if (MV_CC_CreateHandle(&handle_, list.pDeviceInfo[index]) != MV_OK)
        {
            printf("MV_CC_CreateHandle 失败\n");
            return false;
        }
        int nRet = MV_CC_OpenDevice(handle_);
        if (nRet != MV_OK)
        {
            printf("打开相机失败: 0x%x (关掉 MVS 客户端再试)\n", nRet);
            return false;
        }

        // 连续采集模式
        MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);

        // 曝光 / 增益，必须先关自动模式手动值才生效
        MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        SetExposure(exposure_us);
        MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
        MV_CC_SetFloatValue(handle_, "Gain", gain_db);

        PrintCurrentExposure();

        if (MV_CC_StartGrabbing(handle_) != MV_OK)
        {
            printf("开始取流失败\n");
            return false;
        }
        grabbing_ = true;
        return true;
    }

    bool SetExposure(double us)
    {
        if (handle_ == nullptr) return false;
        return MV_CC_SetFloatValue(handle_, "ExposureTime", us) == MV_OK;
    }

    double GetExposure()
    {
        MVCC_FLOATVALUE v = {0};
        if (handle_ && MV_CC_GetFloatValue(handle_, "ExposureTime", &v) == MV_OK)
            return v.fCurValue;
        return 0.0;
    }

    void PrintCurrentExposure()
    {
        MVCC_FLOATVALUE v = {0};
        if (handle_ && MV_CC_GetFloatValue(handle_, "ExposureTime", &v) == MV_OK)
            printf("曝光: %.0f us (范围 %.0f ~ %.0f)\n", v.fCurValue, v.fMin, v.fMax);
    }

    // 取一帧，转成 BGR8 给 OpenCV
    bool Grab(Mat& bgr)
    {
        if (handle_ == nullptr) return false;

        MV_FRAME_OUT out;
        memset(&out, 0, sizeof(out));
        if (MV_CC_GetImageBuffer(handle_, &out, 1000) != MV_OK)
            return false;   // 超时不算错误，下一帧再试

        const int w = (int)out.stFrameInfo.nExtendWidth;
        const int h = (int)out.stFrameInfo.nExtendHeight;
        const size_t need = (size_t)w * h * 3;
        if (buf_.size() < need) buf_.resize(need);

        MV_CC_PIXEL_CONVERT_PARAM cvt;
        memset(&cvt, 0, sizeof(cvt));
        cvt.nWidth = w;
        cvt.nHeight = h;
        cvt.pSrcData = out.pBufAddr;
        cvt.nSrcDataLen = out.stFrameInfo.nFrameLenEx;
        cvt.enSrcPixelType = out.stFrameInfo.enPixelType;
        cvt.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        cvt.pDstBuffer = buf_.data();
        cvt.nDstBufferSize = (unsigned int)buf_.size();

        const int ret = MV_CC_ConvertPixelType(handle_, &cvt);
        // 缓冲一定要立刻归还，否则很快取不到流
        MV_CC_FreeImageBuffer(handle_, &out);
        if (ret != MV_OK) return false;

        Mat(h, w, CV_8UC3, buf_.data()).copyTo(bgr);
        return true;
    }

    void Close()
    {
        if (handle_ == nullptr)
        {
            if (initialized_) { MV_CC_Finalize(); initialized_ = false; }
            return;
        }
        if (grabbing_) MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        grabbing_ = false;
        if (initialized_) { MV_CC_Finalize(); initialized_ = false; }
    }

private:
    void* handle_ = nullptr;
    bool grabbing_ = false;
    bool initialized_ = false;
    vector<unsigned char> buf_;
};

// ================= 装甲板识别 =================
void MakeMask(const Mat& frame, Mat& mask)
{
    Mat hsv, m1, m2;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, LOW_RED_LO, LOW_RED_HI, m1);
    inRange(hsv, HIGH_RED_LO, HIGH_RED_HI, m2);
    mask = m1 | m2;

    // 闭运算：把灯条内部被反光打断的像素连起来
    static const Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);
}

vector<Bar> FindBars(const Mat& mask)
{
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Bar> bars;
    for (const auto& c : contours)
    {
        Rect r = boundingRect(c);
        if (r.width < 2 || r.height < 20) continue;
        // 灯条是细长竖直条，长宽比至少 2
        if ((float)r.height / (float)r.width < 2.0f) continue;
        bars.push_back(Bar{r});
    }
    return bars;
}

// 两两配对灯条，返回最靠近画面中心的一对
// corners 顺序: 左上、右上、右下、左下（与 solvePnP 的世界坐标顺序一致）
bool PairArmor(const vector<Bar>& bars, const Size& imgSize,
               Bar& outL, Bar& outR, vector<Point2f>& corners)
{
    bool found = false;
    float bestDist2 = 1e18f;

    for (size_t i = 0; i < bars.size(); ++i)
    {
        for (size_t j = i + 1; j < bars.size(); ++j)
        {
            const Bar& a = bars[i];
            const Bar& b = bars[j];
            const Bar& L = (a.rect.x < b.rect.x) ? a : b;
            const Bar& R = (a.rect.x < b.rect.x) ? b : a;

            const float avgH = (L.rect.height + R.rect.height) * 0.5f;
            if (avgH <= 0) continue;

            const float hRatio = fabs((float)L.rect.height - (float)R.rect.height) / avgH;
            const float dx = R.cx() - L.cx();
            const float dy = fabs(L.cy() - R.cy());
            const float aspect = dx / avgH;

            // 这几条判据都按灯条高度归一化，所以远近都成立
            if (hRatio > 0.3f) continue;          // 两根灯条高度差不多
            if (dy > avgH * 0.5f) continue;       // 上端基本齐平
            if (aspect < 1.2f || aspect > 4.5f) continue; // 小装甲≈2.3，大装甲≈3.9

            const float mx = (L.cx() + R.cx()) * 0.5f;
            const float my = (L.cy() + R.cy()) * 0.5f;
            const float d2 = (mx - imgSize.width * 0.5f) * (mx - imgSize.width * 0.5f) +
                             (my - imgSize.height * 0.5f) * (my - imgSize.height * 0.5f);

            if (d2 < bestDist2)
            {
                bestDist2 = d2;
                found = true;
                outL = L;
                outR = R;
                // 角点直接取两根灯条的外端点，不再用固定的像素偏移
                corners = {
                    Point2f((float)L.rect.x, (float)L.rect.y),
                    Point2f((float)(R.rect.x + R.rect.width), (float)R.rect.y),
                    Point2f((float)(R.rect.x + R.rect.width), (float)(R.rect.y + R.rect.height)),
                    Point2f((float)L.rect.x, (float)(L.rect.y + L.rect.height)),
                };
            }
        }
    }
    return found;
}

// ================= 小工具 =================
string NowStamp()
{
    const auto now = chrono::system_clock::now();
    const time_t t = chrono::system_clock::to_time_t(now);
    const auto ms = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    tm tmv;
    localtime_r(&t, &tmv);
    char buf[64];
    snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d_%03d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec, (int)ms.count());
    return string(buf);
}

// 判断这一帧和上一张保存的图是不是几乎一样，避免连拍出一堆重复图
bool TooSimilar(const Mat& a, const Mat& b)
{
    if (a.empty() || b.empty()) return false;
    Mat ga, gb;
    resize(a, ga, Size(64, 64));
    resize(b, gb, Size(64, 64));
    cvtColor(ga, ga, COLOR_BGR2GRAY);
    cvtColor(gb, gb, COLOR_BGR2GRAY);
    return mean(abs(ga - gb))[0] < 4.0;
}

int main(int argc, char** argv)
{
    string outDir = "dataset";
    double exposure_us = 5000.0;
    double gain_db = 0.0;
    if (argc > 1) outDir = argv[1];
    if (argc > 2) exposure_us = atof(argv[2]);
    if (argc > 3) gain_db = atof(argv[3]);

    std::filesystem::create_directories(outDir + "/full");
    std::filesystem::create_directories(outDir + "/armor");

    HikCamera cam;
    if (!cam.Open(0, exposure_us, gain_db))
    {
        cam.Close();
        return -1;
    }

    namedWindow("armor capture", WINDOW_NORMAL);
    resizeWindow("armor capture", 1280, 960);

    bool showMask = false;
    bool autoShot = false;
    int stableCount = 0;
    int saved = 0;
    Mat lastSavedCrop;
    auto lastShotTime = chrono::steady_clock::now() - chrono::seconds(10);

    printf("\n按键: s保存  a自动拍摄  m显示mask  [ ]调曝光  q退出\n\n");

    Mat frame, mask, display;
    auto tPrev = chrono::steady_clock::now();
    double fps = 0;

    while (true)
    {
        if (!cam.Grab(frame) || frame.empty())
        {
            continue;
        }

        MakeMask(frame, mask);
        const vector<Bar> bars = FindBars(mask);

        Bar L, R;
        vector<Point2f> corners;
        const bool hasArmor = PairArmor(bars, frame.size(), L, R, corners);

        // ---------- 计时 ----------
        const auto tNow = chrono::steady_clock::now();
        const double dt = chrono::duration<double>(tNow - tPrev).count();
        if (dt > 0) fps = 0.9 * fps + 0.1 * (1.0 / dt);
        tPrev = tNow;

        // ---------- 画图 ----------
        frame.copyTo(display);
        for (const auto& b : bars)
            rectangle(display, b.rect, Scalar(255, 0, 0), 2);

        Mat crop;
        if (hasArmor)
        {
            polylines(display, corners, true, Scalar(0, 255, 0), 2);
            for (const auto& p : corners)
                circle(display, p, 4, Scalar(0, 255, 0), -1);

            // 裁装甲板区域，四周留 20% 边距
            Rect box = boundingRect(corners);
            const int padX = (int)(box.width * 0.2f);
            const int padY = (int)(box.height * 0.2f);
            box.x -= padX; box.y -= padY;
            box.width += padX * 2; box.height += padY * 2;
            box &= Rect(0, 0, frame.cols, frame.rows);   // 贴边时裁剪到画面内
            if (box.area() > 0) crop = frame(box).clone();

            stableCount++;
        }
        else
        {
            stableCount = 0;
        }

        // ---------- 保存 ----------
        auto SaveOnce = [&](const string& tag) {
            const string stamp = NowStamp();
            imwrite(outDir + "/full/" + stamp + ".png", frame);
            if (!crop.empty())
            {
                imwrite(outDir + "/armor/" + stamp + ".png", crop);
                lastSavedCrop = crop.clone();
            }
            lastShotTime = tNow;
            saved++;
            printf("[%s] 已保存 #%d  %s\n", tag.c_str(), saved, stamp.c_str());
        };

        bool wantSave = false;
        const string tag = autoShot ? "auto" : "manual";

        if (autoShot && hasArmor && stableCount >= 5)
        {
            const double since = chrono::duration<double>(tNow - lastShotTime).count();
            if (since > 0.4 && !TooSimilar(crop, lastSavedCrop))
                wantSave = true;
        }

        // ---------- HUD ----------
        const string status = hasArmor ? "ARMOR OK" : "no armor";
        putText(display, status, Point(20, 40), FONT_HERSHEY_SIMPLEX, 1.0,
                hasArmor ? Scalar(0, 255, 0) : Scalar(0, 0, 255), 2);
        putText(display, format("fps %.1f  bars %zu  saved %d", fps, bars.size(), saved),
                Point(20, 80), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(display, format("exposure %.0f us  auto:%s", cam.GetExposure(),
                                autoShot ? "ON" : "OFF"),
                Point(20, 115), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);

        imshow("armor capture", display);
        if (showMask) imshow("mask", mask);
        else destroyWindow("mask");

        if (wantSave) SaveOnce(tag);

        // ---------- 按键 ----------
        const int key = waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == 's' || key == ' ') SaveOnce("manual");
        if (key == 'a')
        {
            autoShot = !autoShot;
            printf("自动拍摄: %s\n", autoShot ? "开" : "关");
        }
        if (key == 'm') showMask = !showMask;
        if (key == '[' || key == ']')
        {
            const double cur = cam.GetExposure();
            const double next = (key == '[') ? cur / 1.2 : cur * 1.2;
            if (cam.SetExposure(next)) cout << "曝光 -> " << (int)cam.GetExposure() << " us" << endl;
        }
    }

    cam.Close();
    destroyAllWindows();
    printf("共保存 %d 组图片到 %s/\n", saved, outDir.c_str());
    return 0;
}
