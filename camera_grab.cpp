#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"

// 取流程序：接上工业相机，实时显示并保存图片
// 用法: ./grab [设备序号] [曝光时间us] [增益dB]
//   例: ./grab 0 5000 0
// 按键: q/ESC 退出   s 保存一张图片(存到 shots/ 目录)   空格 暂停/继续

static void PrintDeviceInfo(MV_CC_DEVICE_INFO* pInfo)
{
    if (pInfo == nullptr) return;

    if (pInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("USB  型号: %s  序列号: %s\n",
               (char*)pInfo->SpecialInfo.stUsb3VInfo.chModelName,
               (char*)pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
    }
    else if (pInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        unsigned int ip = pInfo->SpecialInfo.stGigEInfo.nCurrentIp;
        printf("GigE 型号: %s  IP: %u.%u.%u.%u\n",
               (char*)pInfo->SpecialInfo.stGigEInfo.chModelName,
               (ip >> 24) & 0xff, (ip >> 16) & 0xff, (ip >> 8) & 0xff, ip & 0xff);
    }
    else
    {
        printf("其他类型设备 (TLayerType=%d)\n", pInfo->nTLayerType);
    }
}

int main(int argc, char** argv)
{
    unsigned int nIndex = 0;
    double exposure_us = 5000.0;   // 默认 5ms，装甲板亮灯时可以适当调小
    double gain_db = 0.0;

    if (argc > 1) nIndex = (unsigned int)atoi(argv[1]);
    if (argc > 2) exposure_us = atof(argv[2]);
    if (argc > 3) gain_db = atof(argv[3]);

    // ========= 1. 初始化 SDK、枚举设备 =========
    int nRet = MV_CC_Initialize();
    if (nRet != MV_OK)
    {
        printf("MV_CC_Initialize 失败: 0x%x\n", nRet);
        return -1;
    }

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(stDeviceList));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK)
    {
        printf("MV_CC_EnumDevices 失败: 0x%x\n", nRet);
        MV_CC_Finalize();
        return -1;
    }

    printf("发现 %u 个设备:\n", stDeviceList.nDeviceNum);
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; ++i)
    {
        printf("  [%u] ", i);
        PrintDeviceInfo(stDeviceList.pDeviceInfo[i]);
    }

    if (stDeviceList.nDeviceNum == 0)
    {
        printf("没有找到相机，检查 USB3 线、是否被 MVS 客户端占用\n");
        MV_CC_Finalize();
        return -1;
    }
    if (nIndex >= stDeviceList.nDeviceNum)
    {
        printf("设备序号 %u 越界\n", nIndex);
        MV_CC_Finalize();
        return -1;
    }

    // ========= 2. 打开设备 =========
    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (nRet != MV_OK)
    {
        printf("MV_CC_CreateHandle 失败: 0x%x\n", nRet);
        MV_CC_Finalize();
        return -1;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK)
    {
        printf("MV_CC_OpenDevice 失败: 0x%x (相机可能被 MVS 客户端占用，先关掉它)\n", nRet);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return -1;
    }

    // GigE 相机需要设置最佳包大小，USB 相机这步会自动失败，忽略即可
    int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
    if (nPacketSize > 0)
    {
        MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", nPacketSize);
    }

    // ========= 3. 相机参数 =========
    // 连续采集模式(非软触发)
    MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);

    // 曝光时间与增益：曝光自动必须先关掉，手动值才生效
    MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposure_us);
    printf("设置曝光 %.0f us -> %s\n", exposure_us, nRet == MV_OK ? "OK" : "失败(可能超范围)");

    MV_CC_SetEnumValue(handle, "GainAuto", MV_GAIN_MODE_OFF);
    nRet = MV_CC_SetFloatValue(handle, "Gain", gain_db);
    printf("设置增益 %.1f dB -> %s\n", gain_db, nRet == MV_OK ? "OK" : "失败(可能超范围)");

    // 检查相机实际生效的参数，避免"设置成功但没生效"
    MVCC_FLOATVALUE stExposure = {0};
    if (MV_CC_GetFloatValue(handle, "ExposureTime", &stExposure) == MV_OK)
    {
        printf("相机实际曝光: %.0f us (范围 %.0f ~ %.0f)\n",
               stExposure.fCurValue, stExposure.fMin, stExposure.fMax);
    }

    // ========= 4. 开始取流 =========
    nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK)
    {
        printf("MV_CC_StartGrabbing 失败: 0x%x\n", nRet);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return -1;
    }

    std::filesystem::create_directories("shots");

    std::vector<unsigned char> dst;
    cv::Mat frame;
    bool paused = false;
    bool printed_info = false;
    int shot_count = 0;

    printf("\n取流中... q/ESC 退出, s 保存图片, 空格 暂停\n\n");

    while (true)
    {
        MV_FRAME_OUT stImageInfo;
        memset(&stImageInfo, 0, sizeof(stImageInfo));
        nRet = MV_CC_GetImageBuffer(handle, &stImageInfo, 1000);
        if (nRet != MV_OK)
        {
            printf("取图失败/超时: 0x%x\n", nRet);
            continue;
        }

        const int w = (int)stImageInfo.stFrameInfo.nExtendWidth;
        const int h = (int)stImageInfo.stFrameInfo.nExtendHeight;

        // 转成 OpenCV 能直接用的 BGR8
        const size_t need = (size_t)w * h * 3;
        if (dst.size() < need) dst.resize(need);

        MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
        memset(&stConvertParam, 0, sizeof(stConvertParam));
        stConvertParam.nWidth = w;
        stConvertParam.nHeight = h;
        stConvertParam.pSrcData = stImageInfo.pBufAddr;
        stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLenEx;
        stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        stConvertParam.pDstBuffer = dst.data();
        stConvertParam.nDstBufferSize = (unsigned int)dst.size();

        const int cvtRet = MV_CC_ConvertPixelType(handle, &stConvertParam);

        // 缓冲区用完必须立刻还回去，否则取流会卡死
        MV_CC_FreeImageBuffer(handle, &stImageInfo);

        if (cvtRet != MV_OK)
        {
            printf("像素格式转换失败: 0x%x\n", cvtRet);
            continue;
        }

        cv::Mat img(h, w, CV_8UC3, dst.data());
        img.copyTo(frame);

        if (!printed_info)
        {
            printf("图像尺寸: %d x %d，帧号: %u\n", w, h, stImageInfo.stFrameInfo.nFrameNum);
            printed_info = true;
        }

        if (!paused)
        {
            cv::imshow("camera", frame);
        }

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == ' ')
        {
            paused = !paused;
            printf(paused ? "已暂停\n" : "继续取流\n");
        }
        if (key == 's')
        {
            char name[128];
            snprintf(name, sizeof(name), "shots/shot_%02d.png", shot_count++);
            cv::imwrite(name, frame);
            printf("已保存 %s\n", name);
        }
    }

    // ========= 5. 清理资源（顺序不能乱）=========
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    cv::destroyAllWindows();

    printf("已退出\n");
    return 0;
}
