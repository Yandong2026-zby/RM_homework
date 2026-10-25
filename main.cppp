#include <iostream>
#include <cstring>
#include "MvCameraControl.h"

int main()
{
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK)
    {
        std::cout << "枚举相机失败！错误码：" << nRet << std::endl;
        return -1;
    }

    std::cout << "检测到设备数量：" << stDeviceList.nDeviceNum << std::endl;

    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (pDeviceInfo == nullptr)
            continue;

        std::cout << "第" << i << "台相机：";
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            std::cout << pDeviceInfo->SpecialInfo.stGigEInfo.chModelName << std::endl;
        }
        else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
        {
            std::cout << pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName << std::endl;
        }
    }

    return 0;
}
