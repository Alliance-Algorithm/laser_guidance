#include <cstdio>
#include <cstring>
#include "MvCameraControl.h"

int main() {
    MV_CC_DEVICE_INFO_LIST list;
    std::memset(&list, 0, sizeof(list));
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
    if (ret != MV_OK) {
        std::printf("enum failed: %d\n", ret);
        return 1;
    }
    std::printf("device count: %u\n", list.nDeviceNum);
    for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
        auto* info = list.pDeviceInfo[i];
        if (!info) continue;
        if (info->nTLayerType == MV_USB_DEVICE) {
            auto* usb = &info->SpecialInfo.stUsb3VInfo;
            std::printf("USB dev %u: serial='%s' user_id='%s' model='%s'\n", i,
                usb->chSerialNumber, usb->chUserDefinedName, usb->chModelName);
        } else if (info->nTLayerType == MV_GIGE_DEVICE) {
            auto* gige = &info->SpecialInfo.stGigEInfo;
            std::printf("GIGE dev %u: serial='%s' user_id='%s' model='%s'\n", i,
                gige->chSerialNumber, gige->chUserDefinedName, gige->chModelName);
        }
    }
    return 0;
}
