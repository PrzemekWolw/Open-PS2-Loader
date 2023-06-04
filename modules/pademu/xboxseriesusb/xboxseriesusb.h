#ifndef _DS3USB_H_
#define _DS3USB_H_

#include "irx.h"
#include "usbd.h"
#include "../pademu.h"

#define XBOX_VID 0x045E // Microsoft Corporation

//only one pid rn, i dont know any others
#define XBOX_SERIES_PID1 0x0B12  // Microsoft 1914 Series X/S Gamepad USB

#define MAX_BUFFER_SIZE 64 // Size of general purpose data buffer

typedef struct
{
    pad_device_t dev;
    int usb_id;
    int sema;
    int cmd_sema;
    int controlEndp;
    int interruptEndp;
    int outEndp;
    int usb_resultcode;
    UsbEndpointDescriptor *endin;
    UsbEndpointDescriptor *endout;
    u8 lrum;
    u8 rrum;
    u8 update_rum;
    u8 data[18];
    u8 analog_btn;
    u8 btn_delay;
} xboxseriesusb_device;

enum eHID {
    // {{{
    /* HID event flag */
    HID_FLAG_STATUS_REPORTED = 0x01,
    HID_FLAG_BUTTONS_CHANGED = 0x02,
    HID_FLAG_EXTENSION = 0x04,
    HID_FLAG_COMMAND_SUCCESS = 0x08,

    /* USB HID Transaction Header (THdr) */
    HID_USB_GET_REPORT_FEATURE = 0x03,
    HID_USB_SET_REPORT_OUTPUT = 0x02,
    HID_USB_DATA_INPUT = 0x01,

    /* Defines of various parameters for PS3 Game controller reports */
    PS3_F4_REPORT_ID = 0xF4,
    PS3_F4_REPORT_LEN = 0x04,

    PS3_01_REPORT_ID = 0x01,
    PS3_01_REPORT_LEN = 0x30,

    PS4_02_REPORT_ID = 0x02,
    PS4_11_REPORT_ID = 0x11,
    PS4_11_REPORT_LEN = 0x4D,
    // }}}
};

typedef struct {
    u8 ReportID; // 0x20
    u8 Zero;
    u16 id;

    union
    {
        u8 ButtonStateL; // Main buttons Low
        struct
        {
            u8 Sync : 1;
            u8 Dummy1 : 1;
            u8 Start : 1;
            u8 Back : 1;
            u8 A : 1;
            u8 B : 1;
            u8 X : 1;
            u8 Y : 1;
        };
    };
    union
    {
        u8 ButtonStateH; // Main buttons High
        struct
        {
            u8 Up : 1;
            u8 Down : 1;
            u8 Left : 1;
            u8 Right : 1;
            u8 LB : 1;
            u8 RB : 1;
            u8 LS : 1;
            u8 RS : 1;
        };
    };
    union
    {
        u16 LeftTrigger;
        struct
        {
            u8 LeftTriggerL;
            u8 LeftTriggerH;
        };
    };
    union
    {
        u16 RightTrigger;
        struct
        {
            u8 RightTriggerL;
            u8 RightTriggerH;
        };
    };
    union
    {
        s16 LeftStickX;
        struct
        {
            u8 LeftStickXL;
            u8 LeftStickXH;
        };
    };
    union
    {
        s16 LeftStickY;
        struct
        {
            u8 LeftStickYL;
            u8 LeftStickYH;
        };
    };
    union
    {
        s16 RightStickX;
        struct
        {
            u8 RightStickXL;
            u8 RightStickXH;
        };
    };
    union
    {
        s16 RightStickY;
        struct
        {
            u8 RightStickYL;
            u8 RightStickYH;
        };
    };
} __attribute__((packed)) xboxseriesreport_t;

#endif
