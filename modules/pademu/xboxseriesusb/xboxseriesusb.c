#include "types.h"
#include "loadcore.h"
#include "stdio.h"
#include "sifrpc.h"
#include "sysclib.h"
#include "usbd.h"
#include "usbd_macro.h"
#include "thbase.h"
#include "thsemap.h"
#include "xboxseriesusb.h"

//#define DPRINTF(x...) printf(x)
#define DPRINTF(x...)

#define REQ_USB_OUT (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#define REQ_USB_IN  (USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE)

#define MAX_PADS 4

static u8 usb_buf[MAX_BUFFER_SIZE + 32] __attribute((aligned(4))) = {0};

int usb_probe(int devId);
int usb_connect(int devId);
int usb_disconnect(int devId);

static void usb_release(int pad);
static void usb_config_set(int result, int count, void *arg);

UsbDriver usb_driver = {NULL, NULL, "xboxseriesusb", usb_probe, usb_connect, usb_disconnect};

static void readReport(u8 *data, int pad);
static int Rumble(u8 lrum, u8 rrum, int pad);

xboxseriesusb_device xboxseriesdev[MAX_PADS];
static u8 cmdcnt = 0;

int usb_probe(int devId)
{
    UsbDeviceDescriptor *device = NULL;

    DPRINTF("XBOXSERIESUSB: probe: devId=%i\n", devId);

    device = (UsbDeviceDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    if (device == NULL) {
        DPRINTF("XBOXSERIESUSB: Error - Couldn't get device descriptor\n");
        return 0;
    }

    if ((device->idVendor == XBOX_VID) &&
        (device->idProduct == XBOX_SERIES_PID1))
        return 1;

    return 0;
}

int usb_connect(int devId)
{
    int pad, epCount;
    UsbDeviceDescriptor *device;
    UsbConfigDescriptor *config;
    UsbInterfaceDescriptor *interface;
    UsbEndpointDescriptor *endpoint;

    DPRINTF("XBOXSERIESUSB: connect: devId=%i\n", devId);

    for (pad = 0; pad < MAX_PADS; pad++) {
        if (xboxseriesdev[pad].usb_id == -1)
            break;
    }

    if (pad >= MAX_PADS) {
        DPRINTF("XBOXSERIESUSB: Error - only %d device allowed !\n", MAX_PADS);
        return 1;
    }

    PollSema(xboxseriesdev[pad].sema);

    xboxseriesdev[pad].dev.id = pad;
    xboxseriesdev[pad].usb_id = devId;
    xboxseriesdev[pad].controlEndp = UsbOpenEndpoint(devId, NULL);

    device = (UsbDeviceDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    config = (UsbConfigDescriptor *)UsbGetDeviceStaticDescriptor(devId, device, USB_DT_CONFIG);
    interface = (UsbInterfaceDescriptor *)((char *)config + config->bLength);
    epCount = interface->bNumEndpoints - 1;
	endpoint = (UsbEndpointDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_ENDPOINT);

    do {
        if (endpoint->bmAttributes == USB_ENDPOINT_XFER_INT) {
            if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN && xboxseriesdev[pad].interruptEndp < 0) {
                xboxseriesdev[pad].interruptEndp = UsbOpenEndpointAligned(devId, endpoint);
                xboxseriesdev[pad].endin = endpoint;
                DPRINTF("XBOXSERIESUSB: register Event endpoint id =%i addr=%02X packetSize=%i\n", xboxseriesdev[pad].interruptEndp, endpoint->bEndpointAddress, (unsigned short int)endpoint->wMaxPacketSizeHB << 8 | endpoint->wMaxPacketSizeLB);
            }
            if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT && xboxseriesdev[pad].outEndp < 0) {
                xboxseriesdev[pad].outEndp = UsbOpenEndpointAligned(devId, endpoint);
                xboxseriesdev[pad].endout = endpoint;
                DPRINTF("DS34USB: register Output endpoint id =%i addr=%02X packetSize=%i\n", xboxseriespad[pad].outEndp, endpoint->bEndpointAddress, (unsigned short int)endpoint->wMaxPacketSizeHB << 8 | endpoint->wMaxPacketSizeLB);
            }
        }

        endpoint = (UsbEndpointDescriptor *)((char *)endpoint + endpoint->bLength);

    } while (epCount--);

    if (xboxseriesdev[pad].interruptEndp < 0 || xboxseriesdev[pad].outEndp < 0) {
        usb_release(pad);
        return 1;
    }

    UsbSetDeviceConfiguration(xboxseriesdev[pad].controlEndp, config->bConfigurationValue, usb_config_set, (void *)pad);
    SignalSema(xboxseriesdev[pad].sema);

    return 0;
}

int usb_disconnect(int devId)
{
    u8 pad;

    DPRINTF("XBOXSERIESUSB: disconnect: devId=%i\n", devId);

    for (pad = 0; pad < MAX_PADS; pad++) {
        if (xboxseriesdev[pad].usb_id == devId) {
            pademu_disconnect(&xboxseriesdev[pad].dev);
            break;
        }
    }

    if (pad < MAX_PADS)
        usb_release(pad);

    return 0;
}

static void usb_release(int pad)
{
    PollSema(xboxseriesdev[pad].sema);

    if (xboxseriesdev[pad].interruptEndp >= 0)
        UsbCloseEndpoint(xboxseriesdev[pad].interruptEndp);

	if (xboxseriesdev[pad].outEndp >= 0)
        UsbCloseEndpoint(xboxseriesdev[pad].outEndp);

    xboxseriesdev[pad].controlEndp = -1;
    xboxseriesdev[pad].interruptEndp = -1;
    xboxseriesdev[pad].outEndp = -1;
    xboxseriesdev[pad].usb_id = -1;

    SignalSema(xboxseriesdev[pad].sema);
}

static void usb_data_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)arg;

    //DPRINTF("XBOXSERIESUSB: usb_data_cb: res %d, bytes %d, arg %p \n", resultCode, bytes, arg);

    xboxseriesdev[pad].usb_resultcode = resultCode;

    SignalSema(xboxseriesdev[pad].sema);
}

static void usb_cmd_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)arg;

    //DPRINTF("XBOXSERIESUSB: usb_cmd_cb: res %d, bytes %d, arg %p \n", resultCode, bytes, arg);

    SignalSema(xboxseriesdev[pad].cmd_sema);
}

static void usb_config_set(int result, int count, void *arg)
{
    int pad = (int)arg;

    PollSema(xboxseriesdev[pad].sema);

    cmdcnt = 0;
    usb_buf[0] = 0x05;
    usb_buf[1] = 0x20;
    usb_buf[2] = cmdcnt++;
    usb_buf[3] = 0x01;
    usb_buf[4] = 0x00;
    UsbInterruptTransfer(xboxseriesdev[pad].outEndp, usb_buf, 5, NULL, NULL);
    DelayThread(10000);

    SignalSema(xboxseriesdev[pad].sema);

	pademu_connect(&xboxseriesdev[pad].dev);
}

#define MAX_DELAY 10

static void readReport(u8 *data, int pad)
{
    xboxseriesreport_t *report = (xboxseriesreport_t *)data;
    if (report->ReportID == 0x20) {
        xboxseriesdev[pad].data[0] = ~(report->Back | report->LS << 1 | report->RS << 2 | report->Start << 3 | report->Up << 4 | report->Right << 5 | report->Down << 6 | report->Left << 7);
        xboxseriesdev[pad].data[1] = ~((report->LeftTriggerH != 0) | (report->RightTriggerH != 0) << 1 | report->LB << 2 | report->RB << 3 | report->Y << 4 | report->B << 5 | report->A << 6 | report->X << 7);

        xboxseriesdev[pad].data[2] = report->RightStickXH + 128;    //rx
        xboxseriesdev[pad].data[3] = ~(report->RightStickYH + 128); //ry
        xboxseriesdev[pad].data[4] = report->LeftStickXH + 128;     //lx
        xboxseriesdev[pad].data[5] = ~(report->LeftStickYH + 128);  //ly

        xboxseriesdev[pad].data[6] = report->Right * 255; //right
        xboxseriesdev[pad].data[7] = report->Left * 255;  //left
        xboxseriesdev[pad].data[8] = report->Up * 255;    //up
        xboxseriesdev[pad].data[9] = report->Down * 255;  //down

        xboxseriesdev[pad].data[10] = report->Y * 255; //triangle
        xboxseriesdev[pad].data[11] = report->B * 255; //circle
        xboxseriesdev[pad].data[12] = report->A * 255; //cross
        xboxseriesdev[pad].data[13] = report->X * 255; //square

        xboxseriesdev[pad].data[14] = report->LB * 255;      //L1
        xboxseriesdev[pad].data[15] = report->RB * 255;      //R1
        xboxseriesdev[pad].data[16] = report->LeftTriggerH;  //L2
        xboxseriesdev[pad].data[17] = report->RightTriggerH; //R2
    }
}

static int Rumble(u8 lrum, u8 rrum, int pad)
{
    PollSema(xboxseriesdev[pad].cmd_sema);

    usb_buf[0] = 0x09;
    usb_buf[1] = 0x00;
    usb_buf[2] = cmdcnt++;
    usb_buf[3] = 0x09;  // Substructure (what substructure rest of this packet has)
    usb_buf[4] = 0x00;  // Mode
    usb_buf[5] = 0x0F;  // Rumble mask (what motors are activated) (0000 lT rT L R)
    usb_buf[6] = 0x00;  // lT force
    usb_buf[7] = 0x00;  // rT force
    usb_buf[8] = lrum;  // L force
    usb_buf[9] = rrum;  // R force
    usb_buf[10] = 0x80; // Length of pulse
    usb_buf[11] = 0x00; // Off period
    usb_buf[12] = 0x00; // Repeat count

    return UsbInterruptTransfer(xboxseriesdev[pad].outEndp, usb_buf, 13, usb_cmd_cb, (void *)pad);
}

static unsigned int timeout(void *arg)
{
    int sema = (int)arg;
    iSignalSema(sema);
    return 0;
}

static void TransferWait(int sema)
{
    iop_sys_clock_t cmd_timeout;

    cmd_timeout.lo = 200000;
    cmd_timeout.hi = 0;

    if (SetAlarm(&cmd_timeout, timeout, (void *)sema) == 0) {
        WaitSema(sema);
        CancelAlarm(timeout, NULL);
    }
}

void xboxseriesusb_set_rumble(u8 lrum, u8 rrum, int port)
{
    WaitSema(xboxseriesdev[port].sema);

    xboxseriesdev[port].update_rum = 1;
    xboxseriesdev[port].lrum = lrum;
    xboxseriesdev[port].rrum = rrum;

    SignalSema(xboxseriesdev[port].sema);
}

int xboxseriesusb_get_data(u8 *dst, int size, int port)
{
    int ret = 0;

    WaitSema(xboxseriesdev[port].sema);

    PollSema(xboxseriesdev[port].sema);

    ret = UsbInterruptTransfer(xboxseriesdev[port].interruptEndp, usb_buf, MAX_BUFFER_SIZE, usb_data_cb, (void *)port);

    if (ret == USB_RC_OK) {
        TransferWait(xboxseriesdev[port].sema);
        if (!xboxseriesdev[port].usb_resultcode)
            readReport(usb_buf, port);

        xboxseriesdev[port].usb_resultcode = 1;
    } else {
        UsbCloseEndpoint(xboxseriesdev[port].interruptEndp);
        xboxseriesdev[port].interruptEndp = UsbOpenEndpointAligned(xboxseriesdev[port].usb_id, xboxseriesdev[port].endin);
        DPRINTF("XBOXSERIESUSB: XBOXSERIESUSB_get_data usb transfer error %d\n", ret);
    }

    mips_memcpy(dst, xboxseriesdev[port].data, size);
    ret = xboxseriesdev[port].analog_btn & 1;

    if (xboxseriesdev[port].update_rum) {
        ret = Rumble(xboxseriesdev[port].lrum, xboxseriesdev[port].rrum, port);
        if (ret == USB_RC_OK) {
            TransferWait(xboxseriesdev[port].cmd_sema);
        } else {
            UsbCloseEndpoint(xboxseriesdev[port].outEndp);
            xboxseriesdev[port].outEndp = UsbOpenEndpointAligned(xboxseriesdev[port].usb_id, xboxseriesdev[port].endout);
            DPRINTF("XBOXSERIESUSB: LEDRumble usb transfer error %d\n", ret);
        }

        xboxseriesdev[port].update_rum = 0;
    }

    SignalSema(xboxseriesdev[port].sema);

    return ret;
}

void xboxseriesusb_set_mode(int mode, int lock, int port)
{
    if (lock == 3)
        xboxseriesdev[port].analog_btn = 3;
    else
        xboxseriesdev[port].analog_btn = mode;
}

void xboxseriesusb_reset()
{
    int pad;

    for (pad = 0; pad < MAX_PADS; pad++)
        usb_release(pad);
}

int _start(int argc, char *argv[])
{
    int pad;

    for (pad = 0; pad < MAX_PADS; pad++) {
        xboxseriesdev[pad].usb_id = -1;
		xboxseriesdev[pad].dev.id = -1;
        xboxseriesdev[pad].dev.pad_get_data = xboxseriesusb_get_data;
        xboxseriesdev[pad].dev.pad_set_rumble = xboxseriesusb_set_rumble;
        xboxseriesdev[pad].dev.pad_set_mode = xboxseriesusb_set_mode;

        xboxseriesdev[pad].lrum = 0;
        xboxseriesdev[pad].rrum = 0;
        xboxseriesdev[pad].update_rum = 1;
        xboxseriesdev[pad].sema = -1;
        xboxseriesdev[pad].cmd_sema = -1;
        xboxseriesdev[pad].controlEndp = -1;
        xboxseriesdev[pad].interruptEndp = -1;
        xboxseriesdev[pad].outEndp = -1;

        xboxseriesdev[pad].data[0] = 0xFF;
        xboxseriesdev[pad].data[1] = 0xFF;
        xboxseriesdev[pad].analog_btn = 0;

        mips_memset(&xboxseriesdev[pad].data[2], 0x7F, 4);
        mips_memset(&xboxseriesdev[pad].data[6], 0x00, 12);

        xboxseriesdev[pad].sema = CreateMutex(IOP_MUTEX_UNLOCKED);
        xboxseriesdev[pad].cmd_sema = CreateMutex(IOP_MUTEX_UNLOCKED);

        if (xboxseriesdev[pad].sema < 0 || xboxseriesdev[pad].cmd_sema < 0) {
            DPRINTF("XBOXSERIESUSB: Failed to allocate I/O semaphore.\n");
            return MODULE_NO_RESIDENT_END;
        }
    }

    if (UsbRegisterDriver(&usb_driver) != USB_RC_OK) {
        DPRINTF("XBOXSERIESUSB: Error registering USB devices\n");
        return MODULE_NO_RESIDENT_END;
    }

    return MODULE_RESIDENT_END;
}
