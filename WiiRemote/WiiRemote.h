/*
WiiRemote.h - WiiRemote Bluetooth stack on Arduino with USB Host Shield
Copyright (C) 2010 Tomo Tanaka

This program is based on <wiiblue.pde> which is developed by Oleg Mazurov. This
program also needs MAX3421E and USB libraries for Arduino written by Oleg. The
source codes can be grabbed from <https://github.com/felis/USB_Host_Shield>.


This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _WIIREMOTE_H_
#define _WIIREMOTE_H_

#include "Max3421e.h"
#include "Usb.h"

enum eDescriptor {
    // {{{
    /* CSR Bluetooth data taken from descriptors */
    BT_ADDR = 1,
    CSR_VID = 0x0a12,   // CSR Vendor ID
    CSR_PID = 0x0001,   // CSR Product ID
    BT_CONFIGURATION = 1,
    BT_INTERFACE = 0,   // Only use interface 0
    BT_NUM_EP = 4,
    INT_MAXPKTSIZE = 16,
    BULK_MAXPKTSIZE = 64,
    EP_INTERRUPT = 0x03,    // endpoint type
    EP_BULK = 0x02,         // endpoint type
    EP_POLL = 0x01,         // interrupt poll interval
    // }}}
};

enum eUSBPipe {
    // {{{
    CONTROL_PIPE = 0,
    EVENT_PIPE = 1,
    DATAIN_PIPE = 2,
    DATAOUT_PIPE = 3,
    // }}}
};

enum eHCI {
    // {{{
    /* Bluetooth HCI states for HCI_task() */
    HCI_INIT_STATE = 0,
    HCI_RESET_STATE,
    HCI_BD_ADDR_STATE,
    HCI_INQUIRY_STATE,
    HCI_READY_CONNECT_STATE,
    HCI_CONNECT_OUT_STATE,
    HCI_CONNECTED_STATE,

    /* HCI OpCode Group Field (OGF) */
    HCI_OGF_LINK_CNTRL  = (0x01 << 2),  // OGF: Link Control Commands
    HCI_OGF_LINK_POLICY = (0x02 << 2),  // OGF: Link Policy Commands
    HCI_OGF_CTRL_BBAND  = (0x03 << 2),  // OGF: Controller & Baseband Commands
    HCI_OGF_INFO_PARAM  = (0x04 << 2),  // OGF: Informational Parameters

    /* HCI OpCode Command Field (OCF) */
    HCI_OCF_INQUIRY           = 0x001,  // OGF = 0x01
    HCI_OCF_INQUIRY_CANCEL    = 0x002,  // OGF = 0x01
    HCI_OCF_CREATE_CONNECTION = 0x005,  // OGF = 0x01
    HCI_OCF_RESET             = 0x003,  // OGF = 0x03
    HCI_OCF_READ_BD_ADDR      = 0x009,  // OGF = 0x04

    /* HCI events managed */
    HCI_EVENT_INQUIRY_COMPLETE   = 0x01,
    HCI_EVENT_INQUIRY_RESULT     = 0x02,
    HCI_EVENT_CONNECT_COMPLETE   = 0x03,
    HCI_EVENT_CONNECT_REQUEST    = 0x04,    // do nothing so far
    HCI_EVENT_DISCONN_COMPLETE   = 0x05,
    HCI_EVENT_QOS_SETUP_COMPLETE = 0x0d,    // do nothing so far
    HCI_EVENT_COMMAND_COMPLETE   = 0x0e,
    HCI_EVENT_COMMAND_STATUS     = 0x0f,
    HCI_EVENT_NUM_COMPLETED_PKT  = 0x13,    // do nothing so far

    /* HCI event flags for hci_event_flag */
    HCI_FLAG_COMMAND_COMPLETE = 0x01,
    HCI_FLAG_COMMAND_STATUS   = 0x02,
    HCI_FLAG_CONNECT_COMPLETE = 0x04,
    HCI_FLAG_CONNECT_OK       = 0x08,
    HCI_FLAG_DISCONN_COMPLETE = 0x10,
    HCI_FLAG_INQUIRY_RESULT   = 0x20,
    HCI_FLAG_INQUIRY_COMPLETE = 0x40,

    // used in control endpoint header for HCI Commands
    bmREQ_HCI_OUT = (USB_SETUP_HOST_TO_DEVICE|
                     USB_SETUP_TYPE_CLASS|
                     USB_SETUP_RECIPIENT_DEVICE),
    HCI_COMMAND_REQ = 0,
    // }}}
};

enum eL2CAP {
    // {{{
    /* Bluetooth L2CAP PSM */
    L2CAP_PSM_WRITE = 0x11,
    L2CAP_PSM_READ  = 0x13,

    /* Bluetooth L2CAP states for L2CAP_task() */
    L2CAP_DOWN_STATE = 0,
    L2CAP_INIT_STATE,
    L2CAP_CONTROL_CONNECTING_STATE,
    L2CAP_CONTROL_CONFIGURING_STATE,
    L2CAP_INTERRUPT_CONNECTING_STATE,
    L2CAP_INTERRUPT_CONFIGURING_STATE,
    L2CAP_CONNECTED_STATE,
    L2CAP_WIIREMOTE_LED_STATE,
    L2CAP_WIIREMOTE_CAL_STATE,
    //L2CAP_CALIBRATION_STATE,
    //L2CAP_REPORT_MODE_STATE,
    //L2CAP_LED1_STATE,
    //L2CAP_LED2_STATE,
    //L2CAP_LED3_STATE,
    //L2CAP_LED4_STATE,
    L2CAP_READY_STATE,
    L2CAP_DISCONNECT_STATE,

    /* L2CAP event flags */
    L2CAP_EV_COMMAND_CONNECTED        = 0x01,
    L2CAP_EV_COMMAND_CONFIGURED       = 0x02,
    L2CAP_EV_COMMAND_CONFIG_REQ       = 0x04,
    L2CAP_EV_INTERRUPT_CONNECTED      = 0x08,
    L2CAP_EV_INTERRUPT_CONFIGURED     = 0x10,
    L2CAP_EV_INTERRUPT_CONFIG_REQ     = 0x20,
    L2CAP_EV_COMMAND_DISCONNECT_REQ   = 0x40,
    L2CAP_EV_INTERRUPT_DISCONNECT_REQ = 0x80,

    /* L2CAP signaling command */
    L2CAP_CMD_CONNECTION_REQUEST  = 0x02,
    L2CAP_CMD_CONNECTION_RESPONSE = 0x03,
    L2CAP_CMD_CONFIG_REQUEST      = 0x04,
    L2CAP_CMD_CONFIG_RESPONSE     = 0x05,
    L2CAP_CMD_DISCONNECT_REQUEST  = 0x06,
    L2CAP_CMD_DISCONNECT_RESPONSE = 0x07,

/*  HCI ACL Data Packet
 *
 *   buf[0]          buf[1]          buf[2]          buf[3]
 *   0       4       8    11 12      16              24            31 MSB
 *  .-+-+-+-+-+-+-+-|-+-+-+-|-+-|-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *  |      HCI Handle       |PB |BC |       Data Total Length       |   HCI ACL Data Packet
 *  .-+-+-+-+-+-+-+-|-+-+-+-|-+-|-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *
 *   buf[4]          buf[5]          buf[6]          buf[7]
 *   0               8               16                            31 MSB
 *  .-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *  |            Length             |          Channel ID           |   Basic L2CAP header
 *  .-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *
 *   buf[8]          buf[9]          buf[10]         buf[11]
 *   0               8               16                            31 MSB
 *  .-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *  |     Code      |  Identifier   |            Length             |   Control frame (C-frame)
 *  .-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.   (signaling packet format)
 */
    // }}}
};

enum eHID {
    // {{{
    /* HID event flag */
    HID_FLAG_STATUS_REPORTED  = 0x01,
    HID_FLAG_BUTTONS_CHANGED  = 0x02,
    HID_FLAG_EXTENSION        = 0x04,
    HID_FLAG_COMMAND_SUCCESS  = 0x08,
    HID_FLAG_READ_CALIBRATION = 0x10,

    /* Bluetooth HID Transaction Header (THdr) */
    HID_THDR_SET_REPORT_OUTPUT = 0x52,
    HID_THDR_DATA_INPUT        = 0xa1,

    /* HID interface, output report to WiiRemote */
    OUTPUT_REPORT_LED        = 0x11,
    OUTPUT_REPORT_MODE       = 0x12,
    OUTPUT_REPORT_STATUS     = 0x15,
    OUTPUT_REPORT_WRITE_DATA = 0x16,
    OUTPUT_REPORT_READ_DATA  = 0x17,

    /* HID interface, input report from WiiRemote */
    INPUT_REPORT_STATUS        = 0x20,  // Status information
    INPUT_REPORT_READ_DATA     = 0x21,  // Read memory and register data
    INPUT_REPORT_ACK           = 0x22,  // Acknowledge output report, return function result
    INPUT_REPORT_BUTTONS       = 0x30,  // Buttons data
    INPUT_REPORT_BUTTONS_ACCEL = 0x31,  // Buttons and accelerometer data
    INPUT_REPORT_IR_EXT_ACCEL  = 0x37,  // Buttons and accelerometer with 10 IR bytes and 6 extension bytes
    // }}}
};

enum eBDAddressMode {
    // {{{
    /* Wii Remote BD_ADDR acquisition mode */
    BD_ADDR_FIXED = 0,  // wiimote_bdaddr_ needs to be set
    BD_ADDR_INQUIRY,    // using HCI_INQUIRY to get BD_ADDR
    // }}}
};

enum eWiiRemoteLED {
    // {{{
    WIIREMOTE_LED1 = 0x10,
    WIIREMOTE_LED2 = 0x20,
    WIIREMOTE_LED3 = 0x40,
    WIIREMOTE_LED4 = 0x80,
    // }}}
};

enum eWiiRemoteButtons {
    // {{{
    WIIREMOTE_LEFT  = 0x0001,
    WIIREMOTE_RIGHT = 0x0002,
    WIIREMOTE_DOWN  = 0x0004,
    WIIREMOTE_UP    = 0x0008,
    WIIREMOTE_PLUS  = 0x0010,
    WIIREMOTE_TWO   = 0x0100,
    WIIREMOTE_ONE   = 0x0200,
    WIIREMOTE_B     = 0x0400,
    WIIREMOTE_A     = 0x0800,
    WIIREMOTE_MINUS = 0x1000,
    WIIREMOTE_HOME  = 0x8000,
    // }}}
};

enum eWiiRemote {
    ACCEL_AVERAGE_SIZE = 2,
};

enum eBUF_SIZE {
    MAX_BUFFER_SIZE = 64,   // Size of general purpose data buffer
};


/* HCI macro */
#define hci_timeout             (hci_counter_-- == 0)

/* Macros for HCI event flag tests */
#define hci_command_complete    (hci_event_flag_ & HCI_FLAG_COMMAND_COMPLETE)
#define hci_command_status      (hci_event_flag_ & HCI_FLAG_COMMAND_STATUS)
#define hci_connect_complete    (hci_event_flag_ & HCI_FLAG_CONNECT_COMPLETE)
#define hci_connect_ok          (hci_event_flag_ & HCI_FLAG_CONNECT_OK)
#define hci_disconn_complete    (hci_event_flag_ & HCI_FLAG_DISCONN_COMPLETE)
#define hci_inquiry_result      (hci_event_flag_ & HCI_FLAG_INQUIRY_RESULT)
#define hci_inquiry_complete    (hci_event_flag_ & HCI_FLAG_INQUIRY_COMPLETE)

/* Macros for L2CAP event flag tests */
#define l2cap_command_connected      (l2cap_event_status_ & L2CAP_EV_COMMAND_CONNECTED)
#define l2cap_command_configured     (l2cap_event_status_ & L2CAP_EV_COMMAND_CONFIGURED)
#define l2cap_command_config_req     (l2cap_event_status_ & L2CAP_EV_COMMAND_CONFIG_REQ)
#define l2cap_interrupt_connected    (l2cap_event_status_ & L2CAP_EV_INTERRUPT_CONNECTED)
#define l2cap_interrupt_configured   (l2cap_event_status_ & L2CAP_EV_INTERRUPT_CONFIGURED)
#define l2cap_interrupt_config_req   (l2cap_event_status_ & L2CAP_EV_INTERRUPT_CONFIG_REQ)
#define l2cap_command_disconnected   (l2cap_event_status_ & L2CAP_EV_COMMAND_DISCONNECT_REQ)
#define l2cap_interrupt_disconnected (l2cap_event_status_ & L2CAP_EV_INTERRUPT_DISCONNECT_REQ)

/* Macros for L2CAP event task tests */
#define acl_handle_ok   ((buf[0] | (buf[1] << 8)) == (hci_handle_ | 0x2000))
#define l2cap_control   ((buf[6] | (buf[7] << 8)) == 0x0001)  // Channel ID for ACL-U
#define l2cap_interrupt ((buf[6] | (buf[7] << 8)) == interrupt_scid_)
#define l2cap_command   ((buf[6] | (buf[7] << 8)) == command_scid_)
#define l2cap_connection_response       (buf[8] == 0x03) 
#define l2cap_connection_success        ((buf[16] | (buf[17] << 8)) == 0x0000) 
#define l2cap_configuration_request     (buf[8] == 0x04) 
#define l2cap_configuration_response    (buf[8] == 0x05) 
#define l2cap_configuration_success     ((buf[16] | (buf[17] << 8)) == 0x0000)
#define l2cap_disconnect_request        (buf[8] == 0x06) 

/* Macros for HID event flag tests */
#define hid_status_reported     (hid_flags_ & HID_FLAG_STATUS_REPORTED)
#define hid_buttons_changed     (hid_flags_ & HID_FLAG_BUTTONS_CHANGED)
#define hid_extension           (hid_flags_ & HID_FLAG_EXTENSION)
#define hid_command_success     (hid_flags_ & HID_FLAG_COMMAND_SUCCESS)
#define hid_read_calibration    (hid_flags_ & HID_FLAG_READ_CALIBRATION)
#define hid_handshake_success   (buf[8] == 0)


typedef struct Point3i {
    uint16_t X;
    uint16_t Y;
    uint16_t Z;
} Point3i_t;

typedef struct Point3f {
    float X;
    float Y;
    float Z;
} Point3f_t;

typedef struct Point3Cal {
    Point3i_t offset;   // calibrated zero offset
    Point3i_t gravity;  // force of gravity
} Point3Cal_t;

typedef struct Buttons {
    /* true if a button are pressed, false otherwise */
    bool Left;
    bool Right;
    bool Down;
    bool Up;
    bool Plus;
    bool Two;
    bool One;
    bool B;
    bool A;
    bool Minus;
    bool Home;
} Buttons_t;

typedef struct {
    Point3f_t Accel;
    Buttons_t Button;
} Report_t;


class WiiRemote {
  public:
    WiiRemote(void);
    ~WiiRemote(void);

    void init(void);
    void task(void (*)());
    void setBDAddress(uint8_t *, int);
    void setBDAddressMode(eBDAddressMode);
    uint8_t setLED(uint8_t/*eWiiRemoteLED*/);
    uint8_t setReportMode(uint8_t);         // TODO declare as private
    uint8_t readData(uint32_t, uint16_t);   // TODO declare as private
    uint8_t readWiiRemoteCalibration(void);

    bool buttonPressed(uint16_t/*eWiiRemoteButtons*/);
    bool buttonClicked(uint16_t/*eWiiRemoteButtons*/);

    Report_t Report;

  private:
    void initBTController(void);

    void HCI_task(void);
    void HCI_event_task(void);
    uint8_t hci_reset(void);
    uint8_t hci_inquiry(void);
    uint8_t hci_inquiry_cancel(void);
    uint8_t hci_connect(uint8_t *);
    uint8_t HCI_Command(uint16_t, uint8_t *);

    void L2CAP_task(void);
    void L2CAP_event_task(void);
    uint8_t l2cap_connect(uint16_t, uint16_t);
    uint8_t l2cap_configure(uint16_t);
    uint8_t l2cap_config_response(uint8_t, uint16_t);
    uint8_t l2cap_disconnect_response(uint8_t, uint16_t, uint16_t);
    uint8_t L2CAP_Command(uint8_t *, uint8_t);

    void readReport(uint8_t *);
    uint8_t writeReport(uint8_t *, uint8_t);
    void parseCalData(uint8_t *);
    void parseAccel(uint8_t *);
    void parseButtons(uint8_t *);

    Point3f_t averagePoint3(Point3i_t *, uint8_t);

    // endpoint record structure for the Bluetooth controller
    EP_RECORD ep_record_[ BT_NUM_EP ];

    /* variables used by high level HCI task */
    uint8_t hci_state_;     // current state of bluetooth HCI connection
    uint16_t hci_counter_;  // counter used for bluetooth HCI loops

    /* variables filled from HCI event management */
    uint8_t hci_event_flag_;    // flag of received bluetooth events
    uint8_t hci_handle_;

    /* variables used by L2CAP */
    uint8_t l2cap_state_;       // current state of L2CAP connection
    uint8_t l2cap_counter_;
    uint8_t l2cap_event_status_;
    uint8_t l2cap_txid_;        // packet id increments for each packet sent

    /* L2CAP CID name space: 0x0040-0xffff dynamically allocated */
    uint16_t command_scid_;     // Channel endpoint on command source
    uint16_t interrupt_scid_;   // Channel endpoint on interrupt source
    uint16_t command_dcid_;     // Channel endpoint on command destination
    uint16_t interrupt_dcid_;   // Channel endpoint on interrupt destination

    /* variables used by Bluetooth HID */
    uint8_t hid_flags_;
    uint16_t hid_buttons_;
    uint16_t old_hid_buttons_;
    uint16_t hid_buttons_click_;

    /* */
    Point3i_t Accel_Raw_[ACCEL_AVERAGE_SIZE];
    Point3Cal_t Accel_Cal_;

    uint8_t bdaddr_acquisition_mode_;
    uint8_t wiimote_bdaddr_[6];    // Bluetooth Address of Wii Remote

    uint8_t buf_[MAX_BUFFER_SIZE];
};

#endif  // _WIIREMOTE_H_


// vim: sts=4 sw=4 ts=4 et cin fdm=marker cms=//%s
