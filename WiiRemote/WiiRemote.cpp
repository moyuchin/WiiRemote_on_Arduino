/*
WiiRemote.cpp - WiiRemote Bluetooth stack on Arduino with USB Host Shield
Copyright (C) 2010 Tomo Tanaka

This program is based on <wiiblue.pde> which is developed by Oleg Mazurov. This
program also needs MAX3421E and USB libraries for Arduino written by Oleg. The
source codes can be grabbed from <https://github.com/felis/USB_Host_Shield>.


This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "WProgram.h"
#include <avr/pgmspace.h>

#include "WiiRemote.h"
#include "Max3421e.h"
#include "Usb.h"


#define ARRAY_LENGTH(array) (sizeof(array)/sizeof(*array))


#define WIIREMOTE_DEBUG 0
// {{{
#if WIIREMOTE_DEBUG
void Serial_print_P(const prog_char *str) {
    char c;

    if (!str) return;
    while ((c = pgm_read_byte(str++)) != 0) {
        Serial.print(c, BYTE);
    }
}

void Serial_println_P(const prog_char *str) {
    Serial_print_P(str);
    Serial.println("");
}

#define DEBUG_PRINT(c, f)   Serial.print(c, f)
#define DEBUG_PRINTLN(c, f) Serial.println(c, f)
#define DEBUG_PRINT_P(c)    Serial_print_P(c)
#define DEBUG_PRINTLN_P(c)  Serial_println_P(c)
#else
#define DEBUG_PRINT(c, f)
#define DEBUG_PRINTLN(c, f)
#define DEBUG_PRINT_P(c)
#define DEBUG_PRINTLN_P(c)
#endif
// }}}


MAX3421E Max;
USB Usb;


WiiRemote::WiiRemote(void) {
    l2cap_txid_ = 0;
    command_scid_ = 0x0040;
    interrupt_scid_ = 0x0041;

    hid_flags_ = 0;
    hid_buttons_ = 0;
    old_hid_buttons_ = 0;
    hid_buttons_click_ = 0;

    bdaddr_acquisition_mode_ = BD_ADDR_INQUIRY;
}

WiiRemote::~WiiRemote(void) {
}

void WiiRemote::init(void) {
    Max.powerOn();
    delay(200);
}

void WiiRemote::task(void (*pFunc)(void)) {
    Max.Task();
    Usb.Task();
    delay(1);

    // re-initialize
    if (Usb.getUsbTaskState() == USB_DETACHED_SUBSTATE_INITIALIZE) {
        /* TODO */
    }

    // wait for addressing state
    if (Usb.getUsbTaskState() == USB_STATE_CONFIGURING) {
        initBTController();

        hci_state_ = HCI_INIT_STATE;
        hci_counter_ = 10;
        l2cap_state_ = L2CAP_DOWN_STATE;

        Usb.setUsbTaskState(USB_STATE_RUNNING);
    }

    if (Usb.getUsbTaskState() == USB_STATE_RUNNING) {
        HCI_task();     // poll the HCI event pipe
        L2CAP_task();   // start polling the ACL input pipe too, though discard
                        // data until connected
    }
    if (l2cap_state_ == L2CAP_READY_STATE) {
        if (pFunc) { pFunc(); }
    }
} // task

void WiiRemote::setBDAddress(uint8_t *bdaddr, int size) {
    int array_length = ARRAY_LENGTH(wiimote_bdaddr_);

    for (int i = 0; i < size && i < array_length; i++) {
        wiimote_bdaddr_[i] = bdaddr[i];
    }
}

void WiiRemote::setBDAddressMode(eBDAddressMode mode) {
    bdaddr_acquisition_mode_ = mode;
}


/************************************************************/
/* Initialize Bluetooth USB Controller (CSR)                */
/************************************************************/
void WiiRemote::initBTController(void) {
    uint8_t rcode = 0;  // return code
    uint8_t buf[MAX_BUFFER_SIZE] = {0};
    USB_DEVICE_DESCRIPTOR *device_descriptor;

    /* initialize data structures for endpoints of device 1 */

    // copy endpoint 0 parameters
    ep_record_[ CONTROL_PIPE ] = *( Usb.getDevTableEntry(0,0) );

    // Bluetooth event endpoint
    ep_record_[ EVENT_PIPE ].epAddr = 0x01;
    ep_record_[ EVENT_PIPE ].Attr = EP_INTERRUPT;
    ep_record_[ EVENT_PIPE ].MaxPktSize = INT_MAXPKTSIZE;
    ep_record_[ EVENT_PIPE ].Interval = EP_POLL;
    ep_record_[ EVENT_PIPE ].sndToggle = bmSNDTOG0;
    ep_record_[ EVENT_PIPE ].rcvToggle = bmRCVTOG0;

    // Bluetoth data endpoint
    ep_record_[ DATAIN_PIPE ].epAddr = 0x02;
    ep_record_[ DATAIN_PIPE ].Attr = EP_BULK;
    ep_record_[ DATAIN_PIPE ].MaxPktSize = BULK_MAXPKTSIZE;
    ep_record_[ DATAIN_PIPE ].Interval = 0;
    ep_record_[ DATAIN_PIPE ].sndToggle = bmSNDTOG0;
    ep_record_[ DATAIN_PIPE ].rcvToggle = bmRCVTOG0;

    // Bluetooth data endpoint
    ep_record_[ DATAOUT_PIPE ].epAddr = 0x02;
    ep_record_[ DATAOUT_PIPE ].Attr = EP_BULK;
    ep_record_[ DATAOUT_PIPE ].MaxPktSize = BULK_MAXPKTSIZE;
    ep_record_[ DATAOUT_PIPE ].Interval = 0;
    ep_record_[ DATAOUT_PIPE ].sndToggle = bmSNDTOG0;
    ep_record_[ DATAOUT_PIPE ].rcvToggle = bmRCVTOG0;

    // plug kbd.endpoint parameters to devtable
    Usb.setDevTableEntry(BT_ADDR, ep_record_);

    // read the device descriptor and check VID and PID
    rcode = Usb.getDevDescr(BT_ADDR,
                            ep_record_[ CONTROL_PIPE ].epAddr,
                            DEV_DESCR_LEN,
                            (char *) buf);
    if (rcode) {
        DEBUG_PRINT_P( PSTR("\r\nDevice Descriptor Error: ") );
        DEBUG_PRINT(rcode, HEX);
        while(1);   // stop
    }

    device_descriptor = (USB_DEVICE_DESCRIPTOR *) &buf;
    if ((device_descriptor->idVendor != CSR_VID) ||
        (device_descriptor->idProduct != CSR_PID)) {
        DEBUG_PRINT_P( PSTR("\r\nWrong USB Device ID: ") );
        DEBUG_PRINT_P( PSTR("\r\n\t Vendor ID = ") );
        DEBUG_PRINT(device_descriptor->idVendor, HEX);
        DEBUG_PRINT_P( PSTR("\r\n\tProduct ID = ") );
        DEBUG_PRINT(device_descriptor->idProduct, HEX);
        while(1);   // stop
    }

    // configure device
    rcode = Usb.setConf(BT_ADDR,
                        ep_record_[ CONTROL_PIPE ].epAddr,
                        BT_CONFIGURATION);
    if (rcode) {
        DEBUG_PRINT_P( PSTR("\r\nDevice Configuration Error: ") );
        DEBUG_PRINT(rcode, HEX);
        while(1);   // stop
    }

    //LCD.clear();

    DEBUG_PRINT_P( PSTR("\r\nCSR Initialized") );
    //delay(200);
} // initBTController


/************************************************************/
/* HCI Flow Control                                         */
/************************************************************/
void WiiRemote::HCI_task(void) {
    HCI_event_task();

    switch (hci_state_) {
      case HCI_INIT_STATE:
        // wait until we have looped 10 times to clear any old events
        if (hci_timeout) {
            hci_reset();
            hci_state_ = HCI_RESET_STATE;
            hci_counter_ = 1000;
        }
        break;

      case HCI_RESET_STATE:
        if (hci_command_complete) {
            DEBUG_PRINT_P( PSTR("\r\nHCI Reset complete") );
            switch (bdaddr_acquisition_mode_) {
              case BD_ADDR_FIXED:
                hci_state_ = HCI_READY_CONNECT_STATE;
                hci_counter_ = 10000;
                break;

              case BD_ADDR_INQUIRY:
                hci_inquiry();
                hci_state_ = HCI_INQUIRY_STATE;
                hci_counter_ = 10000;
                break;

              default:
                break;
            }
        }
        if (hci_timeout) {
            DEBUG_PRINT_P( PSTR("\r\nNo response to HCI Reset") );
            hci_state_ = HCI_INIT_STATE;
            hci_counter_ = 10;
        }
        break;

      case HCI_INQUIRY_STATE:
        if (hci_inquiry_result) {
            DEBUG_PRINT_P( PSTR("\r\nHCI Inquiry responded") );
            hci_inquiry_cancel();
            hci_state_ = HCI_READY_CONNECT_STATE;
            hci_counter_ = 10000;
        }
        break;

      case HCI_READY_CONNECT_STATE:
        if (hci_command_complete) {
            if (hci_inquiry_result) {
                DEBUG_PRINT_P( PSTR("\r\nHCI Inquiry complete") );
            }
            hci_connect(wiimote_bdaddr_);   // connect to Wiimote
            hci_state_ = HCI_CONNECT_OUT_STATE;
            hci_counter_ = 10000;
        }
        break;

      case HCI_CONNECT_OUT_STATE:
        if (hci_connect_complete) {
            if(hci_connect_ok) {
                DEBUG_PRINT_P( PSTR("\r\nConnected to Wiimote") );
                hci_state_ = HCI_CONNECTED_STATE;
                l2cap_state_ = L2CAP_INIT_STATE;
            }
            else {
                hci_connect(wiimote_bdaddr_);   // try again to connect to Wiimote
                hci_counter_ = 10000;
            }
        }
        if (hci_timeout) {
            hci_connect(wiimote_bdaddr_);   // try again to connect to Wiimote
            hci_counter_ = 10000;
        }
        break;

      case HCI_CONNECTED_STATE:
        if (hci_disconn_complete) {
            DEBUG_PRINT_P( PSTR("\r\nWiimote Disconnected") );
            hci_state_ = HCI_INIT_STATE;
            hci_counter_ = 10;
            l2cap_state_ = L2CAP_DOWN_STATE;
        }
        break;

      default:
        break;
    }   // switch (hci_state_)

    return;
} // HCI_task

void WiiRemote::HCI_event_task(void) {
    uint8_t rcode = 0;  // return code
    uint8_t buf[MAX_BUFFER_SIZE] = {0};

    // check input on the event pipe (endpoint 1)
    rcode = Usb.inTransfer(BT_ADDR, ep_record_[ EVENT_PIPE ].epAddr,
                           MAX_BUFFER_SIZE, (char *) buf, USB_NAK_NOWAIT);
    if (!rcode) {
        /*  buf[0] = Event Code                            */
        /*  buf[1] = Parameter Total Length                */
        /*  buf[n] = Event Parameters based on each event  */
        DEBUG_PRINT_P( PSTR("\r\nHCI event = 0x") );
        DEBUG_PRINT(buf[0], HEX);
        switch (buf[0]) {   // switch on event type
          case HCI_EVENT_COMMAND_COMPLETE:
            hci_event_flag_ |= HCI_FLAG_COMMAND_COMPLETE;
            break;

          case HCI_EVENT_INQUIRY_RESULT:
            hci_event_flag_ |= HCI_FLAG_INQUIRY_RESULT;

            /* assume that Num_Responses is 1 */
            DEBUG_PRINT_P( PSTR("\r\nFound WiiRemote BD_ADDR:\t") );
            for (uint8_t i = 0; i < 6; i++) {
                wiimote_bdaddr_[5-i] = (uint8_t) buf[3+i];
                DEBUG_PRINT(wiimote_bdaddr_[5-i], HEX);
            }
            break;

          case HCI_EVENT_INQUIRY_COMPLETE:
            hci_event_flag_ |= HCI_FLAG_INQUIRY_COMPLETE;
            break;

          case HCI_EVENT_COMMAND_STATUS:
            hci_event_flag_ |= HCI_FLAG_COMMAND_STATUS;

#if WIIREMOTE_DEBUG
            if (buf[2]) {   // show status on serial if not OK
                DEBUG_PRINT_P( PSTR("\r\nHCI Command Failed: ") );
                DEBUG_PRINT_P( PSTR("\r\n\t             Status = ") );
                DEBUG_PRINT(buf[2], HEX);

                DEBUG_PRINT_P( PSTR("\r\n\tCommand_OpCode(OGF) = ") );
                DEBUG_PRINT( ((buf[5] & 0xFC) >> 2), HEX);

                DEBUG_PRINT_P( PSTR("\r\n\tCommand_OpCode(OCF) = ") );
                DEBUG_PRINT( (buf[5] & 0x03), HEX);
                DEBUG_PRINT(buf[4], HEX);
            }
#endif
            break;
          case HCI_EVENT_CONNECT_COMPLETE:
            hci_event_flag_ |= HCI_FLAG_CONNECT_COMPLETE;

            if (!buf[2]) {  // check if connected OK
                // store the handle for the ACL connection
                hci_handle_ = buf[3] | ((buf[4] & 0x0F) << 8);

                hci_event_flag_ |= HCI_FLAG_CONNECT_OK;
            }
            break;

          case HCI_EVENT_NUM_COMPLETED_PKT:
            break;

          case HCI_EVENT_QOS_SETUP_COMPLETE:
            break;

          case HCI_EVENT_DISCONN_COMPLETE:
            hci_event_flag_ |= HCI_FLAG_DISCONN_COMPLETE;
            break;

          default:
            DEBUG_PRINT_P( PSTR("\r\nUnmanaged Event: ") );
            DEBUG_PRINT(buf[0], HEX);
            break;
        }   // switch (buf[0])
    }
    return;
} // HCI_event_task

/************************************************************/
/* HCI Commands                                             */
/************************************************************/
uint8_t WiiRemote::hci_reset(void) {
    uint8_t buf[3] = {0};

    hci_event_flag_ = 0;    // clear all the flags

    buf[0] = HCI_OCF_RESET;
    buf[1] = HCI_OGF_CTRL_BBAND;
    buf[2] = 0x00; // Parameter Total Length = 0

    return HCI_Command(3, buf);
}

#if 0
uint8_t hci_read_bd_addr(void) {
    uint8_t buf[3] = {0};

    hci_event_flag_ &= ~HCI_FLAG_COMMAND_COMPLETE;

    buf[0] = HCI_OCF_READ_BD_ADDR;
    buf[1] = HCI_OGF_INFO_PARAM;
    buf[2] = 0x00;  // Parameter Total Length = 0

    return HCI_Command(3, buf);
}
#endif

uint8_t WiiRemote::hci_inquiry(void) {
    uint8_t buf[8] = {0};

    hci_event_flag_ &= ~(HCI_FLAG_INQUIRY_RESULT | HCI_FLAG_INQUIRY_COMPLETE);

    buf[0] = HCI_OCF_INQUIRY;
    buf[1] = HCI_OGF_LINK_CNTRL;
    buf[2] = 0x05;  // Parameter Total Length = 5
    buf[3] = 0x33;  // LAP: Genera/Unlimited Inquiry Access Code (GIAC = 0x9E8B33)
    buf[4] = 0x8B;
    buf[5] = 0x9E;
    buf[6] = 0x0A;  // Inquiry time
    buf[7] = 0x03;  // Max 1 response

    return HCI_Command(8, buf);
}

uint8_t WiiRemote::hci_inquiry_cancel(void) {
    uint8_t buf[3] = {0};

    hci_event_flag_ &= ~HCI_FLAG_COMMAND_COMPLETE;

    buf[0] = HCI_OCF_INQUIRY_CANCEL;
    buf[1] = HCI_OGF_LINK_CNTRL;
    buf[2] = 0x0;   // Parameter Total Length = 0

    return HCI_Command(3, buf);
}

uint8_t WiiRemote::hci_connect(uint8_t *bdaddr) {
    uint8_t buf[16] = {0};

    hci_event_flag_ &= ~(HCI_FLAG_CONNECT_COMPLETE | HCI_FLAG_CONNECT_OK);

    buf[0] = HCI_OCF_CREATE_CONNECTION;
    buf[1] = HCI_OGF_LINK_CNTRL;
    buf[2] = 0x0D;  // Parameter Total Length = 13
    buf[3] = *(bdaddr + 5); // 6 octet bluetooth address
    buf[4] = *(bdaddr + 4);
    buf[5] = *(bdaddr + 3);
    buf[6] = *(bdaddr + 2);
    buf[7] = *(bdaddr + 1);
    buf[8] = *bdaddr;
    buf[ 9] = 0x18; // DM1 or DH1 may be used
    buf[10] = 0xCC; // DM3, DH3, DM5, DH5 may be used
    buf[11] = 0x01; // page repetition mode R1
    buf[12] = 0x00; // always 0
    buf[13] = 0x00; // clock offset
    buf[14] = 0x00; // invalid clock offset
    buf[15] = 0x00; // do not allow role switch

    return HCI_Command(16, buf);
}

/* perform HCI Command */
uint8_t WiiRemote::HCI_Command(uint16_t nbytes, uint8_t *dataptr) {
    //hci_event_flag_ &= ~HCI_FLAG_COMMAND_COMPLETE;
    return Usb.ctrlReq(BT_ADDR,
                       ep_record_[ CONTROL_PIPE ].epAddr,
                       bmREQ_HCI_OUT,
                       HCI_COMMAND_REQ,
                       0x00,
                       0x00,
                       0,
                       nbytes,
                       (char *) dataptr);
}


/************************************************************/
/* L2CAP Flow Control                                       */
/************************************************************/
void WiiRemote::L2CAP_task(void) {
    L2CAP_event_task();

    switch (l2cap_state_) {

      case L2CAP_DOWN_STATE:
        break;

      case L2CAP_INIT_STATE:
        l2cap_event_status_ = 0;
        l2cap_connect(command_scid_, L2CAP_PSM_WRITE);
        l2cap_state_ = L2CAP_CONTROL_CONNECTING_STATE;
        break;

      case L2CAP_CONTROL_CONNECTING_STATE:
        if (l2cap_command_connected) {
            l2cap_event_status_ &= ~L2CAP_EV_COMMAND_CONFIGURED;
            l2cap_configure(command_dcid_);
            l2cap_state_ = L2CAP_CONTROL_CONFIGURING_STATE;
      }
        break;

      case L2CAP_CONTROL_CONFIGURING_STATE:
        if (l2cap_command_configured) {
            l2cap_event_status_ &= ~L2CAP_EV_INTERRUPT_CONNECTED;
            l2cap_connect(interrupt_scid_, L2CAP_PSM_READ);
            l2cap_state_ = L2CAP_INTERRUPT_CONNECTING_STATE;
        }
        break;

      case L2CAP_INTERRUPT_CONNECTING_STATE:
        if (l2cap_interrupt_connected) {
            l2cap_event_status_ &= ~L2CAP_EV_INTERRUPT_CONFIGURED;
            l2cap_configure(interrupt_dcid_);
            l2cap_state_ = L2CAP_INTERRUPT_CONFIGURING_STATE;
        }
        break;

      case L2CAP_INTERRUPT_CONFIGURING_STATE:
        if (l2cap_interrupt_configured) {
            l2cap_state_ = L2CAP_CONNECTED_STATE;
        }
        break;

    /* Established L2CAP */

      case L2CAP_CONNECTED_STATE:
        hid_flags_ = 0;
        setLED(WIIREMOTE_LED1);
        //delay(500);
        l2cap_state_ = L2CAP_WIIREMOTE_LED_STATE;
        break;

      case L2CAP_WIIREMOTE_LED_STATE:
        if (hid_command_success) {
            readWiiRemoteCalibration();
            l2cap_state_ = L2CAP_WIIREMOTE_CAL_STATE;
        }
        break;

/*
      case L2CAP_CALIBRATION_STATE:
        if (hid_read_calibration) {
            setLED(WIIREMOTE_LED2);
            delay(600);
            l2cap_state_ = L2CAP_LED2_STATE;
        }
        break;

      case L2CAP_LED2_STATE:
        if (hid_command_success) {
            setLED(WIIREMOTE_LED3);
            delay(600);
            l2cap_state_ = L2CAP_LED3_STATE;
        }
        break;
*/

      case L2CAP_WIIREMOTE_CAL_STATE:
        if (hid_command_success) {
            setReportMode(INPUT_REPORT_IR_EXT_ACCEL);
            l2cap_state_ = L2CAP_READY_STATE;
        }
        break;
/*
      case L2CAP_REPORT_MODE_STATE:
        if (hid_command_success) {
            setLED(WIIREMOTE_LED4);
            l2cap_state_ = L2CAP_READY_STATE;
        }
        break;

      case L2CAP_LED4_STATE:
        if (hid_command_success) {
            l2cap_state_ = L2CAP_READY_STATE;
        }
        break;
*/
      case L2CAP_READY_STATE:
        // a status report will require reporting to be restarted
        if (hid_status_reported) {
            setReportMode(INPUT_REPORT_IR_EXT_ACCEL);
        }
        if (l2cap_interrupt_disconnected || l2cap_command_disconnected) {
            l2cap_state_ = L2CAP_DISCONNECT_STATE;
        }
        break;

      case L2CAP_DISCONNECT_STATE:
        break;

      default:
        break;
    }
    return;
} // L2CAP_task

void WiiRemote::L2CAP_event_task(void) {
    uint8_t rcode = 0;  // return code
    uint8_t buf[MAX_BUFFER_SIZE] = {0};

    // check input on the event pipe (endpoint 2)
    rcode = Usb.inTransfer(BT_ADDR, ep_record_[ DATAIN_PIPE ].epAddr,
                           MAX_BUFFER_SIZE, (char *) buf, USB_NAK_NOWAIT);
    if (!rcode) {
        if (acl_handle_ok) {
            if (l2cap_control) {
                DEBUG_PRINT_P( PSTR("\r\nL2CAP Signaling Command = 0x") );
                DEBUG_PRINT(buf[8], HEX);
                if (l2cap_connection_response) {
                    if (l2cap_connection_success) {
                        if ((buf[14] | (buf[15] << 8)) == command_scid_) {
                            command_dcid_ =  buf[12] | (buf[13] << 8);
                            l2cap_event_status_ |= L2CAP_EV_COMMAND_CONNECTED;
                        }
                        else if ((buf[14] | (buf[15] << 8)) == interrupt_scid_) {
                            interrupt_dcid_ =  buf[12] | ( buf[13] << 8);
                            l2cap_event_status_ |= L2CAP_EV_INTERRUPT_CONNECTED;
                        }
                    } // l2cap_connection_success
                } // l2cap_connection_response
                else if (l2cap_configuration_response) {
                    /* TODO l2cap_configuration_success */
                    if ((buf[12] | (buf[13] << 8)) == command_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_COMMAND_CONFIGURED;
                    }
                    else if ((buf[12] | (buf[13] << 8)) == interrupt_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_INTERRUPT_CONFIGURED;
                    }
                }
                else if (l2cap_configuration_request) {
                    if ((buf[12] | (buf[13] << 8)) == command_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_COMMAND_CONFIG_REQ;
                        l2cap_config_response(buf[9], command_dcid_);
                    }
                    else if ((buf[12] | (buf[13] << 8)) == interrupt_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_INTERRUPT_CONFIG_REQ;
                        l2cap_config_response(buf[9], interrupt_dcid_);
                    }
                }
                else if (l2cap_disconnect_request) {
                    if ((buf[12] | (buf[13] << 8)) == command_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_COMMAND_DISCONNECT_REQ;
                        l2cap_disconnect_response(buf[9], command_scid_, command_dcid_);
                    }
                    else if ((buf[12] | (buf[13] << 8)) == interrupt_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_INTERRUPT_DISCONNECT_REQ;
                        l2cap_disconnect_response(buf[9], command_scid_, command_dcid_);
                    }
                }
            } // l2cap_control
            else if (l2cap_interrupt) {
                readReport(buf);
            } // l2cap_interrupt
            else if (l2cap_command){
                if (hid_handshake_success) {
                    hid_flags_ |= HID_FLAG_COMMAND_SUCCESS;
                }
            } // l2cap_command
        } // acl_handle_ok
    } // !rcode
    return;
} // L2CAP_event_task

/************************************************************/
/* L2CAP Commands                                           */
/************************************************************/
uint8_t WiiRemote::l2cap_connect(uint16_t scid, uint16_t psm) {
    uint8_t cmd_buf[8];
    cmd_buf[0] = L2CAP_CMD_CONNECTION_REQUEST;  // Code
    cmd_buf[1] = (uint8_t) (l2cap_txid_++);     // Identifier
    cmd_buf[2] = 0x04;                          // Length
    cmd_buf[3] = 0x00;
    cmd_buf[4] = (uint8_t) (psm & 0xff);        // PSM
    cmd_buf[5] = (uint8_t) (psm >> 8);
    cmd_buf[6] = (uint8_t) (scid & 0xff);       // Source CID
    cmd_buf[7] = (uint8_t) (scid >> 8);

    return L2CAP_Command((uint8_t *) cmd_buf, 8);
}

uint8_t WiiRemote::l2cap_configure(uint16_t dcid) {
    uint8_t cmd_buf[12];
    cmd_buf[0] = L2CAP_CMD_CONFIG_REQUEST;  // Code
    cmd_buf[1] = (uint8_t) (l2cap_txid_++); // Identifier
    cmd_buf[2] = 0x08;                      // Length
    cmd_buf[3] = 0x00;
    cmd_buf[4] = (uint8_t) (dcid & 0xff);   // Destination CID
    cmd_buf[5] = (uint8_t) (dcid >> 8);
    cmd_buf[6] = 0x00;                      // Flags
    cmd_buf[7] = 0x00;
    cmd_buf[8] = 0x01;  // Config Opt: type = MTU (Maximum Transmission Unit)
    cmd_buf[9] = 0x02;  // Config Opt: length
    cmd_buf[10] = 0xa0; // Config Opt: data = masimum SDU size is 672 octets
    cmd_buf[11] = 0x02;

    return L2CAP_Command((uint8_t *) cmd_buf, 12);
}

uint8_t WiiRemote::l2cap_config_response(
        uint8_t rxid,
        uint16_t dcid) {
    uint8_t resp_buf[10];
    resp_buf[0] = L2CAP_CMD_CONFIG_RESPONSE;    // Code
    resp_buf[1] = rxid;                         // Identifier
    resp_buf[2] = 0x06;                         // Length
    resp_buf[3] = 0x00;
    resp_buf[4] = (uint8_t) (dcid & 0xff);      // Source CID
    resp_buf[5] = (uint8_t) (dcid >> 8);
    resp_buf[6] = 0x00;                         // Result
    resp_buf[7] = 0x00;
    resp_buf[8] = 0x00;                         // Config
    resp_buf[9] = 0x00;

    return L2CAP_Command((uint8_t *) resp_buf, 10);
}

uint8_t WiiRemote::l2cap_disconnect_response(
        uint8_t rxid,
        uint16_t scid,
        uint16_t dcid) {
    uint8_t resp_buf[8];
    resp_buf[0] = L2CAP_CMD_DISCONNECT_RESPONSE;    // Code
    resp_buf[1] = rxid;                             // Identifier
    resp_buf[2] = 0x04;                             // Length
    resp_buf[3] = 0x00;
    resp_buf[4] = (uint8_t) (dcid & 0xff);          // Destination CID
    resp_buf[5] = (uint8_t) (dcid >> 8);
    resp_buf[6] = (uint8_t) (scid & 0xff);          // Source CID
    resp_buf[7] = (uint8_t) (scid >> 8);

    return L2CAP_Command((uint8_t *) resp_buf, 8);
}

uint8_t WiiRemote::L2CAP_Command(uint8_t *data, uint8_t length) {
    uint8_t buf[MAX_BUFFER_SIZE] = {0};
    buf[0] = (uint8_t) (hci_handle_ & 0xff);    // HCI handle with PB,BC flag
    buf[1] = (uint8_t) (((hci_handle_ >> 8) & 0x0f) | 0x20);
    buf[2] = (uint8_t) ((4 + length) & 0xff);   // HCI ACL total data length
    buf[3] = (uint8_t) ((4 + length) >> 8);
    buf[4] = (uint8_t) (length & 0xff);         // L2CAP header: Length
    buf[5] = (uint8_t) (length >> 8);
    buf[6] = 0x01;  // L2CAP header: Channel ID
    buf[7] = 0x00;  // L2CAP Signalling channel over ACL-U logical link
    for (uint8_t i = 0; i < length; i++) {      // L2CAP C-frame
        buf[8+i] = *data;
        data++;
    }

    // output on endpoint 2
    return Usb.outTransfer(BT_ADDR,
                           ep_record_[ DATAOUT_PIPE ].epAddr,
                           (8 + length),
                           (char *) buf);
}


/************************************************************/
/* HID Report (HCI ACL Packet)                              */
/************************************************************/
void WiiRemote::readReport(uint8_t *data) {
    if (data[8] == HID_THDR_DATA_INPUT) {
        switch (data[9]) {

          case INPUT_REPORT_STATUS:
            hid_flags_ |= HID_FLAG_STATUS_REPORTED;
            break;

          case INPUT_REPORT_READ_DATA:
            /* (a1) 21 BB BB SE FF FF DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD   */
            /*  BB BB is the state of the buttons on the Wiimote                        */
            /*  E (low nybble of SE) is the error flag. Error value 0 for no error.     */
            /*  S (high nybble of SE) is the size in bytes, minus one.                  */
            /*  FF FF is the offset expressed in abs. memory address of 1st byte.       */
            if ((data[12] & 0x0f) == 0) {
                if ((data[13] == 0x00) && (data[14] == 0x16)) {
                    hid_flags_ |= HID_FLAG_READ_CALIBRATION;
                    parseCalData(data);
                }
            }
            break;

          case INPUT_REPORT_IR_EXT_ACCEL:
            /* (a1) 37 BB BB AA AA AA II II II II II II II II II II EE EE EE EE EE EE   */
            old_hid_buttons_ = hid_buttons_;
            hid_buttons_ = (data[10] & 0x9f) | (data[11] & 0x9f) << 8;
            if (hid_buttons_ != old_hid_buttons_) {
                hid_flags_ |= HID_FLAG_BUTTONS_CHANGED;
            } else {
                hid_flags_ &= ~HID_FLAG_BUTTONS_CHANGED;
            }
            hid_buttons_click_ = ~hid_buttons_ & old_hid_buttons_;

            parseAccel(data);
            //parseButtons(data);   /* buttonPressed() can be used */
            break;

          default:
            DEBUG_PRINT_P( PSTR("\r\nUnmanaged Input Report: ") );
            DEBUG_PRINT(data[9], HEX);
            break;
        }
    }
} // readReport

uint8_t WiiRemote::writeReport(uint8_t *data, uint8_t length) {
    uint8_t buf[MAX_BUFFER_SIZE] = {0};

    buf[0] = (uint8_t) (hci_handle_ & 0xff);    // HCI handle with PB,BC flag
    buf[1] = (uint8_t) (((hci_handle_ >> 8) & 0x0f) | 0x20);
    buf[2] = (uint8_t) ((5 + length) & 0xff);   // HCI ACL total data length
    buf[3] = (uint8_t) ((5 + length) >> 8);
    buf[4] = (uint8_t) ((1 + length) & 0xff);   // L2CAP header: Length
    buf[5] = (uint8_t) ((1 + length) >> 8);
    buf[6] = (uint8_t) (command_dcid_ & 0xff);   // L2CAP header: Channel ID
    buf[7] = (uint8_t) (command_dcid_ >> 8);
    buf[8] = HID_THDR_SET_REPORT_OUTPUT;        // L2CAP B-frame
    for (uint8_t i = 0; i < length; i++) {
        buf[9+i] = *data;
        data++;
    }

    hid_flags_ &= ~HID_FLAG_COMMAND_SUCCESS;
    // output on endpoint 2
    return Usb.outTransfer(BT_ADDR,
                           ep_record_[ DATAOUT_PIPE ].epAddr,
                           (9 + length),
                           (char *) buf);
} // writeReport

void WiiRemote::parseCalData(uint8_t *data) {
    //uint8_t nbytes = ((data[12] & 0xf0) >> 4) + 1;
    Accel_Cal_.offset.X = (data[15] << 2) | (data[18] & 0x30) >> 4;
    Accel_Cal_.offset.Y = (data[16] << 2) | (data[18] & 0x0c) >> 2;
    Accel_Cal_.offset.Z = (data[17] << 2) | (data[18] & 0x03);
    Accel_Cal_.gravity.X = (data[19] << 2) | (data[22] & 0x30) >> 4;
    Accel_Cal_.gravity.Y = (data[20] << 2) | (data[22] & 0x0c) >> 2;
    Accel_Cal_.gravity.Z = (data[21] << 2) | (data[22] & 0x03);

    DEBUG_PRINT_P( PSTR("\r\nCalibration Data") );
    DEBUG_PRINT_P( PSTR("\r\n\tX0 = ") ); DEBUG_PRINT(Accel_Cal_.offset.X, HEX);
    DEBUG_PRINT_P( PSTR("\r\n\tY0 = ") ); DEBUG_PRINT(Accel_Cal_.offset.Y, HEX);
    DEBUG_PRINT_P( PSTR("\r\n\tZ0 = ") ); DEBUG_PRINT(Accel_Cal_.offset.Z, HEX);
    DEBUG_PRINT_P( PSTR("\r\n\tXG = ") ); DEBUG_PRINT(Accel_Cal_.gravity.X, HEX);
    DEBUG_PRINT_P( PSTR("\r\n\tYG = ") ); DEBUG_PRINT(Accel_Cal_.gravity.Y, HEX);
    DEBUG_PRINT_P( PSTR("\r\n\tZG = ") ); DEBUG_PRINT(Accel_Cal_.gravity.Z, HEX);
}

void WiiRemote::parseAccel(uint8_t *data) {
    static uint8_t accel_cnt;
    uint8_t i = accel_cnt++ % ACCEL_AVERAGE_SIZE;
    Point3f_t accel_avg;

    /* In all Data Reporting Modes which include Accelerometer data except  */
    /* for mode 0x3e/0x3f, the accelerometer data is reported as three      */
    /* consecutive bytes XX, YY and ZZ:                                     */
    /* (a1) RR BB BB XX YY ZZ [...]                                         */
    Accel_Raw_[i].X = (data[12] << 2) | ((data[10] & 0x60) >> 5);
    Accel_Raw_[i].Y = (data[13] << 2) | ((data[11] & 0x20) >> 4);
    Accel_Raw_[i].Z = (data[14] << 2) | ((data[11] & 0x40) >> 5);

    if (accel_cnt >= ACCEL_AVERAGE_SIZE) {
        accel_avg = averagePoint3(Accel_Raw_, ACCEL_AVERAGE_SIZE);
    } else {
        accel_avg.X = (float) Accel_Cal_.offset.X;
        accel_avg.Y = (float) Accel_Cal_.offset.Y;
        accel_avg.Z = (float) Accel_Cal_.offset.Z;
    }

    Report.Accel.X = (accel_avg.X - (float) Accel_Cal_.offset.X) /
                     (float) (Accel_Cal_.gravity.X - Accel_Cal_.offset.X);
    Report.Accel.Y = (accel_avg.Y - (float) Accel_Cal_.offset.Y) /
                     (float) (Accel_Cal_.gravity.Y - Accel_Cal_.offset.Y);
    Report.Accel.Z = (accel_avg.Z - (float) Accel_Cal_.offset.Z) /
                     (float) (Accel_Cal_.gravity.Z - Accel_Cal_.offset.Z);

    DEBUG_PRINT_P( PSTR("\r\nparseAccel X=") );
    DEBUG_PRINT(Report.Accel.X, 2);
    DEBUG_PRINT_P( PSTR(" Y=") );
    DEBUG_PRINT(Report.Accel.Y, 2);
    DEBUG_PRINT_P( PSTR(" Z=") );
    DEBUG_PRINT(Report.Accel.Z, 2);
}

void WiiRemote::parseButtons(uint8_t *data) {
    uint16_t buttons = (data[10] & 0x9f) | (data[11] & 0x9f) << 8;
    Report.Button.Left  = ((buttons & WIIREMOTE_LEFT) != 0);
    Report.Button.Right = ((buttons & WIIREMOTE_RIGHT) != 0);
    Report.Button.Down  = ((buttons & WIIREMOTE_DOWN) != 0);
    Report.Button.Up    = ((buttons & WIIREMOTE_UP) != 0);
    Report.Button.Plus  = ((buttons & WIIREMOTE_PLUS) != 0);
    Report.Button.Two   = ((buttons & WIIREMOTE_TWO) != 0);
    Report.Button.One   = ((buttons & WIIREMOTE_ONE) != 0);
    Report.Button.B     = ((buttons & WIIREMOTE_B) != 0);
    Report.Button.A     = ((buttons & WIIREMOTE_A) != 0);
    Report.Button.Minus = ((buttons & WIIREMOTE_MINUS) != 0);
    Report.Button.Home  = ((buttons & WIIREMOTE_HOME) != 0);
}


/************************************************************/
/* WiiRemote HID Command (primitive)                        */
/************************************************************/
uint8_t WiiRemote::setLED(uint8_t led) {
    uint8_t hid_buf[2];
    hid_buf[0] = OUTPUT_REPORT_LED;
    hid_buf[1] = led;

    //hid_flags_ &= ~HID_FLAG_COMMAND_SUCCESS;
    return writeReport((uint8_t *) hid_buf, 2);
}

uint8_t WiiRemote::setReportMode(uint8_t mode) {
    uint8_t hid_buf[3];
    hid_buf[0] = OUTPUT_REPORT_MODE;
    hid_buf[1] = 0x00;  // Wiimote will only send an report when the data has changed
    hid_buf[2] = mode;

    //hid_flags_ &= ~HID_FLAG_COMMAND_SUCCESS;
    hid_flags_ &= ~HID_FLAG_STATUS_REPORTED;
    return writeReport((uint8_t *) hid_buf, 3);
}

uint8_t WiiRemote::readData(uint32_t offset, uint16_t size) {
    uint8_t hid_buf[7];
    hid_buf[0] = OUTPUT_REPORT_READ_DATA;
    hid_buf[1] = (uint8_t) ((offset & 0xff000000) >> 24);   /* TODO involve Rumble flag */
    hid_buf[2] = (uint8_t) ((offset & 0x00ff0000) >> 16);
    hid_buf[3] = (uint8_t) ((offset & 0x0000ff00) >> 8);
    hid_buf[4] = (uint8_t) ((offset & 0x000000ff));
    hid_buf[5] = (uint8_t) ((size & 0xff00) >> 8);
    hid_buf[6] = (uint8_t) ((size & 0x00ff));

    //hid_flags_ &= ~HID_FLAG_COMMAND_SUCCESS;
    hid_flags_ &= ~HID_FLAG_READ_CALIBRATION;
    return writeReport((uint8_t *) hid_buf, 7);
}

/************************************************************/
/* WiiRemote Command                                        */
/************************************************************/
uint8_t WiiRemote::readWiiRemoteCalibration(void) {
    return readData(0x0016, 8);
}

bool WiiRemote::buttonPressed(uint16_t button) {
    /* true while a button is pressed */
    return ((hid_buttons_ & button) != 0);
}

bool WiiRemote::buttonClicked(uint16_t button) {
    /* true when a button is clicked */
    bool click = ((hid_buttons_click_ & button) != 0);
    hid_buttons_click_ &= ~button;  // clear "click" event
    return click;
}


/************************************************************/
/* etc                                                      */
/************************************************************/
Point3f_t WiiRemote::averagePoint3(Point3i_t *data, uint8_t size) {
    Point3f_t sum = {0.0, 0.0, 0.0};

    for (uint8_t i = 0; i < size; i++) {
        sum.X += (float) data->X;
        sum.Y += (float) data->Y;
        sum.Z += (float) data->Z;
        data++;
    }
    sum.X /= size;
    sum.Y /= size;
    sum.Z /= size;

    return sum;
}


// vim: sts=4 sw=4 ts=4 et cin fdm=marker cms=//%s syntax=arduino
