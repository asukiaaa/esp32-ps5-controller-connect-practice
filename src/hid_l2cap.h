#ifndef _HID_L2CAP_H_
#define _HID_L2CAP_H_

#include "stack/bt_types.h"

enum BT_STATUS {
  BT_UNINITIALIZED,
  BT_DISCONNECTED,
  BT_CONNECTING,
  BT_CONNECTED
};

#define HID_L2CAP_MESSAGE_SIZE  8
typedef void (*HID_L2CAP_CALLBACK)(uint8_t *p_msg, size_t length); 

long hid_l2cap_initialize(HID_L2CAP_CALLBACK callback);
long hid_l2cap_connect(BD_ADDR addr);
long hid_l2cap_reconnect(void);
BT_STATUS hid_l2cap_is_connected(void);

#endif
