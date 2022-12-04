#include <Arduino.h>
#include <esp_bt_main.h>

#include "esp32-hal-bt.h"
#include "osi/allocator.h"
#include "stack/l2c_api.h"
extern "C" {
#include "stack/btm_api.h"
}
#include "hid_l2cap.h"

#define HID_L2CAP_ID_HIDC 0x40
#define HID_L2CAP_ID_HIDI 0x41

static long hid_l2cap_init_service(const char *name, uint16_t psm,
                                   uint8_t security_id);
static void hid_l2cap_deinit_service(const char *name, uint16_t psm);

// tL2CA_CONNECT_CFM_CB hid_l2cap_connect_cfm_cback;
static void hid_l2cap_connect_cfm_cback(uint16_t l2cap_cid, uint16_t result);
static void hid_l2cap_config_ind_cback(uint16_t l2cap_cid,
                                       tL2CAP_CFG_INFO *p_cfg);
static void hid_l2cap_config_cfm_cback(uint16_t l2cap_cid,
                                       tL2CAP_CFG_INFO *p_cfg);
static void hid_l2cap_disconnect_ind_cback(uint16_t l2cap_cid, bool ack_needed);
static void hid_l2cap_disconnect_cfm_cback(uint16_t l2cap_cid, uint16_t result);
static void hid_l2cap_data_ind_cback(uint16_t l2cap_cid, BT_HDR *p_msg);

static void dump_bin(const char *p_message, const uint8_t *p_bin, int len);

static BT_STATUS is_connected = BT_UNINITIALIZED;
static BD_ADDR g_bd_addr;
static HID_L2CAP_CALLBACK g_callback;

static uint16_t l2cap_cid_hidc;
static uint16_t l2cap_cid_hidi;

static tL2CAP_ERTM_INFO hid_ertm_info;
static tL2CAP_CFG_INFO hid_cfg_info;

// static tL2CA_CONNECT_IND_CB hid_l2cap_connect_ind_cb;
void hid_l2cap_connect_ind_cb(uint8_t *address, uint16_t localCid, uint16_t psm,
                              uint8_t identifier) {
  /* Send connection pending response to the L2CAP layer. */
  L2CA_CONNECT_RSP(address, identifier, localCid, L2CAP_CONN_PENDING,
                   L2CAP_CONN_PENDING, NULL, NULL);

  /* Send response to the L2CAP layer. */
  L2CA_CONNECT_RSP(address, identifier, localCid, L2CAP_CONN_OK, L2CAP_CONN_OK,
                   NULL, NULL);

  /* Send a Configuration Request. */
  L2CA_CONFIG_REQ(localCid, &hid_cfg_info);

  if (psm == BT_PSM_HIDC) {
    l2cap_cid_hidc = localCid;
  } else if (psm == BT_PSM_HIDI) {
    l2cap_cid_hidi = localCid;
  }

  Serial.println("hid_l2cap_connect_ind_cb");
}

// tL2CA_CONNECT_PND_CB;
void hid_l2cap_connect_pnd_cb(uint16_t cid) {
  Serial.println("pending connection cid: " + String(cid));
}

// tL2CA_QOS_VIOLATION_IND_CB;
void hid_l2cap_qos_violation_ind_cb(uint8_t *address) {
  Serial.println("hid violation");
}

// tL2CA_CONGESTION_STATUS_CB;
void hid_l2cap_congestion_status_cb(uint16_t cid, bool congested) {
  Serial.println("conguestion status cb cid: " + String(cid) +
                 " conguested: " + String(congested));
}

// tL2CA_TX_COMPLETE_CB;
void hid_l2cap_tx_complete_cb(uint16_t cid, uint16_t numberOfSdu) {
  Serial.println("tx complete cid:" + String(cid) +
                 " number of sdu: " + String(numberOfSdu));
}

static const tL2CAP_APPL_INFO dyn_info = {
    hid_l2cap_connect_ind_cb,       hid_l2cap_connect_cfm_cback,
    hid_l2cap_connect_pnd_cb,       hid_l2cap_config_ind_cback,
    hid_l2cap_config_cfm_cback,     hid_l2cap_disconnect_ind_cback,
    hid_l2cap_disconnect_cfm_cback, hid_l2cap_qos_violation_ind_cb,
    hid_l2cap_data_ind_cback,       hid_l2cap_congestion_status_cb,
    hid_l2cap_tx_complete_cb};

static long hid_l2cap_init_services(void) {
  long ret;
  ret =
      hid_l2cap_init_service("HIDC", BT_PSM_HIDC, BTM_SEC_SERVICE_FIRST_EMPTY);
  if (ret != 0) return ret;
  ret = hid_l2cap_init_service("HIDI", BT_PSM_HIDI,
                               BTM_SEC_SERVICE_FIRST_EMPTY + 1);
  if (ret != 0) return ret;

  return 0;
}

static void hid_l2cap_deinit_services(void) {
  hid_l2cap_deinit_service("HIDC", BT_PSM_HIDC);
  hid_l2cap_deinit_service("HIDI", BT_PSM_HIDI);
}

static long hid_l2cap_init_service(const char *name, uint16_t psm,
                                   uint8_t security_id) {
  /* Register the PSM for incoming connections */
  if (!L2CA_Register(psm, (tL2CAP_APPL_INFO *)&dyn_info)) {
    Serial.printf("%s Registering service %s failed\n", __func__, name);
    return -1;
  }

  /* Register with the Security Manager for our specific security level (none)
   */
  if (!BTM_SetSecurityLevel(false, name, security_id, 0, psm, 0, 0)) {
    Serial.printf("%s Registering security service %s failed\n", __func__,
                  name);
    return -1;
  }

  Serial.printf("[%s] Service %s Initialized\n", __func__, name);

  return 0;
}

static void hid_l2cap_deinit_service(const char *name, uint16_t psm) {
  L2CA_Deregister(psm);
  Serial.printf("[%s] Service %s Deinitialized\n", __func__, name);
}

BT_STATUS hid_l2cap_is_connected(void) { return is_connected; }

long hid_l2cap_reconnect(void) {
  long ret;
  ret = L2CA_CONNECT_REQ(BT_PSM_HIDC, g_bd_addr, NULL, NULL);
  Serial.printf("L2CA_CONNECT_REQ ret=%d\n", ret);
  if (ret == 0) {
    return -1;
  }
  l2cap_cid_hidc = ret;

  is_connected = BT_CONNECTING;

  return ret;
}

long hid_l2cap_connect(BD_ADDR addr) {
  memmove(g_bd_addr, addr, sizeof(BD_ADDR));

  return hid_l2cap_reconnect();
}

long hid_l2cap_initialize(HID_L2CAP_CALLBACK callback) {
  if (!btStarted() && !btStart()) {
    Serial.println("btStart failed");
    return -1;
  }

  esp_bluedroid_status_t bt_state = esp_bluedroid_get_status();
  if (bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
    if (esp_bluedroid_init()) {
      Serial.println("esp_bluedroid_init failed");
      return -1;
    }
  }

  if (bt_state != ESP_BLUEDROID_STATUS_ENABLED) {
    if (esp_bluedroid_enable()) {
      Serial.println("esp_bluedroid_enable failed");
      return -1;
    }
  }

  if (hid_l2cap_init_services() != 0) {
    Serial.println("hid_l2cap_init_services failed");
    return -1;
  }

  g_callback = callback;

  is_connected = BT_DISCONNECTED;

  return 0;
}

static void hid_l2cap_connect_cfm_cback(uint16_t l2cap_cid, uint16_t result) {
  Serial.printf("[%s] l2cap_cid: 0x%02x\n  result: %d\n", __func__, l2cap_cid,
                result);
}

static void hid_l2cap_config_cfm_cback(uint16_t l2cap_cid,
                                       tL2CAP_CFG_INFO *p_cfg) {
  Serial.printf("[%s] l2cap_cid: 0x%02x\n  p_cfg->result: %d\n", __func__,
                l2cap_cid, p_cfg->result);

  if (l2cap_cid == l2cap_cid_hidc) {
  } else if (l2cap_cid == l2cap_cid_hidi) {
    is_connected = BT_CONNECTED;

    Serial.println("Hid Connected");
  }
}

static void hid_l2cap_config_ind_cback(uint16_t l2cap_cid,
                                       tL2CAP_CFG_INFO *p_cfg) {
  Serial.printf(
      "[%s] l2cap_cid: 0x%02x\n  p_cfg->result: %d\n  p_cfg->mtu_present: %d\n "
      " p_cfg->mtu: %d\n",
      __func__, l2cap_cid, p_cfg->result, p_cfg->mtu_present, p_cfg->mtu);

  p_cfg->result = L2CAP_CFG_OK;

  L2CA_ConfigRsp(l2cap_cid, p_cfg);
}

static void hid_l2cap_disconnect_ind_cback(uint16_t l2cap_cid,
                                           bool ack_needed) {
  Serial.printf("[%s] l2cap_cid: 0x%02x\n  ack_needed: %d\n", __func__,
                l2cap_cid, ack_needed);
  is_connected = BT_DISCONNECTED;
  g_callback = NULL;
}

static void hid_l2cap_disconnect_cfm_cback(uint16_t l2cap_cid,
                                           uint16_t result) {
  Serial.printf("[%s] l2cap_cid: 0x%02x\n  result: %d\n", __func__, l2cap_cid,
                result);
}

unsigned long printedAt = 0;
static void hid_l2cap_data_ind_cback(uint16_t l2cap_cid, BT_HDR *p_buf) {
  if (millis() - printedAt > 500UL) {
    printedAt = millis();
    Serial.printf("[%s] l2cap_cid: 0x%02x\n", __func__, l2cap_cid);
    Serial.printf("event=%d len=%d offset=%d layer_specific=%d\n", p_buf->event,
                  p_buf->len, p_buf->offset, p_buf->layer_specific);
    dump_bin("\tdata=", &p_buf->data[p_buf->offset], p_buf->len);

    if (g_callback != NULL) g_callback(&p_buf->data[p_buf->offset], p_buf->len);
  }

  osi_free(p_buf);
}

static void dump_bin(const char *p_message, const uint8_t *p_bin, int len) {
  Serial.printf("%s", p_message);
  for (int i = 0; i < len; i++) {
    Serial.printf("%02x ", p_bin[i]);
  }
  Serial.printf("\n");
}
