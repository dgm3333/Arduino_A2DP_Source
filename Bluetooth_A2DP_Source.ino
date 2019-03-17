/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
/* 
 *  Arduino port of https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/a2dp_source
 *  
 *  After connection with A2DP sink is established, the example performs the following running loop 1-2-3-4-1:
 *    1. audio transmission starts and lasts for a while
 *    2. audio transmission stops
 *    3. disconnect with target device
 *    4. reconnect to target device
 *    
 *    The example implements an event loop triggered by a periodic "heart beat" timer and events from Bluetooth protocol stack callback functions.
 *    
 *    For current stage, the supported audio codec in ESP32 A2DP is SBC (SubBand Coding). 
 *    SBC specification is in Appendix B (page 50) of the document A2DP_Spec_V1_0 (can be found with search engine, although the original is behind the Bluetooth firewall)
 *    
 *    SBC audio stream is encoded from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data. 
 *    Other SBC configurations can be supported but there is a need for additional modifications to the protocol stack.
  */



// Code tested and works on WEMOS Wifi and Bluetooth Battery (actually using HiGrow hardware)
// The ESP32 is visible as ESP_A2DP_SRC - Would be nice if it was possible to send from ESP32 to android phone, but AFAICT the android stack doesn't implement the sink protocol
// Change this to the name of your bluetooth speaker/headset
#define  BT_SINK "BNX-60"
#define BT_DEVICE_NAME "ESP_A2DP_SRC"

// samples from the mod archive: https://modarchive.org/index.php?request=view_by_moduleid&query=42146
#include "enigma.h"
#define CURMOD enigma_mod
//#include "one_channel_moog_mod.h"
//#define CURMOD one_channel_moog_mod   //enigma_mod


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "ESP_LOG.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "esp32-hal-bt.h"
#include "esp32-hal-bt.c"
};
#include "bt_app_core.h"
#include "bt_app_core.c"


#define BT_AV_TAG   "BT_AV"

/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/* A2DP global state */
enum {
    APP_AV_STATE_IDLE,
    APP_AV_STATE_DISCOVERING,
    APP_AV_STATE_DISCOVERED,
    APP_AV_STATE_UNCONNECTED,
    APP_AV_STATE_CONNECTING,
    APP_AV_STATE_CONNECTED,
    APP_AV_STATE_DISCONNECTING,
};

/* sub states of APP_AV_STATE_CONNECTED */
enum {
    APP_AV_MEDIA_STATE_IDLE,
    APP_AV_MEDIA_STATE_STARTING,
    APP_AV_MEDIA_STATE_STARTED,
    APP_AV_MEDIA_STATE_STOPPING,
};

#define BT_APP_HEART_BEAT_EVT                (0xff00)

/// handler for bluetooth stack enabled events
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

/// callback function for A2DP source
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

/// callback function for A2DP source audio data stream
static int32_t bt_app_a2d_data_cb_static(uint8_t *data, int32_t len);
static int32_t bt_app_a2d_data_cb_increase(uint8_t *data, int32_t len);
static int32_t bt_app_a2d_data_cb_sine(uint8_t *data, int32_t len);
static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len);

static void a2d_app_heart_beat(void *arg);

/// A2DP application state machine
static void bt_app_av_sm_hdlr(uint16_t event, void *param);

/* A2DP application state machine handler for each state */
static void bt_app_av_state_unconnected(uint16_t event, void *param);
static void bt_app_av_state_connecting(uint16_t event, void *param);
static void bt_app_av_state_connected(uint16_t event, void *param);
static void bt_app_av_state_disconnecting(uint16_t event, void *param);

static esp_bd_addr_t s_peer_bda = {0};
static uint8_t s_peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static int s_a2d_state = APP_AV_STATE_IDLE;
static int s_media_state = APP_AV_MEDIA_STATE_IDLE;
static int s_intv_cnt = 0;
static int s_connecting_intv = 0;
static uint32_t s_pkt_cnt = 0;

static TimerHandle_t s_tmr;


static int sine_phase;

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

void setup()
{
 
  Serial.begin(115200);
  while(!Serial) { ; }  // wait for serial port to connect. Needed for native USB port only
  Serial.flush();
  Serial.println();
    
  const char compile_data[] = __FILE__ " " __DATE__ " " __TIME__;
  Serial.println(compile_data);




    
    Serial.println("Initialize Non-Volatile Storage (NVS)");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        Serial.printf("ESP_LOGE: BT_AV_TAG: In func: %s: initialize controller failed\n", __func__);
        return;
    }

//  Doesn't like BT Classic, but works with Dual Mode (ie don't use ESP_BT_MODE_CLASSIC_BT)
    if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) {
        Serial.printf("ESP_LOGE: BT_AV_TAG: In func: %s: enable controller failed\n", __func__);
        return;
    }

    if (esp_bluedroid_init() != ESP_OK) {
        Serial.printf("ESP_LOGE: BT_AV_TAG: In func: %s: initialize bluedroid failed\n", __func__);
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        Serial.printf("ESP_LOGE: BT_AV_TAG: In func: %s: enable bluedroid failed\n", __func__);
        return;
    }

    /* create application task */
    bt_app_task_start_up();

    /* Bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}

void loop() {}

static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void filter_inquiry_scan_result(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *eir = NULL;
    esp_bt_gap_dev_prop_t *p;

    Serial.printf("ESP_LOGI: BT_AV_TAG: Scanned device: %s\n", bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            Serial.printf("ESP_LOGI: BT_AV_TAG: --Class of Device: 0x%x\n", cod);
            // NB enumeration is listed in 
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            Serial.printf("ESP_LOGI: BT_AV_TAG: --RSSI (Received signal strength indication): %d\n", rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            eir = (uint8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
        default:
            break;
        }
    }

    /* search for device with MAJOR service class as "rendering" in COD */
    if (!esp_bt_gap_is_valid_cod(cod) ||
            !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING)) {
        return;
    }

    /* search for device named  BT_SINK in its extended inqury response */
    if (eir) {
        get_name_from_eir(eir, s_peer_bdname, NULL);
        if (strcmp((char *)s_peer_bdname,  BT_SINK) != 0) {
            return;
        }

        Serial.printf("ESP_LOGI: BT_AV_TAG: Found a target device, address %s, name %s\n", bda_str, s_peer_bdname);
        s_a2d_state = APP_AV_STATE_DISCOVERED;
        memcpy(s_peer_bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
        Serial.printf("ESP_LOGI: BT_AV_TAG: Cancel device discovery ...\n");
        esp_bt_gap_cancel_discovery();
    }
}


void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        filter_inquiry_scan_result(param);
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            if (s_a2d_state == APP_AV_STATE_DISCOVERED) {
                s_a2d_state = APP_AV_STATE_CONNECTING;
//                Serial.printf("ESP_LOGI: BT_AV_TAG: Device discovery stopped.\n");
                Serial.printf("ESP_LOGI: BT_AV_TAG: Device discovery stopped: a2dp connecting to peer: %s\n", s_peer_bdname);
                esp_a2d_source_connect(s_peer_bda);
            } else {
                // not discovered, continue to discover
                Serial.printf("ESP_LOGI: BT_AV_TAG: Device discovery failed, continue to discover...\n");
                esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            }
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            Serial.printf("ESP_LOGI: BT_AV_TAG: Discovery started.\n");
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVCS_EVT:
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            Serial.printf("ESP_LOGI: BT_AV_TAG: authentication success: %s", param->auth_cmpl.device_name);
//            ESP_LOG_buffer_hex(" + BT_AV_TAG + ", param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            Serial.printf("ESP_LOGE: BT_AV_TAG authentication failed, status:%d\n", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        Serial.printf("ESP_LOGI: BT_AV_TAG: ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d\n", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            Serial.printf("ESP_LOGI: BT_AV_TAG: Input pin code: 0000 0000 0000 0000\n");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            Serial.printf("ESP_LOGI: BT_AV_TAG: Input pin code: 1234\n");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        Serial.printf("ESP_LOGI: BT_AV_TAG: ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d\n", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        Serial.printf("ESP_LOGI: BT_AV_TAG: ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d\n", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        Serial.printf("ESP_LOGI: BT_AV_TAG: ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!\n");
        break;
#endif

    default: {
        Serial.printf("ESP_LOGI: BT_AV_TAG: event: %d", event);
        break;
    }
    }
    return;
}

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    Serial.printf("ESP_LOGD: BT_AV_TAG: %s evt %d\n", __func__, event);
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        /* set up device name */
        char *dev_name = BT_DEVICE_NAME;
        esp_bt_dev_set_device_name(dev_name);

        /* register GAP callback function */
        esp_bt_gap_register_callback(bt_app_gap_cb);

        /* initialize A2DP source */
        esp_a2d_register_callback(&bt_app_a2d_cb);
//        esp_a2d_source_register_data_callback(bt_app_a2d_data_cb_static);           // generates static/white noise
//        static int32_t bt_app_a2d_data_cb_increase(uint8_t *data, int32_t len);     // generates linear increasing noise
//        static int32_t bt_app_a2d_data_cb_sine(uint8_t *data, int32_t len);         // generates sine wave noise
        esp_a2d_source_register_data_callback(bt_app_a2d_data_cb);
        esp_a2d_source_init();

        /* set discoverable and connectable mode */
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);

        /* start device discovery */
        Serial.printf("ESP_LOGI: BT_AV_TAG: Starting device discovery...\n");
        s_a2d_state = APP_AV_STATE_DISCOVERING;
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);

        /* create and start heart beat timer */
        do {
            int tmr_id = 0;
            s_tmr = xTimerCreate("connTmr", (10000 / portTICK_RATE_MS),
                               pdTRUE, (void *)tmr_id, a2d_app_heart_beat);
            xTimerStart(s_tmr, portMAX_DELAY);
        } while (0);
        break;
    }
    default:
        Serial.printf("ESP_LOGE: BT_AV_TAG: In func: %s: unhandled evt %d", __func__, event);
        break;
    }
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    bt_app_work_dispatch(bt_app_av_sm_hdlr, event, param, sizeof(esp_a2d_cb_param_t), NULL);
}

static void a2d_app_heart_beat(void *arg)
{
    bt_app_work_dispatch(bt_app_av_sm_hdlr, BT_APP_HEART_BEAT_EVT, NULL, 0, NULL);
}

static void bt_app_av_sm_hdlr(uint16_t event, void *param)
{
    Serial.printf("ESP_LOGI: BT_AV_TAG: %s state %d, evt 0x%x: ", __func__, s_a2d_state, event);
    switch (s_a2d_state) {
    case APP_AV_STATE_DISCOVERING:
        Serial.println("APP_AV_STATE_DISCOVERING");
        break;
    case APP_AV_STATE_DISCOVERED:
        Serial.println("APP_AV_STATE_DISCOVERED");
        break;
    case APP_AV_STATE_UNCONNECTED:
        Serial.println("APP_AV_STATE_UNCONNECTED");
        bt_app_av_state_unconnected(event, param);
        break;
    case APP_AV_STATE_CONNECTING:
        Serial.println("APP_AV_STATE_CONNECTING");
        bt_app_av_state_connecting(event, param);
        break;
    case APP_AV_STATE_CONNECTED:
        Serial.println("APP_AV_STATE_CONNECTED");
        bt_app_av_state_connected(event, param);
        break;
    case APP_AV_STATE_DISCONNECTING:
        Serial.println("APP_AV_STATE_DISCONNECTING");
        bt_app_av_state_disconnecting(event, param);
        break;
    default:
        Serial.printf("\nESP_LOGE: BT_AV_TAG: In func: %s: invalid state %d\n", __func__, s_a2d_state);
        break;
    }
}

static void bt_app_av_state_unconnected(uint16_t event, void *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        break;
    case BT_APP_HEART_BEAT_EVT: {
        uint8_t *p = s_peer_bda;
        Serial.printf("ESP_LOGI: BT_AV_TAG: Heartbeat Event: a2dp most recent peer connection: %s @ %02x:%02x:%02x:%02x:%02x:%02x\n",
                 s_peer_bdname, p[0], p[1], p[2], p[3], p[4], p[5]);
        esp_a2d_source_connect(s_peer_bda);
        s_a2d_state = APP_AV_STATE_CONNECTING;
        s_connecting_intv = 0;
        break;
    }
    default:
        Serial.printf("ESP_LOGE: BT_AV_TAG: In func: %s: unhandled evt %d\n", __func__, event);
        break;
    }
}

static void bt_app_av_state_connecting(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp connected\n");
            s_a2d_state =  APP_AV_STATE_CONNECTED;
            s_media_state = APP_AV_MEDIA_STATE_IDLE;
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_NONE);
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            s_a2d_state =  APP_AV_STATE_UNCONNECTED;
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        break;
    case BT_APP_HEART_BEAT_EVT:
        if (++s_connecting_intv >= 2) {
            s_a2d_state = APP_AV_STATE_UNCONNECTED;
            s_connecting_intv = 0;
        }
        break;
    default:
        Serial.printf("ESP_LOGE: BT_AV_TAG: In func: %s: unhandled evt %d\n", __func__, event);
        break;
    }
}

static void bt_app_av_media_proc(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;
    switch (s_media_state) {
    case APP_AV_MEDIA_STATE_IDLE: {
        if (event == BT_APP_HEART_BEAT_EVT) {
            Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp media ready checking ...\n");
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
        } else if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
            a2d = (esp_a2d_cb_param_t *)(param);
            if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY &&
                    a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp media ready, starting ...\n");
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
                s_media_state = APP_AV_MEDIA_STATE_STARTING;
            }
        }
        break;
    }
    case APP_AV_MEDIA_STATE_STARTING: {
        if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
            a2d = (esp_a2d_cb_param_t *)(param);
            if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START &&
                    a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp media start successfully.\n");
                s_intv_cnt = 0;
                s_media_state = APP_AV_MEDIA_STATE_STARTED;
            } else {
                // not started succesfully, transfer to idle state
                Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp media start failed.\n");
                s_media_state = APP_AV_MEDIA_STATE_IDLE;
            }
        }
        break;
    }
    case APP_AV_MEDIA_STATE_STARTED: {
/*        if (event == BT_APP_HEART_BEAT_EVT) {
            if (++s_intv_cnt >= 10) {
                Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp media stopping...\n");
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
                s_media_state = APP_AV_MEDIA_STATE_STOPPING;
                s_intv_cnt = 0;
            }
        }
*/        break;
    }
    case APP_AV_MEDIA_STATE_STOPPING: {
        if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
            a2d = (esp_a2d_cb_param_t *)(param);
            if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_STOP &&
                    a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp media stopped successfully, disconnecting...\n");
                s_media_state = APP_AV_MEDIA_STATE_IDLE;
                esp_a2d_source_disconnect(s_peer_bda);
                s_a2d_state = APP_AV_STATE_DISCONNECTING;
            } else {
                Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp media stopping...\n");
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
            }
        }
        break;
    }
    }
}

static void bt_app_av_state_connected(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp disconnected\n");
            s_a2d_state = APP_AV_STATE_UNCONNECTED;
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            s_pkt_cnt = 0;
        }
        break;
    }
    case ESP_A2D_AUDIO_CFG_EVT:
        // not suppposed to occur for A2DP source
        break;
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    case BT_APP_HEART_BEAT_EVT: {
        bt_app_av_media_proc(event, param);
        break;
    }
    default:
        Serial.printf("ESP_LOGE: BT_AV_TAG: In func: %s: unhandled evt %d\n", __func__, event);
        break;
    }
}

static void bt_app_av_state_disconnecting(uint16_t event, void *param)
{
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            Serial.printf("ESP_LOGI: BT_AV_TAG: a2dp disconnected\n");
            s_a2d_state =  APP_AV_STATE_UNCONNECTED;
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    case BT_APP_HEART_BEAT_EVT:
        break;
    default:
        Serial.printf("ESP_LOGE: BT_AV_TAG: In func: %s: unhandled evt %d\n", __func__, event);
        break;
    }
}



static int32_t bt_app_a2d_data_cb_static(uint8_t *data, int32_t len)
{
    // Generates random numbers to provide static / white noise for forwarding to the output
    if (len < 0 || data == NULL) {
        Serial.printf("entered func: %s but no data to load\n", __func__);
        return 0;
    }

    // generate random sequence
    int val = rand() % (1 << 16);                   // restrict the number to a 16bit integer (ie max of 65536)
    for (int i = 0; i < (len >> 1); i++) {          // len>>1==len/2 : data format is uint8_t, but SBC wants uint16_t, so you run the process once for every two bytes
        data[(i << 1)] = val & 0xff;                // use bit mask to retain only the low byte     // i<<1==i*2 : ie step 2 bytes for every value
        data[(i << 1) + 1] = (val >> 8) & 0xff;     // bit shift to move the upper byte to the lower, then bit mask to retain only this part (ie mask out the old low byte)
    }

    return len;
}


uint16_t SBCVal=0;
static int32_t bt_app_a2d_data_cb_increase(uint8_t *data, int32_t len)
{
    // generates increasing value for forwarding to the output
    if (len < 0 || data == NULL) {
        Serial.printf("entered func: %s but no data to load\n", __func__);
        return 0;
    }

    for (int i = 0; i < (len >> 1); i++) {          // len>>1==len/2 : data format is uint8_t, but SBC wants uint16_t, so you run the process once for every two bytes
        data[(i << 1)] = SBCVal & 0xff;                // use bit mask to retain only the low byte     // i<<1==i*2 : ie step 2 bytes for every value
        data[(i << 1) + 1] = (SBCVal >> 8) & 0xff;     // bit shift to move the upper byte to the lower, then bit mask to retain only this part (ie mask out the old low byte)
        SBCVal = (SBCVal +1) % (1 << 16);       // increment the value to continue the sequence (max val 65536)
    }
    
    return len;
}


static const int16_t sine_int16[] = {
  32768,     34825,     36875,     38908,     40917,     42894,     44830,     46720,     48554,     50325,
  52028,     53654,     55199,     56654,     58015,     59277,     60434,     61482,     62416,     63234,
  63931,     64506,     64955,     65277,     65470,     65535,     65470,     65277,     64955,     64506,
  63931,     63234,     62416,     61482,     60434,     59277,     58015,     56654,     55199,     53654,
  52028,     50325,     48554,     46720,     44830,     42894,     40917,     38908,     36875,     34825,
  32768,     30711,     28661,     26628,     24619,     22642,     20706,     18816,     16982,     15211,
  13508,     11882,     10337,      8882,      7521,      6259,      5102,      4054,      3120,      2302,
   1605,      1030,       581,       259,        66,         1,        66,       259,       581,      1030,
   1605,      2302,      3120,      4054,      5102,      6259,      7521,      8882,     10337,     11882,
  13508,     15211,     16982,     18816,     20706,     22642,     24619,     26628,     28661,     30711,
};
int sineTableSize = sizeof(sine_int16);
static int32_t bt_app_a2d_data_cb_sine(uint8_t *data, int32_t len)
{
    // generates increasing value for forwarding to the output
    if (len < 0 || data == NULL) {
        Serial.printf("entered func: %s but no data to load\n", __func__);
        return 0;
    }

    // generate sine wave
    for (int i = 0; i < (len >> 1); i++) {
        data[(i << 1)] = sine_int16[sine_phase];
        data[(i << 1) + 1] = sine_int16[sine_phase];
        sine_phase = (sine_phase +1) % sineTableSize;
    }

    return len;
}



uint32_t modLoc=100;                  // Just a test, so guess a place to start which is hopefully past the header
uint32_t modSize=sizeof(CURMOD);
static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len)
{
    // plays a mod from program memory
    if (len < 0 || data == NULL) {
        Serial.printf("entered func: %s but no data to load\n", __func__);
        return 0;
    }

    // use longer print statements only for testing: if you print too much it slows the Task and the watchdog may trigger
    Serial.printf("%d -> %d\n", modLoc, len);
//    Serial.printf("func: %s: Loaded data from %d to ", __func__, modLoc);

    int cycle=0;
    for (int i = 0; i < (len >> 1); i++) {
        data[(i << 1)] = CURMOD[modLoc];
        data[(i << 1) + 1] = CURMOD[modLoc+1];

        cycle++;
        if (cycle == 20) {
          modLoc = (modLoc + 2) % modSize;
          if (modLoc < 100) modLoc=100;
          cycle=0;
        }


    }
//    Serial.printf("to %d bytes of file (total size: %d)\n", modLoc, modSize);

    return len;
}



  //        data[(i << 1) + 1] = 0x00;   // for 8bit data will have to be padded out
  //        data[(i << 1)] = CURMOD[i] & 0xff;
  //        data[(i << 1) + 1] = (CURMOD[i+1] >> 8) & 0xff;     // bit shift to move the upper byte to the lower, then bit mask to retain only this part (ie mask out the old low byte)

