#include <string.h>
#include "ble_gap.h"
#include "ble_gatt.h"
#include "ble_gatts.h"
#include "ble_error.h"
#include "dbg_print.h"
#include "ble_utils.h"
#include "wrapper_os.h"

enum ble_es_srv_att_idx
{
    BLE_ES_SRV_IDX_SVC,
    BLE_ES_SRV_IDX_PRESSURE_CHAR,
    BLE_ES_SRV_IDX_PRESSURE_CHAR_VAL,
    BLE_ES_SRV_IDX_PRESSURE_CCCD_CFG,
    BLE_ES_SRV_IDX_TEMPERATURE_CHAR,
    BLE_ES_SRV_IDX_TEMPERATURE_CHAR_VAL,
    BLE_ES_SRV_IDX_TEMPERATURE_CCCD_CFG,
    BLE_ES_SRV_IDX_HUMIDITY_CHAR,
    BLE_ES_SRV_IDX_HUMIDITY_CHAR_VAL,
    BLE_ES_SRV_IDX_HUMIDITY_CCCD_CFG,
    BLE_ES_SRV_IDX_NB,
};

static uint8_t svc_id;

const uint8_t ble_es_srv_svc_uuid[BLE_GATT_UUID_16_LEN] = UUID_16BIT_TO_ARRAY(BLE_GATT_SVC_ENVIRONMENTAL_SENSING);

const ble_gatt_attr_desc_t ble_es_srv_att_db[BLE_ES_SRV_IDX_NB] = {

    [BLE_ES_SRV_IDX_SVC]                        = { UUID_16BIT_TO_ARRAY(BLE_GATT_DECL_PRIMARY_SERVICE)  , PROP(RD)            , 0                                                },

    [BLE_ES_SRV_IDX_PRESSURE_CHAR]              = { UUID_16BIT_TO_ARRAY(BLE_GATT_DECL_CHARACTERISTIC)   , PROP(RD)            , 0                                                },

    [BLE_ES_SRV_IDX_PRESSURE_CHAR_VAL]          = { UUID_16BIT_TO_ARRAY(BLE_GATT_CHAR_PRESSURE)         , PROP(NTF)           , OPT(NO_OFFSET)                                   },

    [BLE_ES_SRV_IDX_PRESSURE_CCCD_CFG]          = { UUID_16BIT_TO_ARRAY(BLE_GATT_DESC_CLIENT_CHAR_CFG)  , PROP(RD) | PROP(WR) , OPT(NO_OFFSET)                                   },

    [BLE_ES_SRV_IDX_TEMPERATURE_CHAR]           = { UUID_16BIT_TO_ARRAY(BLE_GATT_DECL_CHARACTERISTIC)   , PROP(RD)            , 0                                                },

    [BLE_ES_SRV_IDX_TEMPERATURE_CHAR_VAL]       = { UUID_16BIT_TO_ARRAY(BLE_GATT_CHAR_TEMPERATURE)      , PROP(NTF)           , OPT(NO_OFFSET)                                   },

    [BLE_ES_SRV_IDX_TEMPERATURE_CCCD_CFG]       = { UUID_16BIT_TO_ARRAY(BLE_GATT_DESC_CLIENT_CHAR_CFG)  , PROP(RD) | PROP(WR) , OPT(NO_OFFSET)                                   },

    [BLE_ES_SRV_IDX_HUMIDITY_CHAR]              = { UUID_16BIT_TO_ARRAY(BLE_GATT_DECL_CHARACTERISTIC)   , PROP(RD)            , 0                                                },

    [BLE_ES_SRV_IDX_HUMIDITY_CHAR_VAL]          = { UUID_16BIT_TO_ARRAY(BLE_GATT_CHAR_HUMIDITY)         , PROP(NTF)           , OPT(NO_OFFSET)                                   },

    [BLE_ES_SRV_IDX_HUMIDITY_CCCD_CFG]          = { UUID_16BIT_TO_ARRAY(BLE_GATT_DESC_CLIENT_CHAR_CFG)  , PROP(RD) | PROP(WR) , OPT(NO_OFFSET)                                   },

};

ble_status_t ble_es_srv_cb(ble_gatts_msg_info_t *p_srv_msg_info)
{
    uint8_t status = BLE_ERR_NO_ERROR;

    switch (p_srv_msg_info->srv_msg_type) {
    case BLE_SRV_EVT_SVC_ADD_RSP:
        dbg_print(NOTICE, "[ble_es_srv_cb], svc_add_rsp status = 0x%x\r\n",
                  p_srv_msg_info->msg_data.svc_add_rsp.status);
        break;

    case BLE_SRV_EVT_SVC_RMV_RSP:
        dbg_print(NOTICE, "[ble_es_srv_cb], svc_rmv_rsp status = 0x%x\r\n",
                  p_srv_msg_info->msg_data.svc_rmv_rsp.status);
        break;

    case BLE_SRV_EVT_GATT_OPERATION: {
        uint8_t conn_idx = p_srv_msg_info->msg_data.gatts_op_info.conn_idx;

        if (p_srv_msg_info->msg_data.gatts_op_info.gatts_op_sub_evt == BLE_SRV_EVT_NTF_IND_SEND_RSP) {
            ble_gatts_ntf_ind_send_rsp_t *p_rsp = &p_srv_msg_info->msg_data.gatts_op_info.gatts_op_data.ntf_ind_send_rsp;
            (void) p_rsp;
        } else if (p_srv_msg_info->msg_data.gatts_op_info.gatts_op_sub_evt == BLE_SRV_EVT_WRITE_REQ) {
            ble_gatts_write_req_t *p_req = &p_srv_msg_info->msg_data.gatts_op_info.gatts_op_data.write_req;
            (void) p_req;
        } else if (p_srv_msg_info->msg_data.gatts_op_info.gatts_op_sub_evt == BLE_SRV_EVT_READ_REQ) {
            ble_gatts_read_req_t * p_req = &p_srv_msg_info->msg_data.gatts_op_info.gatts_op_data.read_req;

            switch (p_req->att_idx) {
                case BLE_ES_SRV_IDX_PRESSURE_CCCD_CFG:
                case BLE_ES_SRV_IDX_TEMPERATURE_CCCD_CFG:
                case BLE_ES_SRV_IDX_HUMIDITY_CCCD_CFG:
                    p_req->val_len = BLE_GATT_CCCD_LEN;
                    p_req->att_len = BLE_GATT_CCCD_LEN;
                    memset(p_req->p_val, 0, p_req->val_len);
                    break;

                default:
                    break;
            }
        }
    } break;

    default:
        break;
    }

    return status;
}

ble_status_t ble_es_srv_init(void)
{
    return ble_gatts_svc_add(&svc_id, ble_es_srv_svc_uuid, 0, SVC_UUID(16),
                             ble_es_srv_att_db, BLE_ES_SRV_IDX_NB,
                             ble_es_srv_cb);
}

ble_status_t ble_es_srv_deinit(void)
{
    return ble_gatts_svc_rmv(svc_id);
}

ble_status_t ble_es_srv_send_pressure(uint8_t conn_idx, float value)
{
    uint32_t converted = (uint32_t) (value * 100.0f);
    return ble_gatts_ntf_ind_send(conn_idx, svc_id, BLE_ES_SRV_IDX_PRESSURE_CHAR_VAL,
                                  (void *) &converted, sizeof(converted), BLE_GATT_NOTIFY);
}

ble_status_t ble_es_srv_send_temperature(uint8_t conn_idx, float value)
{
    int16_t converted = (int16_t) (value * 100.0f);
    return ble_gatts_ntf_ind_send(conn_idx, svc_id, BLE_ES_SRV_IDX_TEMPERATURE_CHAR_VAL,
                                  (void *) &converted, sizeof(converted), BLE_GATT_NOTIFY);
}

ble_status_t ble_es_srv_send_humidity(uint8_t conn_idx, float value)
{
    uint16_t converted = (uint16_t) (value * 100.0f);
    return ble_gatts_ntf_ind_send(conn_idx, svc_id, BLE_ES_SRV_IDX_HUMIDITY_CHAR_VAL,
                                  (void *) &converted, sizeof(converted), BLE_GATT_NOTIFY);
}