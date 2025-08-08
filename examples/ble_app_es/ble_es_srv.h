#ifndef BLE_ES_SRV_H

#include <ble_error.h>

ble_status_t ble_es_srv_init(void);
ble_status_t ble_es_srv_deinit(void);
ble_status_t ble_es_srv_send_pressure(int conn_idx, float value);
ble_status_t ble_es_srv_send_temperature(int conn_idx, float value);
ble_status_t ble_es_srv_send_humidity(int conn_idx, float value);

#endif
