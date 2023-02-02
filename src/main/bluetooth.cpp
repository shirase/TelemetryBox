#include "targets/target.h"

#ifdef USE_BLUETOOTH

#include "bluetooth.h"
#include <Arduino.h>

BluetoothSerial BTSerial;

static void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    /*if (event == ESP_SPP_SRV_OPEN_EVT) {
    
    } 
    else if (event == ESP_SPP_SRV_STOP_EVT) {
    
    }*/
}

void bluetoothInit()
{
    BTSerial.begin("Mavlink");
    BTSerial.register_callback((esp_spp_cb_t *)callback);
}

#endif