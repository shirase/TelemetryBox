#include <Arduino.h>
#include "targets/target.h"
#include "main/led.h"
#include "main/bluetooth.h"
#include "main/telemetry.h"
#include "main/crsf.h"
#include "main/mavlink.h"
#include "drivers/serial.h"
#include "drivers/uart.h"

void setup() {
  Serial.begin(9600);

#ifdef USE_BLUETOOTH
  bluetoothInit();
  serialInit(&mavlinkPort, &BTSerial);
#else
  serialInit(&mavlinkPort, &Serial);
#endif

  xTaskCreate(
    ledTask,
    "ledTask",
    configMINIMAL_STACK_SIZE,
    NULL,
    configMAX_PRIORITIES - 2,
    NULL
  );

  static uartPort_t uartPort = {
    .uart_num = CRSF_UART,
    .rxPin = CRSF_UART_RX_PIN,
    .txPin = CRSF_UART_TX_PIN,
    .baud_rate = CRSF_UART_BAUDRATE,
    .serial_port = &crsfPort,
  };
  uartInit(&uartPort);

  xTaskCreate(
    crsfTask,
    "crsfTask",
    configMINIMAL_STACK_SIZE * 4,
    NULL,
    5,
    NULL
  );

  xTaskCreate(
    mavlinkTask,
    "mavlinkTask",
    configMINIMAL_STACK_SIZE * 4,
    NULL,
    5,
    NULL
  );

  xTaskCreate(
    telemetryTask,
    "telemetryTask",
    configMINIMAL_STACK_SIZE * 4,
    NULL,
    5,
    NULL
  );
}

void loop() {
  // put your main code here, to run repeatedly:
}