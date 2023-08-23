#include "telemetry.h"
#include <Arduino.h>
#include "commands.h"
#include "input.h"

// Telemetry message rate in Hz
//Default: 10hz
//Maximum: 40hz (depending on what additional processing is being done)
//#define TELEMETRY_HZ 10
#define TELEMETRY_HZ 1

uint16_t wait = 1000/TELEMETRY_HZ;

void telemetryTask(void * parameter)
{
    for( ;; ) {
        TickType_t start = xTaskGetTickCount();

        // Send MAVLink heartbeat
        command_heartbeat(system_id, component_id, system_type, autopilot_type, system_mode, custom_mode, system_state);

        // Send parameters (needed for detection by some GCS software)
        command_parameters(system_id, component_id);

        //Send battery status
        command_status(system_id, component_id, battery_remaining, voltage_battery, current_battery);

        // Send GPS and altitude data
        command_gps(system_id, component_id, upTime, fixType, lat, lon, alt, gps_alt, heading, groundspeed, gps_hdop, gps_sats);

        // Send HUD data (speed, heading, climbrate etc.)
        command_hud(system_id, component_id, airspeed, groundspeed, heading, throttle, alt, climbrate);

        // Send attitude data to artificial horizon
        command_attitude(system_id, component_id, upTime, roll, pitch, yaw);

        TickType_t delay = wait / portTICK_PERIOD_MS - (xTaskGetTickCount() - start);
        vTaskDelay(delay > 0 ? delay : 1);
    }
}
