/*
 * app_main.c
 *
 *  Created on: Oct 24, 2023
 *      Author: G4T1PR0
 */

#include <app_main.h>
#include <Devices/Devices.hpp>
#include <HardwareController/HardwareController.hpp>

Devices devices;
HardwareController hwc(&devices);

void app_init() {
    devices.init();
    hwc.init();
}
void app_main() {
    app_init();
    while (1) {
        hwc.update();
    }
}