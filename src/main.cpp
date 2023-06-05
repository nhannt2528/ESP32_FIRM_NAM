#include <Arduino.h>
//  #include "tu_dieu_khien_bom_ht2.h"
// #include "tu_dieu_khien_1_bom.h"
// #include "sensor_7in1_lcd.h"
// #include "test.h"
// #include "check_flowsensor.h"
// #include "rem_cat_nang.h"
// #include "plc-gate.h"
// #include "../include/may_cham_phan_DaiHocDalat/champhan.h"
// #include "testEC.h"
// #include "pmw.h"
// #include "cat_rem_dhdl.h"
// #include "champhanvuondau.h"
// #include "dosing_gateway.h"
// #include "pidEC.h"
// #include "1pump.h"
// #include "lora_relay.h"
// #include "dwin.h"
// #include "test_Dwin.h"
#include "gatewaysensor.h"
// #include "testboard.h"
void setup()
{
    appInit();
    // Serial.begin(9600);
    // Serial2.begin(9600, SERIAL_8N1);
    // Serial.println("Start");
    // pinMode(2, INPUT_PULLUP);
    // Serial.begin(9600);
    // DWIN_LCD.begin(115200, SERIAL_8N1, 4, 5);
}
void loop()
{
    appRun();
    // if(Serial2.available()){
    //      Serial.print((char)Serial2.read());
    }
//    if (digitalRead(2) == 0)
//     {
//         if (millis() - timeoutbt > 500)
//         {
            
//             Sw_page++;
//             if (Sw_page > 3)
//             {
//                 Sw_page = 0;
//             }
//             dwinShowPage(Sw_page);
//             Serial.println("PAGE : " + String(Sw_page));
//             timeoutbt = millis();
//         }
//     }
//     updateHMIState();
// }
