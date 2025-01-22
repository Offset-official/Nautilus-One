#include <SPI.h>
#include "LCD_Driver.h"
#include "GUI_Paint.h"
#include "image.h"
#include <ArduinoJson.h>

DynamicJsonDocument doc(2048); // Global allocation for JSON parsing

void setup() {
    Config_Init();
    LCD_Init();
    LCD_Clear(0xffff); 
    Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, WHITE);
    Paint_Clear(WHITE);

    // Initial drawing
    Paint_DrawImage(disarmed, 0, 0, 35, 35);
}

void loop() {
    if (Serial.available() > 0) { // Ensure sufficient data
        // Parse JSON
        DeserializationError error = deserializeJson(doc, Serial);
        if (error) {
            Serial.print("Failed to parse JSON: ");
            Serial.println(error.f_str());
            Serial.flush(); // Clear serial buffer on error
            return;
        }

        // Process JSON
        const unsigned char* arm_status = doc["armed"] ? armed : disarmed;
        const char* battery_J_val = doc["battery_J"] | "NA";
        const char* battery_N_val = doc["battery_N"] | "NA";
        const char* task_val = doc["task"] | "NTSK";

        // Update Display
        static const unsigned char* last_arm_status = nullptr;
        if (arm_status != last_arm_status) {
            Paint_DrawImage(arm_status, 0, 0, 35, 35);
            last_arm_status = arm_status;
        }

        Paint_DrawImage(battery_J, 260, 0, 35, 35);
        Paint_DrawString_EN(290, 12.5, battery_J_val, &Font20, WHITE, BLACK);

        Paint_DrawImage(battery_N, 200, 0, 35, 35);
        Paint_DrawString_EN(232, 12.5, battery_N_val, &Font20, WHITE, BLACK);

        Paint_DrawImage(task, 3, 202, 35, 35);
        Paint_DrawString_EN(40, 211, task_val, &Font20, WHITE, BLACK);
    }
}
