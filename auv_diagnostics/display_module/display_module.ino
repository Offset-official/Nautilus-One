#include <SPI.h>
#include "LCD_Driver.h"
#include "GUI_Paint.h"
#include "image.h"
#include <ArduinoJson.h>
#include <SD.h>


#define MAX_IMAGE_SIZE (100 * 100 * 2)

const int chipSelect = BUILTIN_SDCARD; 


DynamicJsonDocument doc(2048); // Global allocation for JSON parsing

void setup() {
    Config_Init();
    LCD_Init();
    LCD_Clear(0xffff); 
    Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, WHITE);
    Paint_Clear(WHITE);

    Serial.print("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
    }
    Serial.println("card initialized.");

    // Loop through all frames
    for (int frameIndex = 1; frameIndex <= 72; frameIndex++) {
        char filename[72];
        snprintf(filename, sizeof(filename), "/loading_screen/%04d.rgb565", frameIndex);
        display_image_from_sd(filename, 110, 65, 100, 100);
    }

    LCD_Clear(0xffff);
    Paint_Clear(WHITE);


    // Initial drawing
    Paint_DrawImage(disarmed, 0, 0, 35, 35);
}

void loop() {
    if (Serial.available() > 0) { 

        DeserializationError error = deserializeJson(doc, Serial);
        if (error) {
            Serial.print("Failed to parse JSON: ");
            Serial.println(error.f_str());
            Serial.flush();
            return;
        }

        // Process JSON
        const unsigned char* arm_status = doc["armed"] ? armed : disarmed;
        const char* battery_J_val = doc["battery_J"] | "NA";
        const char* battery_N_val = doc["battery_N"] | "NA";
        const char* task_val = doc["task"] | "NTSK";
        const char* jetson_nav_conn_val = doc["jetson_connection"] ? jetson_nav_conn : jetson_nav_conn_not;

        // Update Display
        static const unsigned char* last_arm_status = nullptr;
        if (arm_status != last_arm_status) {
            Paint_DrawImage((arm_status), 0, 3, 35, 35);
            last_arm_status = arm_status;
        }

        // Battery
        Paint_DrawImage(battery_J, 260, 3, 35, 35);
        Paint_DrawString_EN(290, 12.5, battery_J_val, &Font20, WHITE, BLACK);

        Paint_DrawImage(battery_N, 200, 3, 35, 35);
        Paint_DrawString_EN(232, 12.5, battery_N_val, &Font20, WHITE, BLACK);

        // Task
        Paint_DrawImage(task, 3, 202, 35, 35);
        Paint_DrawString_EN(40 , 211, task_val, &Font20, WHITE, BLACK);

        // Jetson Navigator Connection
        Paint_DrawImage(jetson_nav_conn_val, 35, 3, 35, 35);
    }
}


void display_image_from_sd(const char* filename, UWORD Startx, UWORD Starty, UWORD Width, UWORD Height) {
    // Open the image file from the SD card
    File file = SD.open(filename);
    if (!file) {
        Serial.print("Failed to open ");
        Serial.println(filename);
        return;
    }

    // Allocate a buffer to read the image data
    const int bufferSize = Width * Height * 2; // RGB565 format (2 bytes per pixel)
    unsigned char* imageData = (unsigned char*)malloc(bufferSize);
    if (!imageData) {
        Serial.println("Failed to allocate memory for image data.");
        file.close();
        return;
    }

    // Read the image data into the buffer
    int bytesRead = file.read(imageData, bufferSize);
    if (bytesRead != bufferSize) {
        Serial.println("Error: Image file is incomplete or corrupted.");
        free(imageData);
        file.close();
        return;
    }

    // Close the file as it's no longer needed
    file.close();

    // Display the image using Paint_DrawImage
    Paint_DrawImage(imageData, Startx, Starty, Width, Height);

    // Free the allocated memory
    free(imageData);

    Serial.println("Image displayed!");
}
