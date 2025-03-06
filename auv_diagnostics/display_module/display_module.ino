#include <SPI.h>
#include "LCD_Driver.h"
#include "GUI_Paint.h"
#include "image.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#define PIN 6
#define NUMPIXELS 40
#define DELAYVAL 1


#define MAX_IMAGE_SIZE (100 * 100 * 2)

const int chipSelect = BUILTIN_SDCARD;

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);



DynamicJsonDocument doc(2048);  // Global allocation for JSON parsing

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
  pixels.begin();
  rainbow(10); 
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
    const char* neopixel_color = doc["neopixel_color"] | "off";
    const int neopixel_count = doc["neopixel_count"];

    // Work with neopixel strip
    uint8_t r, g, b;
    hexToRGB(neopixel_color, r, g, b);

    if (neopixel_count > 0) {
      pixels.clear();
      for (int i = 0; i < neopixel_count; i++) {
        // Light up from the beginning (0 to NUM_TO_LIGHT-1)
        pixels.setPixelColor(i, pixels.Color(r, g, b));

        // Light up from the end (39 to 40-NUM_TO_LIGHT)
        pixels.setPixelColor(40 - 1 - i, pixels.Color(r, g, b));
        pixels.show();
      }
    }

    else {
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(r, g, b));
        pixels.show();
      }
    }

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
    Paint_DrawString_EN(40, 211, task_val, &Font20, WHITE, BLACK);

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
  const int bufferSize = Width * Height * 2;  // RGB565 format (2 bytes per pixel)
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

void hexToRGB(const char* hex, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (hex[0] == '#') {
    hex++;  // Skip the '#' character
  }
  if (strlen(hex) == 6) {
    char rStr[3] = { hex[0], hex[1], '\0' };
    char gStr[3] = { hex[2], hex[3], '\0' };
    char bStr[3] = { hex[4], hex[5], '\0' };

    r = strtoul(rStr, NULL, 16);
    g = strtoul(gStr, NULL, 16);
    b = strtoul(bStr, NULL, 16);
  } else {
    r = g = b = 0;  // Default to black if invalid input
  }
}

void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    pixels.rainbow(firstPixelHue);
    pixels.show();
    delay(wait);
  }
}