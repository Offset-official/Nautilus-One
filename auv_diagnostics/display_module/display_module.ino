#include <SPI.h>
#include "LCD_Driver.h"
#include "GUI_Paint.h"
#include "image.h"
#include <ArduinoJson.h>

void setup()
{
  Config_Init();
  LCD_Init();
  LCD_Clear(0xffff);
  Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, WHITE);
  Paint_Clear(WHITE);
  // Paint_DrawString_EN(30, 10, "D", &Font24, YELLOW, RED);
  // Paint_DrawString_EN(30, 34, "ABC", &Font24, BLUE, CYAN);
  
  
  // Paint_DrawRectangle(125, 10, 225, 58, RED,  DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
  // Paint_DrawLine(125, 10, 225, 58, MAGENTA,   DOT_PIXEL_2X2,LINE_STYLE_SOLID);
  // Paint_DrawLine(225, 10, 125, 58, MAGENTA,   DOT_PIXEL_2X2,LINE_STYLE_SOLID);
  
  // Paint_DrawCircle(150,100, 25, BLUE,   DOT_PIXEL_2X2,   DRAW_FILL_EMPTY);
  // Paint_DrawCircle(180,100, 25, BLACK,  DOT_PIXEL_2X2,   DRAW_FILL_EMPTY);
  // Paint_DrawCircle(210,100, 25, RED,    DOT_PIXEL_2X2,   DRAW_FILL_EMPTY);
  // Paint_DrawCircle(165,125, 25, YELLOW, DOT_PIXEL_2X2,   DRAW_FILL_EMPTY);
  // Paint_DrawCircle(195,125, 25, GREEN,  DOT_PIXEL_2X2,   DRAW_FILL_EMPTY);
  
  

  Paint_DrawImage(disarmed, 0, 0, 35, 35); 
  // //Paint_DrawFloatNum (5, 150 ,987.654321,4,  &Font20,    WHITE,   LIGHTGREEN);

}
void loop()
{
  if (Serial.available() > 0){
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, Serial);

    if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.f_str());
      return;
    }

    // Accessing Data from JSON
    // const char* arm_status = doc["armed"] ? "A" : "D";

    const char* arm_status = doc["armed"] ? armed : disarmed;

    Paint_DrawImage(arm_status, 0, 0, 35, 35);
  }
  
}



/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
