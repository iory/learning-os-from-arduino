#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

void setup()
{
    matrix.begin();

    uint8_t frame[8][12] = {
        {1,1,0,0,1,1,1,1,1,1,1,1},
        {1,1,0,0,1,1,1,1,1,1,1,1},
        {1,1,0,0,1,1,0,0,1,1,0,0},
        {1,1,0,0,1,1,0,0,1,1,0,0},
        {1,1,0,0,1,1,0,0,1,1,0,0},
        {1,1,0,0,1,1,0,0,1,1,0,0},
        {1,1,1,1,1,1,0,0,1,1,0,0},
        {0,1,1,1,1,0,0,0,1,1,0,0},
    };

    matrix.renderBitmap(frame, 8, 12);
}

void loop()
{
    delay(1000);
}
