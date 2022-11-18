/*
 * Sensor connected to the LEFT grove plug on the WIO.
 *  NeoPixel to WIO: Pin 4(5V), Pin 6(GND), Pin 16(D2)
 */
//#include <Adafruit_NeoPixel.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
//#include <TFT_eSPI.h>
#include <Wire.h>
#define Console Serial           // command processor input/output stream

const uint16_t PixelCount = 20;
const uint16_t PixelPin = D4;
const uint16_t CalibrationPin = D5;  // Used to set yellow/red transition distance
const uint16_t MaxRange = 1200;  // Units mm

int calSwitchStatus = 0;

char printBuffer[40];
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(PixelCount, PixelPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PixelCount, PixelPin, NEO_GRB);

//int GreenLedRange = (int) PixelCount * 33;
//int YellowLedRange = (int) PixelCount * 0.66;

int GreenLedRange = 7;
int YellowLedRange = 14;
// Strip is divided into 3 equal sizes, green, yellow, and red
//  "Stop" zone is when you are in the red area


int stopRange = 0;

void testStrip()
{

int pauseTime = 100;

    for (int i = 0; i < GreenLedRange; i++)
    {
        strip.setPixelColor(i, 0, 64, 0);
        strip.show();
        delay (pauseTime);
    }

    for (int i = GreenLedRange; i < YellowLedRange; i++)
    {
        strip.setPixelColor(i, 64, 0, 64);
        strip.show();
        delay (pauseTime);
    }

    for (int i = YellowLedRange; i < PixelCount; i++)
    {
        strip.setPixelColor(i, 64, 0, 0);
        strip.show();
        delay (pauseTime);
    }

    delay(500);
    strip.clear();
}

void setup()
{

    Console.begin(115200);
    while (!Serial)
    {
        delay(1);
    }

    // CalibrationPin input will be used to set the stop distance
    pinMode(CalibrationPin, INPUT_PULLUP);     // set pin to input
    digitalWrite(CalibrationPin, HIGH); // turn on pullup resistors

    strip.begin(); //
    strip.clear(); //
    strip.show();  //

    testStrip();

    delay(50);

    Console.println(F("VL53L0X API Simple Ranging example\n\n"));

    // int loxStartLimit = 100;
    // int loxStartTries = 0;
    // bool loxIsStarted = false;

    if (!lox.begin())
    {
        delay(50);
        Serial.println(F("Failed to boot VL53L0X"));
        strip.setPixelColor(2, 64, 0, 0); // 2nd LED: red for lox is not started
        strip.show();
        while (1)
            ;
    }
    else
    {
        Serial.println(F("Successfully started VL53L0X"));
        strip.setPixelColor(2, 0, 64, 0); // next LED: green for lox is  started
        strip.show();
        delay(1000);

        lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
        lox.startRangeContinuous();
    }
}

void loop()
{
    uint16_t ledPrevNum = 0, ledNum = 0, range = 0;
    range++;
    ledNum++;
    ledPrevNum++;

    strip.setPixelColor(0, 64, 64, 64); // white
    strip.show();
    delay(90);
    ///strip.setPixelColor(0, 32, 32, 32); // dimmer white
    ///strip.show();

    char str[12];
    VL53L0X_RangingMeasurementData_t measure;



    // First business....if the value of "stopRange" has not been set, 
    // indicate on strip that no calibration has been done...do not trust strip output

    calSwitchStatus = digitalRead (CalibrationPin);
    sprintf (printBuffer, "BUTTON STATE=%d", calSwitchStatus);
    //Serial.print (printBuffer);
    delay (100);
    if ((stopRange == 0) ||  (calSwitchStatus == 0))// or if button has been pressed to re-calibrate
    { // Could also set a persistent variable that button has been used.
    Serial.println ("Device is either not calibrated, or the calibration button has been pressed");
        strip.setPixelColor(0, 128, 0, 0);
        strip.setPixelColor(2, 128, 0, 0);
        strip.setPixelColor(4, 128, 0, 0);
        strip.setPixelColor(6, 128, 0, 0);
        strip.setPixelColor(8, 128, 0, 0);
        strip.show();

        // Wait until calibration button has been pressed

        if (calSwitchStatus == 0)
        { // btn has been pressed
            Serial.println("Calibrating stop point");  // only need to say this once per button press
            strip.setPixelColor(YellowLedRange + 1, 0, 0, 128);
            strip.show();
            while (calSwitchStatus == 0)
            {
                //  Ping until button is released
                //Serial.println(calSwitchStatus);

                if (measure.RangeStatus != 4)
                { // We have a valid return value
                    lox.rangingTest(&measure, false);
                    range = measure.RangeMilliMeter;
                    Serial.println(range);
                    if (range != 0) {
                        stopRange = range;
                        sprintf(printBuffer, "stopRange has been set to %d", stopRange);
                    Serial.println(printBuffer);
                    }
                    else {
                        Serial.println("Range is 0.  I need something to calibrate against");
                    }
                    // Anything between 0 and stopRange will be indicated in red section of LED strip
                }
                calSwitchStatus = digitalRead(CalibrationPin);
                //Serial.print ("BUTTON STATE");
                //Serial.println (calSwitchStatus);
                delay(20);
            }
            strip.setPixelColor(YellowLedRange + 1, 128, 128, 128);
            strip.show();
            delay(30);
        }
    }
    else {  // Normal ping and display routine}
            lox.rangingTest(&measure, false);

            if (measure.RangeStatus != 4)
            { // We have a valid return value
                range = measure.RangeMilliMeter;
                sprintf(str, "% 5d", range);
                Serial.println( str);
                if (range > MaxRange)
                    range = MaxRange;

                ledNum = ((uint32_t)range * (PixelCount - 1) / MaxRange); // Normalize range to pixel
                sprintf (printBuffer, "D=%d  LED=%d", range, ledNum);
             
                // Need to adjust so first 1/3 is green, second is yellow, third is red.
            }

            else
            {
                strip.setPixelColor(0, 128, 128, 128);
                // Serial.println("Setting first pixel to white");
                strcpy(str, "  ---");
            }

            delay(30);
            int StartRedZone = 300;
            int StartYellowZone = 600;

            // There's just no stopping in the red zone....

            // Update the strip if new value
            if (ledNum != ledPrevNum)
            {
                strip.clear();
                if (range < StartRedZone)
                {                                           // 1ft
                    strip.setPixelColor(ledNum, 128, 0, 0); // red
                }
                else if (range < StartYellowZone)
                {                                             // 2ft
                    strip.setPixelColor(ledNum, 128, 128, 0); // yellow
                }
                else
                {
                    strip.setPixelColor(ledNum, 0, 128, 0); // green
                }
                ledPrevNum = ledNum;
            }
            strip.show();

            /// delay(1000);
        }
}
