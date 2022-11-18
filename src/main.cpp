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

const uint16_t PixelCount = 21;
const uint16_t PixelPin = D4;
const uint16_t CalibrationPin = D5;  // Used to set yellow/red transition distance
const uint16_t MaxRange = 1200;  // Units mm

int calSwitchStatus = 0; // Init button state to not pressed

int mmPerLed = -1;
// Various states of calibration


const int NOTCALIBRATED = 0;
const int CALIBRATING = 1;
const int CALIBRATED = 2;
const int CALFAILURE = 3;

int calibrationState = NOTCALIBRATED;

int range = -1;

boolean notCalibratedYetNotice = false;
boolean calibratingNotice = false;
boolean calibrationNeedsTargetNotice = false;



char printBuffer[80];
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(PixelCount, PixelPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PixelCount, PixelPin, NEO_GRB);

//int GreenLedRange = (int) PixelCount * 33;
//int YellowLedRange = (int) PixelCount * 0.66;

int GreenLedRange = (int)(PixelCount / 3); // First third is green
int YellowLedRange = GreenLedRange * 2;    // Second third is yellow
int RedLedRange = YellowLedRange + GreenLedRange;  // Last third is red

// Strip is divided into 3 equal sizes, green, yellow, and red
//  "Stop" zone is when you are in the red area



void testStrip()
{
int pauseTime = 10;
    for (int i = 0; i < GreenLedRange; i++)
    {
        strip.setPixelColor(i, 0, 64, 0);
        strip.show();
        delay (pauseTime);
    }
    for (int i = GreenLedRange; i < YellowLedRange; i++)
    {
        strip.setPixelColor(i, 0, 64, 64);
        strip.show();
        delay (pauseTime);
    }
    for (int i = YellowLedRange; i < PixelCount; i++)
    {
        strip.setPixelColor(i, 64, 0, 0);
        strip.show();
        delay (pauseTime);
    }
    delay(200);
    strip.clear();
}

void setup()
{

    // CalibrationPin input will be used to set the stop distance
    pinMode(CalibrationPin, INPUT_PULLUP);     // set pin to input
    digitalWrite(CalibrationPin, HIGH); // turn on pullup resistors

    Console.begin(115200);
    while (!Serial)
    {
        delay(1);
    }

    Serial.print ("Initial greenLedRange: ");
    Serial.println (GreenLedRange);

    Serial.print ("Initial yellowLedRange: ");
    Serial.println (YellowLedRange);


    strip.begin(); //
    strip.clear(); //
    strip.show();  //

    testStrip();

    delay(500);

    Console.println(F("VL53L0X Parking guide with ESP8266\n"));

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




void ledShowNotCalibrated() {
    for (int i=0; i<= PixelCount; i=i+2) {
        strip.setPixelColor(i, 128, 0, 0);
    }
    strip.show();
}

void ledShowCalibrationFailure () {
    for (int i=0; i<= PixelCount; i=i+2) {
        strip.setPixelColor(i, 0, 0, 128);  // All blue for calibration failure
    }
    strip.show();
}

int redRange, yellowRange, greenRange, maxSensorRange;

void doRangeCalibration()
{
    VL53L0X_RangingMeasurementData_t measure;
    if (calibratingNotice == false)
    {
        Serial.println("Calibrating stop point"); // only need to say this once per button press
        calibratingNotice = true;
    }

    lox.rangingTest(&measure, false); // Request a measurement
    if (measure.RangeStatus != 4)
    { // We have a valid return value
        range = measure.RangeMilliMeter;
        if (range == 0)
        {
            Serial.println("Nothing in range.  I need something to calibrate against");
            calibrationNeedsTargetNotice = true; // Only need to say this once per button press
        }
        else
        {
            // Calibration explanation
            // When the user holds the button, a series of measurements set the "stop" zone, and record that in mm.
            // We want to support different display mechanimsm (neopixel, servo, 7-segment, etc), so latest measurement will
            // always be available in mm

            // For the neopixel strip my implementation uses the last 1/3 of the strip to show (in red) how far you are past the stop point.
            // Might add a TOO CLOSE indicator if range is dangerously close to the sensor.

            redRange = range;
            // So, let's take the calibration distance, and extrapolate the "green" and "yellow" distances.

            yellowRange = range * 2; // Middle 1/3 of neopixel strip
            greenRange = range * 3;  // First 1/3 of neopixel strip
            maxSensorRange = greenRange;  // Measurements beyond this are clipped to first green LED

            // Now let's figure out how many mm/LED so we can scale for the display

           // Serial.print("I have this many LEDs:");
           // Serial.print(PixelCount / 3);
           // Serial.print(" to show the range ");
           // Serial.print(redRange);

            mmPerLed = (int)(redRange / (PixelCount / 3));

            sprintf(printBuffer, "Calibrated Red=%d Yellow=%d Green=%d  mmPerLed=%d  max=%d", redRange, yellowRange, greenRange, mmPerLed, maxSensorRange);
            Serial.println(printBuffer);
        }
        calibrationState = CALIBRATED;
    }
}

int oldValue = -1;

void showRangeOnLedStrip(int value)
{
    //  We need to show the inverse of the number of LEDs

    if (value != oldValue)
    {
        oldValue = value;
        int lightCount = PixelCount - value;
        Serial.print("Will show ");
        Serial.print(lightCount);
        Serial.println("lights");

        strip.clear();
        for (int i = 0; i < lightCount; i++)
        {
            if (i > 14)
            {
                strip.setPixelColor(i, 32, 0, 0); // dimmer white
                strip.show();
            }
            else if (i > 7)
            {
                strip.setPixelColor(i, 32, 32, 0); // dimmer white
                strip.show();
            }
            else
            {
                strip.setPixelColor(i, 0, 32, 0); // dimmer white
                strip.show();
            }
        }
    }
}

void ledShowCalibrated() {
    VL53L0X_RangingMeasurementData_t measure;

    delay(200);
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) { // We have a valid return value  
        range = measure.RangeMilliMeter;
        if (range > maxSensorRange) 
            range = maxSensorRange;
        Serial.print(range);
        Serial.print ("  ");
        int ledDistance = (int)(range / mmPerLed);
        sprintf (printBuffer, "Range: %d   mmPerLed:%d   Active LEDs: %d", range, mmPerLed, ledDistance);
        Serial.println (printBuffer);
        showRangeOnLedStrip(ledDistance);
    }
    else
        Serial.println ("Ranging error");
    //delay (600);

}





void loop()
{
    //uint16_t ledPrevNum = 0, ledNum = 0, range = 0;
    //char str[12];
  
    //range++;
    //ledNum++;
    //ledPrevNum++;

    //strip.setPixelColor(0, 64, 64, 64); // white
    //strip.show();
    //delay(90);
    ///strip.setPixelColor(0, 32, 32, 32); // dimmer white
    ///strip.show();


// Has the calibration button been pressed
    calSwitchStatus = digitalRead (CalibrationPin);
    if (calSwitchStatus == 0) {
        calibrationState = CALIBRATING;
        calibratingNotice = false;  // Reset to allow messages
    }

    if (calibrationState == NOTCALIBRATED) {
        if (notCalibratedYetNotice == false) {
            Serial.print ("Not calibrated yet");
            ledShowNotCalibrated();
            notCalibratedYetNotice = true;
        }
    }

    else if (calibrationState == CALIBRATING) {
        Serial.println ("Calibrating");
        doRangeCalibration();
        delay (1000);
    }
    else if (calibrationState == CALIBRATED)
        ledShowCalibrated();  // Normal ranging routine after calibration
    else
        ledShowCalibrationFailure();


//if (oldCalSwitchStatus != calSwitchStatus) {
//    sprintf (printBuffer, "BUTTON STATE went from %d to %d", oldCalSwitchStatus, calSwitchStatus);
//    Serial.print (printBuffer);
//}
//    delay (100);


/*

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
        */
}
