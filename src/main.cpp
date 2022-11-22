
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#include <Wire.h>
#define Console Serial // command processor input/output stream

/// EEprom code and docs are from https://github.com/esp8266/Arduino/tree/master/libraries/EEPROM/examples

const uint16_t PixelCount = 21;
const uint16_t PixelPin = D4;
const uint16_t CalibrationPin = D5; // Used to set yellow/red transition distance
const uint16_t MaxRange = 1200;     // Units mm

int calSwitchStatus = 0; // Init button state to not pressed

// Various states of calibration
const int NOTCALIBRATED = 0;
const int CALIBRATING = 1;
const int CALIBRATED = 2;
const int CALFAILURE = 3;

// 2-byte spacing for integers stored in EEPROM
//  USE CODED VARIABLE, NOT WHAT'S IN EEPROM    int EEPROMADDRSENSOROFFSET = 0;
int EEPROMADDRREDRANGE = 2;
int EEPROMADDRYELRANGE = 4;
int EEPROMADDRGRNRANGE = 6;
int EEPROMADDRMMPERLED = 8;

int calibrationState = NOTCALIBRATED;

int range = -1;
int mmPerLed = -1;
int sensorOffset = 200; // The target must be at least this distance away from the sensor.  We do not want to park with the vehicle on the sensor
int rangeWidth = 0;              // The width of the closest (red) range minus the sensorOffset

// Avoid repetetive messages with Notice booleans
boolean notCalibratedYetNotice = false;
boolean calibratingNotice = false;
boolean calibrationNeedsTargetNotice = false;

int redRange, yellowRange, greenRange, maxSensorRange;
char printBuffer[80];

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PixelCount, PixelPin, NEO_GRB);

// Determine which pixels will be green, yellow, or red
int GreenLedRange = (int)(PixelCount / 3);        // First third is green
int YellowLedRange = GreenLedRange * 2;           // Second third is yellow
int RedLedRange = YellowLedRange + GreenLedRange; // Last third is red

void testStrip()
{
    int pauseTime = 5;
    for (int i = 0; i < PixelCount; i++)
    {
        strip.setPixelColor(i, 32, 32, 32);
        strip.show();
        delay(pauseTime);
    }
    delay(200);
    strip.clear();
}

// EEprom routines for values potentially > 256
void writeIntIntoEEPROM(int address, int number)
{
    EEPROM.write(address, number >> 8);
    EEPROM.write(address + 1, number & 0xFF);
}

int readIntFromEEPROM(int address)
{
    byte byte1 = EEPROM.read(address);
    byte byte2 = EEPROM.read(address + 1);
    return (byte1 << 8) + byte2;
}

// start reading from the first byte (address 0) of the EEPROM
// Leveraged from here: https://roboticsbackend.com/arduino-store-int-into-eeprom/

void setup()
{

    Console.begin(115200);
    while (!Serial)
    {
        delay(1);
    }

    // When the Calibrate button is pushed, we will set the range to the yellow/red transition point
    // This inforation will be stored in pseudo-eeprom on the nodemcu

    pinMode(CalibrationPin, INPUT_PULLUP); // set pin to input
    digitalWrite(CalibrationPin, HIGH);    // turn on pullup resistors

    strip.begin(); //
    strip.clear(); //
    strip.show();  //
    EEPROM.begin(512);

    testStrip();
    delay(500);

    Serial.println("ToF Parking Assistant\n");

    // int loxStartLimit = 100;
    // int loxStartTries = 0;
    // bool loxIsStarted = false;

    //sensorOffset = readIntFromEEPROM(EEPROMADDRSENSOROFFSET);
    redRange = readIntFromEEPROM(EEPROMADDRREDRANGE);
    yellowRange = readIntFromEEPROM(EEPROMADDRYELRANGE);
    greenRange = readIntFromEEPROM(EEPROMADDRGRNRANGE);
    mmPerLed = readIntFromEEPROM(EEPROMADDRMMPERLED);

    sprintf (printBuffer, "EEPROM offset: %d rRange: %d yRange: %d gRange: %d  mmLed: %d", sensorOffset, redRange, yellowRange, greenRange, mmPerLed);
    Serial.println (printBuffer);
    delay (5000);

/*
    if ( (sensorOffset == 255) && (redRange == 255) && (yellowRange == 255) && (greenRange == 255))  // Fresh, unwritten EEPROM
        calibrationState == NOTCALIBRATED;
    else
        calibrationState = CALIBRATED;
*/

    if (!lox.begin())
    {
        delay(50);
        Serial.println(F("Failed to boot VL53L0X ToF Sensor"));
        strip.setPixelColor(2, 64, 0, 0); // 2nd LED: red for lox is not started
        strip.show();
        while (1) // On failure, do nothing forever
            ;
    }
    else
    {
        Serial.println(F("Successfully started VL53L0X ToF Sensor"));
        strip.setPixelColor(2, 0, 64, 0); // next LED: green for lox is  started
        strip.show();
        delay(1000);

        lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
        lox.startRangeContinuous();
    }
}

void ledShowNotCalibrated()
{ // Every other LED is red if calabration has not been done yet.
    for (int i = 0; i <= PixelCount; i = i + 2)
    {
        strip.setPixelColor(i, 128, 0, 0);
    }
    strip.show();
}

void ledShowCalibrationFailure()
{ // Every other LED blue for calibration failure
    for (int i = 0; i <= PixelCount; i = i + 2)
    {
        strip.setPixelColor(i, 0, 0, 128);
    }
    strip.show();
}

boolean targetTooCloseNotice = false;

void doRangeCalibration()
{
    VL53L0X_RangingMeasurementData_t measure;
    Serial.print("Beginning calibration\n");.

    targetTooCloseNotice = false;

    if (calibratingNotice == false) // only need to say this once per button press
    {
        Serial.println("Calibrating ToF Sensor");
        calibratingNotice = true;
    }

    lox.rangingTest(&measure, false); // Request a measurement
    if (measure.RangeStatus != 4)
    { // We have a valid return value
        range = measure.RangeMilliMeter;
        Serial.print("Range=");
        Serial.println(range);

        if (range == 0)
        {
            if (calibrationNeedsTargetNotice == false)
            {
                Serial.println("Nothing in range.  I need something to calibrate against");
                calibrationNeedsTargetNotice = true; // Only need to say this once per button press
            }
        }

        else if (range < sensorOffset)
        {
            Serial.println("The target is too close to the sensor.  Back it up");
            targetTooCloseNotice = true; // Don't keep saying this
        }

        else
        {
            // Calibration explanation
            // When the user holds the button, a series of measurements set the "stop" zone, and record that in mm.
            // We want to support different display mechanisms (neopixel, servo, 7-segment, etc), so latest measurement will
            // always be available in mm

            // For the neopixel strip my implementation uses the last 1/3 of the strip to show (in red) how far you are past the stop point.
            // Might add a TOO CLOSE indicator like blinking light if range is dangerously close to the sensor.
            // TODO  Add an offset.  The vehicle SHOULD never be right on top of the sensor chip!!!

            int sensorDelta = range - sensorOffset;  // Red zone starts at sensorOffset, ends at "range".  This # of mm will be repeated for yellow and green zones

            Serial.print ("Red led range: ");
            Serial.print (sensorOffset);
            Serial.print (" to ");
            Serial.println (range);
            redRange = range;



            // redRange = range - sensorOffset;
            sprintf(printBuffer, "SensorOffset: %d  Range: %d  redRange: %d", sensorOffset, range, redRange);
            Serial.println(printBuffer);
            // delay(10000);

            // Now that we have the red distance, let's extrapolate the "yellow" and "green" distances.
            yellowRange = (sensorDelta * 2) + sensorOffset; // Middle 1/3 of neopixel strip
            greenRange = (sensorDelta * 3) + sensorOffset;  // First 1/3 of neopixel strip
            maxSensorRange = greenRange;                 // Measurements beyond the green range are clipped to first green LED

            Serial.print ("Yellow led range: ");
            Serial.print (redRange);
            Serial.print (" to ");
            Serial.println (yellowRange);
            
            Serial.print ("Green led range: ");
            Serial.print (yellowRange);
            Serial.print (" to ");
            Serial.println (greenRange);

            // Use pixelCount to determine how many mm are represented by each lit pixel
            mmPerLed = (int)((sensorDelta * 3) / PixelCount);

            sprintf(printBuffer, "Calibrated Red=%d Yellow=%d Green=%d  mmPerLed=%d  max=%d", redRange, yellowRange, greenRange, mmPerLed, maxSensorRange);
            Serial.println(printBuffer);

            // Show one yellow and one red LED at the target zone.  This is the stop point
            strip.clear();
            strip.setPixelColor((int)(PixelCount * 2 / 3), 128, 0, 0);
            strip.setPixelColor((int)(PixelCount * 2 / 3) - 1, 128, 128, 0);
            strip.show();
            delay(100);
            // TODO Wait for calibration button to be released, and write ranges and mmPerLed into pseudo-EEPROM
            // Save tghe calibration value into EEPROM when the button is released.  Otherwise keep re-reading
            calSwitchStatus = digitalRead(CalibrationPin);
            if (calSwitchStatus == 1)
            {
                sprintf(printBuffer, "Storing calibration values of %d %d %d %d %d", sensorOffset, redRange, yellowRange, greenRange, mmPerLed);
                Serial.println(printBuffer);


                writeIntIntoEEPROM(EEPROMADDRREDRANGE, redRange);
                writeIntIntoEEPROM(EEPROMADDRYELRANGE, yellowRange);
                writeIntIntoEEPROM(EEPROMADDRGRNRANGE, greenRange);
                writeIntIntoEEPROM(EEPROMADDRMMPERLED, mmPerLed);
                calibrationState = CALIBRATED;
                delay(5000);

                Serial.print("NOW READING BACK VALUES FROM EEPROM");
                redRange = readIntFromEEPROM(EEPROMADDRREDRANGE);
                yellowRange = readIntFromEEPROM(EEPROMADDRYELRANGE);
                greenRange = readIntFromEEPROM(EEPROMADDRGRNRANGE);
                mmPerLed = readIntFromEEPROM(EEPROMADDRMMPERLED);

                sprintf(printBuffer, "READBACK EEPROM offset: %d rRange: %d yRange: %d gRange: %d  mmLed: %d", sensorOffset, redRange, yellowRange, greenRange, mmPerLed);
                Serial.println(printBuffer);
                delay(5000);
            }
        }
    }
}

int oldValue = -1;

void showRangeOnLedStrip(int value)
{
    //  We need to show the inverse of the number of LEDs

    if (value != oldValue)
    {
        oldValue = value;
        Serial.print("Will show ");
        Serial.print(value);
        Serial.println(" lights");

        strip.clear();
        strip.show();

        if (value == 0)
        {
            Serial.print("There should be NO leds active");
            strip.clear();
            strip.show();
        }
        else
        {
            for (int i = 0; i <= value; i++)
            {
                if (i >= (int)(PixelCount * 2 / 3))   // red for upper third
                    strip.setPixelColor(i, 32, 0, 0); // red
                else if (i >= (int)(PixelCount / 3))
                    strip.setPixelColor(i, 32, 32, 0); // yellow for middle third
                else
                    strip.setPixelColor(i, 0, 32, 0); // green
                strip.show();
            }
        }
    }
}

void ledShowCalibrated()
{
    VL53L0X_RangingMeasurementData_t measure;

    delay(200);
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4)
    { // We have a valid return value
        range = measure.RangeMilliMeter;
        if (range > maxSensorRange)
            range = maxSensorRange;
        int ledDistance = (int)((range - sensorOffset) / mmPerLed);
        // Subtract ledDistance from PixelCount, because we're lighting more LEDs as we get closer (smaller ToF value, more lights)
        sprintf(printBuffer, "Range: %d   mmPerLed:%d   Active LEDs: %d", range, mmPerLed, PixelCount - ledDistance);
        Serial.println(printBuffer);
        showRangeOnLedStrip(PixelCount - ledDistance);
    }
    else
    {
        // Serial.println ("Ranging error");
        showRangeOnLedStrip(0);
    }
    // delay (600);
}

int address = 0;

void loop()
{
    // uint16_t ledPrevNum = 0, ledNum = 0, range = 0;

    // Has the calibration button been pressed.  This is either for initial calibration or re-calibraion
    calSwitchStatus = digitalRead(CalibrationPin);
    if (calSwitchStatus == 0)
    {
        calibrationState = CALIBRATING;
        calibratingNotice = false; // Reset to allow messages
    }

    if (calibrationState == NOTCALIBRATED)
    {
        if (notCalibratedYetNotice == false)
        {
            Serial.print("Not calibrated yet");
            ledShowNotCalibrated();
            notCalibratedYetNotice = true;
        }
    }

    else if (calibrationState == CALIBRATING)
    {
        Serial.println("Calibrating");
        doRangeCalibration();
    }
    /*
    if (calSwitchStatus == 0)
    {
        calibrationState = CALIBRATING;
        calibratingNotice = false; // Reset to allow messages
    }
    */

    else if (calibrationState == CALIBRATED)
    {

        /*
            // read a byte from the current address of the EEPROM
            value = EEPROM.read(address);

            Serial.print(address);
            Serial.print("\t");
            Serial.print(value, DEC);
            Serial.println();

            // advance to the next address of the EEPROM
            address = address + 1;

            // there are only 512 bytes of EEPROM, from 0 to 511, so if we're
            // on address 512, wrap around to address 0
            if (address == 512)
            {
                address = 0;
            }

            delay(500);


     */
        ledShowCalibrated(); // Normal ranging routine after calibration
    }
    else
        ledShowCalibrationFailure();
}
