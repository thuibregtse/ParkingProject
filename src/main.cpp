
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#include <Wire.h>
#define Console Serial // command processor input/output stream
#define EEPROM_SIZE 18
/// EEprom code and docs are from https://github.com/esp8266/Arduino/tree/master/libraries/EEPROM/examples

const uint16_t PixelCount = 30;
const uint16_t PixelPin = D3; 
const uint16_t CalibrationPin = D0; // Used to set yellow/red transition distance
const uint16_t MaxRange = 1200;     // Units mm


int smoothIndex = 0;
int smoothTotal = 0;              // the running total
int smoothAverage = 0; 
const int numReadings = 10;        // Number of readings to average

int readings[numReadings];  //

int calSwitchStatus = 0; // Init button state to not pressed

// Various states of calibration
const int NOTCALIBRATED = 0;
const int CALIBRATING = 1;
const int CALIBRATED = 2;
const int CALFAILURE = 3;


int calibrationState = CALIBRATED;  // need to implement checksum that goes into eeprom to decide if this unit has been calibrated.

int range = -1;
int mmPerLed = -1;
int sensorOffset = 300; // The target must be at least this distance away from the sensor.  We do not want to park with the vehicle on the sensor
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
    int pauseTime = 300;
    for (int i = 0; i < PixelCount; i++)
        strip.setPixelColor(i, 0, 32, 0);
    strip.show();
    delay(pauseTime);
    strip.clear();
}


void saveCalibrationValues()
{
    int address = 0;
    EEPROM.put(address, sensorOffset);
    address += sizeof(sensorOffset); 
    EEPROM.put(address, redRange);
    address += sizeof(redRange); 
    EEPROM.put(address, yellowRange);
    address += sizeof(yellowRange); 
    EEPROM.put(address, greenRange);
    address += sizeof(greenRange); 
    Serial.print ("DEBUG: I AM SAVING A mmPerLed OF ");
    Serial.println (mmPerLed);
    EEPROM.put(address, mmPerLed);
    address += sizeof(mmPerLed); 
    EEPROM.commit();
}

void readCalibrationValues()
{
    int address = 0;
    EEPROM.put(address, sensorOffset);
    address += sizeof(sensorOffset); 
    EEPROM.get(address, redRange);
    address += sizeof(redRange); 
    EEPROM.get(address, yellowRange);
    address += sizeof(yellowRange); 
    EEPROM.get(address, greenRange);
    address += sizeof(greenRange); 
    EEPROM.get(address, mmPerLed);
    Serial.print ("DEBUG: I READ BACK A mmPerLed OF ");
    Serial.println (mmPerLed);
    address += sizeof(mmPerLed); 
    sprintf(printBuffer, "readCalibrationValues returned offset: %d r: %d y: %d g: %d  mmPerLed: %d", sensorOffset, redRange, yellowRange, greenRange, mmPerLed);
    Serial.println(printBuffer);

    //EEPROM.end();
}

// start reading from the first byte (address 0) of the EEPROM
// Leveraged from here: https://roboticsbackend.com/arduino-store-int-into-eeprom/


int readToFDistance()
{
    VL53L0X_RangingMeasurementData_t measure;
    delay(100);
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4)
    { // We have a valid return value
        range = measure.RangeMilliMeter;
///Serial.println(range);
        /*
        if (range > maxSensorRange)
            range = maxSensorRange;
            */
        return (range);
    }
    else
        return (-1);
}



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

    strip.begin(); 
    strip.clear(); 
    strip.show();  
    EEPROM.begin(EEPROM_SIZE);


  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

    testStrip();
    delay(200);

    Serial.println("ToF Parking Assistant\n");

    readCalibrationValues();
    // TODO:  Look for valid checksum of calibration info
    calibrationState = CALIBRATED;
    sprintf (printBuffer, "IN SETUP: EEPROM offset: %d rRange: %d yRange: %d gRange: %d  mmLed: %d", sensorOffset, redRange, yellowRange, greenRange, mmPerLed);
    Serial.println (printBuffer);
    delay (400);


    if (!lox.begin())
    {
        delay(20);
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
    Serial.print ("      LEAVING THE SETUP ROUTINE.  SENSOR IS CONFIGURED");
}

void ledShowNotCalibrated()
{ // Every other LED is red if calabration has not been done yet.  Eventually make this a method for a distanceStrip object
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
    Serial.print("Beginning calibration\n");

    targetTooCloseNotice = false;

    if (calibratingNotice == false) // only need to say this once per button press
    {
        Serial.println("Calibrating ToF Sensor");
        calibratingNotice = true;
    }

    int range = readToFDistance();

    if (range > -1)

        if (range == 0)
        {
            if (calibrationNeedsTargetNotice == false)
            {
                Serial.println("Range = 0; Nothing in range.  I need something to calibrate against");
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

            int sensorDelta = range - sensorOffset; // Red zone starts at sensorOffset, ends at "range".  This # of mm will be repeated for yellow and green zones
            Serial.println("BEGINNING CALIBRATION   ");
            ///Serial.print("Red LEDs will start at: ");
            Serial.print(range);
            Serial.print (" ");
 

            delay (100);


            redRange = range;
            // Now that we have the red distance, let's extrapolate the "yellow" and "green" distances.
            yellowRange = (sensorDelta * 2) + sensorOffset; // Middle 1/3 of neopixel strip
            greenRange = (sensorDelta * 3) + sensorOffset;  // First 1/3 of neopixel strip
            maxSensorRange = greenRange;                    // Measurements beyond the green range are clipped to first green LED

            // Use pixelCount to determine how many mm are represented by each lit pixel
            mmPerLed = (int)((sensorDelta * 3) / PixelCount);

            sprintf(printBuffer, "New values: Red=%d Yellow=%d Green=%d  mmPerLed=%d  max=%d", redRange, yellowRange, greenRange, mmPerLed, maxSensorRange);
            Serial.println(printBuffer);

            // Show one yellow and one red LED at the target zone.  This is the stop point
            strip.clear();
            strip.setPixelColor((int)(PixelCount * 2 / 3), 32, 0, 0);
            strip.setPixelColor((int)(PixelCount * 2 / 3) - 1, 32, 32, 0);
            strip.show();
            delay(100);
            // TODO Wait for calibration button to be released, and write ranges and mmPerLed into pseudo-EEPROM
            // Save tghe calibration value into EEPROM when the button is released.  Otherwise keep re-reading
            calSwitchStatus = digitalRead(CalibrationPin);
            if (calSwitchStatus == 1)
            {
                sprintf(printBuffer, "STORING calibration values of %d %d %d %d %d", sensorOffset, redRange, yellowRange, greenRange, mmPerLed);
                Serial.println(printBuffer);
                saveCalibrationValues();
                calibrationState = CALIBRATED;
                delay(500);
                readCalibrationValues();
                delay(1000);
            }
        }
}


int oldValue = -1;

// The neopixel strip should really be an object with offset, r/y/g ranges, and mmPerLed. 
// Right now these are globals


// TODO:  ADD LOGIC TO REMOVE OUTLIER DATA POINTS
int smooth(int latestValue)
{

    smoothTotal = smoothTotal - readings[smoothIndex];
    // read from the sensor:
    //if (latestValue < 0)
    //    latestValue = 0;
    readings[smoothIndex] = latestValue;
    smoothTotal = smoothTotal + readings[smoothIndex];
    // advance to the next position in the array:
    smoothIndex = smoothIndex + 1;
    // if we're at the end of the array...
    if (smoothIndex >= numReadings)
    {
        // ...wrap around to the beginning:
        smoothIndex = 0;
    }
    // calculate the average:
    smoothAverage = smoothTotal / numReadings;
    return (smoothAverage);
}

void showRangeOnLedStrip(int rangeValue) // Value is now range in mm, not # of LEDs
{
    Serial.print(rangeValue);

    if (rangeValue == -1)
    { // Either no target or out of range.  Display nothing
       // Serial.print("STRIP IS BEING ASKED TO SHOW VALUE OF -1");
        strip.clear();
        strip.show();
    }

    else if (rangeValue < sensorOffset)
    { // Target is dangerously inside the sensorOffset Range
        for (int i = 0; i <= PixelCount; i++)
            strip.setPixelColor(i, 32, 0, 0); // red
        strip.show();
    }

    else if (rangeValue != oldValue) //  Draw valid value in G/Y/R range on strip
    {
        Serial.println("(New) ");
        oldValue = rangeValue;

        if (mmPerLed == 0)
        {
            Serial.println("Warning: mmPerLed=0.  Substituting 5 to avoid / 0 error  (large PixelCount?)");
            delay(2000);
            mmPerLed = 5; ///  Avoid / 0 error
                          //  We need to show the inverse of the number of LEDs
                          //                                Below used to be ledDistance
        }
        int ledValue = (PixelCount - (int)((range - sensorOffset) / mmPerLed));

        if (ledValue < 0)
            ledValue = 0;

        Serial.print("Range: ");
        Serial.print(rangeValue);
        Serial.print(" LED: ");
        Serial.println(ledValue);

        // strip.clear();
        // strip.show();

        if (ledValue == 0)
        {
            Serial.print("There should be NO leds active");
            strip.clear();
            strip.show();
        }
        else // BELOW RANGES SHOULD BE CONFIGURABLE, NO HARD-CODED
        {
            for (int i = 0; i <= ledValue; i++)
            {
                if (i >= (int)(PixelCount * 2 / 3))   // red for upper third
                    strip.setPixelColor(i, 32, 0, 0); // red
                else if (i >= (int)(PixelCount / 3))
                    strip.setPixelColor(i, 32, 32, 0); // yellow for middle third
                else
                    strip.setPixelColor(i, 0, 32, 0); // green
                strip.show();
            }
            for (int i = ledValue; i <= PixelCount; i++)
                strip.setPixelColor(i, 1, 1, 1); // dull white
        }
    }
}

    int address = 0;

    void loop()
    {
// TODO  Add routine that checks if calibration values have been saved in EEPROM.
//       This will probably be a saved checksum of other saved values

        // Has the calibration button been pressed.  This is either for initial calibration or re-calibraion
        calSwitchStatus = digitalRead(CalibrationPin);
        if (calSwitchStatus == 0)
        {
            calibrationState = CALIBRATING;
            calibratingNotice = false; //  Soon, set this to True to stop repeating messages
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
        else if (calibrationState == CALIBRATED)
        {
            // Should have some X millisecond minimum delay here.
            //  Ranging pings could increase as target gets closer

            int distanceToTarget = readToFDistance();
            // Experimental smoothing function
            //Serial.print ("distanceToTarget: ");
            //Serial.print (distanceToTarget);

            int smoothedDistance= smooth(distanceToTarget);
            Serial.print(" S:");
            Serial.println(smoothedDistance);
            showRangeOnLedStrip(smoothedDistance);
        }
        else
            ledShowCalibrationFailure();
    }

    //#include <EEPROM.h>//https://github.com/esp8266/Arduino/blob/master/libraries/EEPROM/EEPROM.h