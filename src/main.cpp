
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



int smoothIndex = 0;
int smoothTotal = 0;              // the running total
int smoothAverage = 0; 
const int numReadings = 20;        // Number of readings for smoothing

int readings[numReadings];  //

int calSwitchStatus = 0; // Init button state to not pressed

// Various states of calibration
const int NOTCALIBRATED = 0;
const int CALIBRATING = 1;
const int CALIBRATED = 2;
const int CALFAILURE = 3;

int MAXSENSORRANGE = 900;  //  Limit of reliable ToF measurement

int calibrationState = CALIBRATED;  // need to implement checksum that goes into eeprom to decide if this unit has been calibrated.

int range = -1;
int mmPerLed = -1;
int sensorOffset = 100; // The target must be at least this distance away from the sensor.  We do not want to park with the vehicle on the sensor
int rangeWidth = 0;              // The width of the closest (red) range minus the sensorOffset

// Avoid repetetive messages with Notice booleans
boolean notCalibratedYetNotice = false;
boolean calibratingNotice = false;
boolean calibrationNeedsTargetNotice = false;

int redRange, yellowRange, greenRange;
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


int readToFDistance()
{
    VL53L0X_RangingMeasurementData_t measure;
    delay(100);
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4)
    { // We have a valid return value
        range = measure.RangeMilliMeter;

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
    Serial.println("ToF Parking Assistant\n");


    pinMode(CalibrationPin, INPUT_PULLUP);  // Calibration button sets the STOP zone. 
    digitalWrite(CalibrationPin, HIGH);    // turn on pullup resistor

    strip.begin(); 
    strip.clear(); 
    strip.show();  
    EEPROM.begin(EEPROM_SIZE);


  // initialize the smoothing array to 0
  for (int i = 0; i < numReadings; i++) 
    readings[i] = 0;

    testStrip();
    delay(200);


    readCalibrationValues();  // Load previous values from EEPROM
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
    Serial.println ("Done with Setup routine.  Sensor is configured");
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
    Serial.println("Beginning calibration");

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
            // We want to support different display mechanisms (neopixel, servo, 7-segment, etc), so stop measurement will
            // always be available in mm

            int sensorDelta = range - sensorOffset; // Red zone starts at sensorOffset, ends at "range".  This # of mm will be repeated for yellow and green zones

            //Serial.print(range);
            //Serial.print (" ");
            //delay (100);


            redRange = range;  // This is the stop point.  Anything closer than this is medium red.
            // Now that we have the red distance, let's extrapolate the "yellow" and "green" distances.


            int yellowGreenRange = MAXSENSORRANGE - redRange;  //  We have this many mm between start of reliable readings and red stop point
            yellowRange = redRange + (int)(yellowGreenRange / 2);
           
            greenRange = MAXSENSORRANGE;

            sprintf (printBuffer, "offset: %d  R %d  Y %d  G %d", sensorOffset, redRange, yellowRange, greenRange);
            Serial.println (printBuffer);
            delay(1000);




            // Use pixelCount to determine how many mm are represented by each lit pixel
            mmPerLed = (int)((yellowGreenRange / 2) / PixelCount);

            sprintf(printBuffer, "GY values: Red=%d Yellow=%d Green=%d  mmPerLed=%d  max=%d", redRange, yellowRange, greenRange, mmPerLed);
            Serial.println(printBuffer);
            // Show G/Y/R sequence to indicate calibrating
            strip.clear();
            strip.setPixelColor(0, 32, 0, 0);
            strip.show();
            delay (500);
            strip.setPixelColor(0, 0, 32, 0);
            strip.show();
            delay (500);
            strip.setPixelColor(0, 0, 0, 32);
            strip.show();
            delay (500);
            strip.clear();

            // TODO Wait for calibration button to be released, and write ranges and mmPerLed into pseudo-EEPROM
            // Save tghe calibration value into EEPROM when the button is released.  Otherwise keep re-reading
            calSwitchStatus = digitalRead(CalibrationPin);
            if (calSwitchStatus == 1)
            {
                sprintf(printBuffer, "STORING calibration values of %d %d %d %d %d", sensorOffset, redRange, yellowRange, greenRange, mmPerLed);
                Serial.println(printBuffer);
                saveCalibrationValues();
                calibrationState = CALIBRATED;
                strip.setPixelColor(0, 32, 32, 32);
                strip.show();
                delay(500);
                strip.clear();

                //readCalibrationValues();
                //delay(1000);
            }
        }
}


int oldValue = -1;

// The neopixel strip should really be an object with offset, r/y/g ranges, and mmPerLed. 
// Right now these are globals


// TODO:  ADD LOGIC TO REMOVE OUTLIER DATA POINTS
int smooth(int latestValue)
{

    if (latestValue == -1)
        latestValue = MAXSENSORRANGE;

    
        smoothTotal = smoothTotal - readings[smoothIndex];

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
    
    Serial.print("S: ");
    Serial.println(smoothAverage);
    return (smoothAverage);
}

void showRangeOnLedStrip(int rangeValue) // Value is now range in mm, not # of LEDs
{
    //Serial.print(rangeValue);
if (mmPerLed == 0)  // Safeguard against mmPerLed being 0 by roundoff
        {
            Serial.println("Warning: mmPerLed=0.  Substituting 5 to avoid / 0 error  (large PixelCount?)");
            delay(2000);
            mmPerLed = 5; ///  Avoid / 0 error
                          //  We need to show the inverse of the number of LEDs
                          //                                Below used to be ledDistance
        }



    if (rangeValue == -1)
    { // Either no target or out of range.  Display nothing
        Serial.print("STRIP IS BEING ASKED TO SHOW VALUE OF -1");
        strip.clear();
        strip.show();
    }

    else if (rangeValue < sensorOffset)
    { // Target is dangerously inside the sensorOffset Range
        strip.clear();
        for (int i = 0; i <= PixelCount; i++)
            strip.setPixelColor(i, 64, 64, 64); // white....STOP STOP STOP
        strip.show();
    }

    else if (rangeValue < redRange)
    { // target is between the calibrated stop point and the sensorOffset point.  Every other LED is RED
            strip.clear();
        for (int i = 0; i <= PixelCount; i=i+2)
            strip.setPixelColor(i, 32, 0, 0); // red
        strip.show();
    }

    else if (rangeValue < yellowRange ) //  Turn strip yellow, draw # of pixels
    {
        int ledValue = ((int)((yellowRange - range) / mmPerLed));
        if (ledValue < 0)
            ledValue = 0;

            strip.clear();
            for (int i = 0; i <= ledValue; i++)
                    strip.setPixelColor(i, 32, 32, 0); // strip is now in yellow mode
            for (int i = ledValue; i <= PixelCount; i++)
                strip.setPixelColor(i, 2, 2, 0); // dull yellow for "unlit" LEDs
            strip.show();
        
    }

   else if (rangeValue < greenRange ) //  Turn strip yellow, draw # of pixels
    {
        int ledValue = ((int)((greenRange - range) / mmPerLed));
        if (ledValue < 0)
            ledValue = 0;


            strip.clear();
            for (int i = 0; i <= ledValue; i++)
                    strip.setPixelColor(i, 0, 32, 0); // strip is now in green mode
            for (int i = ledValue; i <= PixelCount; i++)
                strip.setPixelColor(i, 0, 2, 0); // dull green
            strip.show();
    }
    else {
          strip.setPixelColor(0, 0, 0, 32); // Error or nothing in range
          strip.show();
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
            // Should have some X millisecond minimum delay here based on 8266 RTC
            //  Ranging pings could increase as target gets closer

            int distanceToTarget = readToFDistance();
            if (distanceToTarget != -1) { 
                int smoothedDistance= smooth(distanceToTarget);
                //  Serial.print(" S:");
                //  Serial.println(smoothedDistance);
                showRangeOnLedStrip(smoothedDistance);
            }
            else {
                strip.clear();
                strip.setPixelColor (0, 0,0,12);
                strip.show();
            }
        }
        else
            ledShowCalibrationFailure();
    }

    //#include <EEPROM.h>//https://github.com/esp8266/Arduino/blob/master/libraries/EEPROM/EEPROM.h