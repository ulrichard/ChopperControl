/*
Chopper control. Replaces the receiver unit of an rc helicopter.
At first it provieds sensor data to a remote controlling unit while 
executing the commands received over the bluetooth link.
*/
#define USE_COMPASS
#define USE_ACCELERO
//#define USE_BARO
#define USE_DRIVEMOTOR
#define USE_RANGE_SCANNER
#define USE_ULTRASND_AGL
//#define USE_HELP_MSG


#include <Servo.h> 
#include <Wire.h>
#ifdef USE_HELP_MSG
  #include <avr/pgmspace.h>
#endif
#include "Streaming.h"
#ifdef USE_RANGE_SCANNER
  #include "RangeScanner.h"
#endif
#ifdef USE_BARO
  #define DISABLE_MS5534C
  #include "IntersemaBaro.h"
#endif
#ifdef USE_COMPASS
  #include "PID_Beta6.h"
#endif

// analog pins
static const uint8_t pinUSDist     =   2; // Ultrasonic distance sensor to look down for the height above ground.
static const uint8_t pinIRDist     =   3; // Infrared distance sensor on the sweeping servo to look for obstacles.
//                                     4  // i2c SDA
//                                     5  // i2c SCL
// digital pins
//                                     0  // rx serial over bluetooth
//                                     1  // tx serial over bluetooth
static const uint8_t pinSensorPwr  =   2; // power for compass, accelerometer and barometer
//                                     3  // 
//                                     4; // 
static const uint8_t pinMotorUpper =   5; // upper rotor throttle -> needs to be PWM
static const uint8_t pinMotorLower =   6; // lower rotor throttle -> needs to be PWM
//                                     7  // bluetooth reset, don't use!!!
//                                     8; // 
//                                     9; // 
static const uint8_t pinServoIrRng =  10; // servo for swaping the ir proximity sensor
//                                    11  // 
static const uint8_t pinServoFB    =  12; // servo for forward backward tilt
static const uint8_t pinServoLR    =  13; // servo for left right tilt
// i2c addresses
static const uint8_t addrCompas    = (0x42 >> 1);
static const uint8_t addrNunchuck  =  0x52;

#ifdef USE_RANGE_SCANNER
  RangeScanner  rangescn(pinIRDist, pinServoIrRng);
#endif
#ifdef USE_BARO
  Intersema::BaroPressure_MS5534B baro(false);
#endif
#ifdef USE_DRIVEMOTOR
  Servo srvFrontBack;
  Servo srvLeftRight;
#endif
#ifdef USE_ULTRASND_AGL
  double aglSetpoint, aglMeas, aglPidOutput;
  PID pidAGL(&aglMeas, &aglPidOutput, &aglSetpoint, 2, 5, 1);
#endif
#ifdef USE_COMPASS
  double pidSetpoint, pidInput, pidOutput;
  PID pidCompas(&pidInput, &pidOutput, &pidSetpoint, 2, 5, 1);
#endif
bool          doSendSerial    = false;
bool          doSweepServo    = false;
size_t        loopDelay       = 100; // [ms]
uint8_t       targetHeading   = 0;   // [°]
uint16_t      targetGndHeight = 0;   // [cm]
uint8_t       motorSpeed      = 0;   // [0 - 255]
uint32_t      loopCounter     = 0;   //

size_t evalSerialCmd();
void   evalCmdStr(char *txt);
 
void setup() 
{
    pinMode(pinSensorPwr, OUTPUT); 
    digitalWrite(pinSensorPwr, LOW);  
    delay(200);      
    digitalWrite(pinSensorPwr, HIGH);  
    delay(200);
    Serial.begin(115200); // arduino bt has only this baud rate
    Wire.begin();         // join i2c bus as master
#ifdef USE_DRIVEMOTOR
    pinMode(pinMotorUpper, OUTPUT); 
    pinMode(pinMotorLower, OUTPUT); 
    srvFrontBack.attach(pinServoFB);
    srvLeftRight.attach(pinServoLR);
#endif
#ifdef USE_RANGE_SCANNER
    rangescn.begin();
    rangescn.setStepAngle(5); 
    rangescn.setScanningRange(160);
    rangescn.moveToIdlePos();
#endif
#ifdef USE_ACCELERO
    Wire.beginTransmission(addrNunchuck);// transmit to device 0x52
    Wire.send(0x40);// sends memory address
    Wire.send(0x00);// sends a zero, requesting the first data
    Wire.endTransmission();// stop transmitting    
    delay(100);
#endif
#ifdef USE_BARO
    baro.begin();
#endif
#ifdef USE_ULTRASND_AGL 
    aglSetpoint = 0.0; // we take off from the ground
    pidAGL.SetInputLimits (0, 600); // height above ground in cm [0 - 600cm with 2.5cm resolution]
    pidAGL.SetOutputLimits(0, 255); // 8 bit PWM resolution
    pidAGL.SetMode(AUTO);
#endif
#ifdef USE_COMPASS
    pidSetpoint = 0;
    pidCompas.SetInputLimits (-180.0, 180.0);
    pidCompas.SetOutputLimits(   0.0,  40.0);
    pidCompas.SetMode(AUTO);
#endif
}
 
void loop() 
{
    const long loopStart = millis();
    ++loopCounter;
    
#ifdef USE_RANGE_SCANNER
    const uint8_t proximityAngle = rangescn.curr_pos();
    const int     proximityValue = rangescn.raw_measure();
    if(doSweepServo)
        rangescn.moveNextPos(); // move to the next scan position
    if(doSendSerial)
        Serial << "RA" << _DEC(proximityAngle) << "RV" << _DEC(proximityValue); 
#endif

#ifdef USE_ACCELERO
    int accelX = 0, accelY = 0, accelZ = 0;
    {
        Wire.requestFrom(addrNunchuck, static_cast<uint8_t>(6));
        int cnt=0;
        uint8_t buf[6];
        while(Wire.available() && cnt < 6) 
        {
            char cc = Wire.receive();
            cc = (cc ^ 0x17) + 0x17;
            buf[cnt++] = cc;
        }
    
        accelX = buf[2];
        accelX *= 4;
        if((buf[5] >> 2) & 1) 
            accelX += 2;
        if((buf[5] >> 3) & 1)
            accelX += 1;
        accelY = buf[3];
        accelY *= 4;
        if((buf[5] >> 4) & 1) 
            accelY += 2;
        if((buf[5] >> 5) & 1)
            accelY += 1;
        accelZ = buf[4];
        accelZ *= 4;
        if((buf[5] >> 6) & 1) 
            accelZ += 2;
        if((buf[5] >> 7) & 1)
            accelZ += 1;
        
        if(doSendSerial)
            Serial << "AX" << _DEC(accelX) << "AY" << _DEC(accelY) << "AZ" << _DEC(accelZ);
    }
#endif

#ifdef USE_COMPASS
    Wire.beginTransmission(addrCompas);
    Wire.send('A');  // command sensor to measure angle  
    Wire.endTransmission();  
    
    // step 2: wait for readings to happen 
    delay(10);                        // datasheet suggests at least 6000 microseconds 
  
    // step 3: request reading from sensor 
    Wire.requestFrom(addrCompas, static_cast<uint8_t>(2));       // request 2 bytes from slave device #33 
 
    // step 4: receive reading from sensor 
    int compassHeading = 0;
    if(Wire.available() >= 2)         // if two bytes were received 
    { 
        compassHeading = Wire.receive(); // receive high byte (overwrites previous reading) 
        compassHeading = compassHeading << 8;       // shift high byte to be high 8 bits 
        compassHeading += Wire.receive();    // receive low byte as lower 8 bits 
        compassHeading /= 10;
        compassHeading -= 90;  // mounting position on the board
        while(compassHeading < 0)
            compassHeading += 360;
        while(compassHeading > 360)
            compassHeading -= 360;
        if(doSendSerial)
            Serial << "CH" << _DEC(compassHeading);
    } 
#endif

#ifdef USE_BARO
    const uint32_t altitude = baro.getHeightCentiMeters();
    if(doSendSerial)
        Serial << "BH" << _DEC(altitude);
#endif

#ifdef USE_ACCELERO
    Wire.beginTransmission(addrNunchuck);// transmit to device 0x52
    Wire.send(0x00);// sends one byte
    Wire.endTransmission();// stop transmitting
#endif
    
    while(Serial.available())
        evalSerialCmd();
        
#ifdef USE_ULTRASND_AGL
    if(targetGndHeight)
    {
        // read the ultrasonic distance sensor to find out the height above ground
        const int usAGLval = analogRead(pinUSDist);
        if(doSendSerial)
            Serial << "AG" << _DEC(usAGLval); 
        
        aglMeas = usAGLval * 5.0 * 200 * 1.296 / 1024;
        aglSetpoint = targetGndHeight;
        pidAGL.Compute();
        motorSpeed = aglPidOutput;
    }
#endif
        
#ifdef USE_DRIVEMOTOR
    if(targetGndHeight)
    {
      
    }

    if(motorSpeed < 10)
    {
        analogWrite(pinMotorLower, 0);
        analogWrite(pinMotorUpper, 0);
    }
    else
    {
#ifndef USE_COMPASS
        analogWrite(pinMotorLower, motorSpeed);
        analogWrite(pinMotorUpper, motorSpeed);
#else // USE_COMPASS
        // calculate the speed for the two rotors with heading correction
        int headingDeviation  = targetHeading - compassHeading;
        while(headingDeviation < -180)
            headingDeviation += 360;
        while(headingDeviation > 180)
            headingDeviation -= 360;
    
        pidInput = headingDeviation;
        pidCompas.Compute();
        const int throttleDeviation = pidOutput - 20;
        int throttleLower = motorSpeed - throttleDeviation;
        throttleLower = max(throttleLower, 0);
        throttleLower = min(throttleLower, 255);
        int throttleUpper = motorSpeed + throttleDeviation;
        throttleUpper = max(throttleUpper, 0);
        throttleUpper = min(throttleUpper, 255);

        analogWrite(pinMotorLower, throttleLower);
        analogWrite(pinMotorUpper, throttleUpper);
        
        if(doSendSerial)
            Serial << "ML" << _DEC(throttleLower)
                   << "MU" << _DEC(throttleUpper)
                   << "HD" << _DEC(headingDeviation)
                   << "TD" << _DEC(throttleDeviation);
#endif // USE_COMPASS
    }
#endif // USE_DRIVEMOTOR
    
    if(doSendSerial)
    {
        Serial << "AM" << _DEC(available_memory())
               << "LT" << _DEC(millis() - loopStart)
               << "\n";
    }
    
    const size_t dela = (millis() - loopStart > loopDelay ? 0 : loopDelay - (millis() - loopStart));
    delay(dela); 
    
}

size_t evalSerialCmd()
{
    char tmptxt[32];
    const uint8_t bufsize = sizeof(tmptxt); 
  
    if(!Serial.available()) 
        return 0;

    delay(5);  // wait a little for serial data

    memset(tmptxt, 0, bufsize); // set it all to zero
    size_t i = 0;
    size_t waitCycles = 0;
    while(i < bufsize) 
    {
        if(Serial.available())
        {
            char cc = Serial.read();
            if(cc == '\n' || cc == '\r' || cc == '\0')
                break;
            tmptxt[i++] = cc;
        }
        else if(waitCycles++ > 20)
            break;
        else
            delay(10);       
    }
    tmptxt[min(i, bufsize-1)] = '\0';  // terminate the string
    
    if(i > 2)
        evalCmdStr(tmptxt);
    
    return i;  // return number of chars read
}

void evalCmdStr(char *txt)
{  
    if(!strcmp(txt, "lon"))                 // logging on
        doSendSerial = true;
    else if(!strcmp(txt, "loff"))           // logging off
        doSendSerial = false;
#ifdef USE_DRIVEMOTOR
    else if(!strncmp(txt, "th", 2))         // target heading
        targetHeading = atoi(txt + 2);
    else if(!strncmp(txt, "gh", 2))         // target height above ground
        targetGndHeight = atoi(txt + 2);
    else if(!strncmp(txt, "ms", 2))         // motor speed
        motorSpeed = atoi(txt + 2);
    else if(!strncmp(txt, "fb", 2))         // front back tilt
        srvFrontBack.write(atoi(txt + 2));
    else if(!strncmp(txt, "lr", 2))         // left right tilt
        srvLeftRight.write(atoi(txt + 2)); 
#endif
#ifdef USE_RANGE_SCANNER
    else if(!strcmp(txt, "swon"))           // swap on
        doSweepServo = true;
    else if(!strcmp(txt, "swoff"))          // swap off
    {
        doSweepServo = false;
        rangescn.moveToIdlePos();
    }
    else if(!strncmp(txt, "swst", 4))      // swap step
        rangescn.setStepAngle(atoi(txt + 4));
    else if(!strncmp(txt, "swrg", 4))      // swap range
        rangescn.setScanningRange(atoi(txt + 4));
#endif
    else if(!strncmp(txt, "lode", 4))      // loop delay
    {
        int newval = atoi(txt + 4);
        if(newval >= 10 && newval <= 1000)
            loopDelay = newval;
    }
    else
    {
#ifdef USE_HELP_MSG
        prog_uchar helpstr[] PROGMEM =
            "\nIrServoRangeScanner\n"
            "'lon' send serial data\n"
            "'loff' stop serial data\n"
            "'swon' moving ir proximity sensor\n"
            "'swoff' stop moving ir proximity sensor\n"
            "'swst<nn>' angle deg per step\n"
            "'swrg<nn>' scanning range\n"
            "'th<nn>' desired compass heading\n"
            "'ms<nn>' motor speed for the rotors\n"
            "'fb<nn>' servo forward backward tilt\n"
            "'lr<nn>' servo left right tilt\n"
            "'lode<nn>' millisecs to sleep on loop\n"
            "'?'  for this help msg\n\n"
            ; 
        prog_uchar *cptr = helpstr;
        char cc;
        while(cc = pgm_read_byte(cptr++))
             Serial.print(cc, BYTE);
#endif
        
        if(strlen(txt) && strcmp(txt, "?"))
        {
            Serial.print("Invalid option: \"");
            Serial.print(txt);
            Serial.println("\"");
        }
    }    
}
 
int available_memory()
{
    char stack = 1;
    extern char* __brkval;
    extern char* __malloc_heap_end;
    extern size_t __malloc_margin;
    if(__malloc_heap_end)
        return __malloc_heap_end - __brkval;
    return (&stack - __brkval) - __malloc_margin; 
}
