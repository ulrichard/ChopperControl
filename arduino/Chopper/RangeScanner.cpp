
#include "RangeScanner.h"


RangeScanner::RangeScanner(uint8_t pinIrDist, uint8_t pinServo)
  : pinIrDist_(pinIrDist), pinServo_(pinServo), servoMinPosDeg_(45), 
    servoMaxPosDeg_(135), servoStepDeg_(10), servoIdelPosDeg_(90), 
    servoPosDeg_(servoIdelPosDeg_), turnClockwise_(false)
{
    
}

void RangeScanner::begin()
{
    myservo_.attach(pinServo_);
}

size_t RangeScanner::raw_measure() const
{
    int analogValue = analogRead(pinIrDist_);
    return analogValue;
}

size_t RangeScanner::distance_cm() const
{
    // thanks to luckylarry.co.uk for the easy calculation
    const float volts  = analogRead(pinIrDist_) * 5.0 / 1024;
    const float dist_m = 65 * pow(volts, -1.1);
    return dist_m * 100;
}
    
void RangeScanner::moveNextPos()
{
    if(servoPosDeg_ < 0 || servoPosDeg_ > 180)
        servoPosDeg_ = 90;
        
    if(turnClockwise_)
    {
        servoPosDeg_ -= servoStepDeg_;
        if(servoPosDeg_ <= servoMinPosDeg_)
            turnClockwise_ = false;
    }
    else
    {
        servoPosDeg_ += servoStepDeg_;
        if(servoPosDeg_ >= servoMaxPosDeg_)
            turnClockwise_ = true;
    }
    myservo_.write(servoPosDeg_);
    //const size_t usec = map(servoPosDeg_, 0, 180, 1000, 2000);
    //myservo_.writeMicroseconds(usec * 16);
}

void RangeScanner::moveToIdlePos()
{
    servoPosDeg_ = servoIdelPosDeg_;
    myservo_.write(servoPosDeg_);
    //const size_t usec = map(servoPosDeg_, 0, 180, 1000, 2000);
    //myservo_.writeMicroseconds(usec * 16);
}

void RangeScanner::setIdlePos(const uint8_t idlePosDeg)
{
    if(idlePosDeg >= 0 && idlePosDeg <= 180)
        servoIdelPosDeg_ = idlePosDeg;
}

void RangeScanner::setStepAngle(const uint8_t newStepDeg)
{
    if(newStepDeg > 0 && newStepDeg <= (servoMaxPosDeg_ - servoMinPosDeg_) / 2)
        servoStepDeg_ = newStepDeg;
}

void RangeScanner::setScanningRange(const uint8_t deg)
{
    const uint8_t halfrng = deg / 2;
    servoMinPosDeg_ = servoIdelPosDeg_ - halfrng;
    servoMaxPosDeg_ = servoIdelPosDeg_ + halfrng;
}

void RangeScanner::setScanningRange(const uint8_t minPosDeg, const uint8_t maxPosDeg)
{
    servoMinPosDeg_ = minPosDeg;
    servoMaxPosDeg_ = maxPosDeg;
}

    
    
