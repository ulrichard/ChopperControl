#ifndef RANGE_SCANNER_H
#define RANGE_SCANNER_H

#include <Servo.h> 
#include <WProgram.h>

class RangeScanner
{
public:
    RangeScanner(uint8_t pinIrDist, uint8_t pinServo);
    
    uint8_t curr_pos() const { return servoPosDeg_; }
    size_t raw_measure() const;
    size_t distance_cm() const;
    
    void begin();
    void moveNextPos();
    void moveToIdlePos();
    void setIdlePos(const uint8_t idlePosDeg);
    void setStepAngle(const uint8_t newStepDeg);
    void setScanningRange(const uint8_t deg);
    void setScanningRange(const uint8_t minPosDeg, const uint8_t maxPosDeg);

private:
    Servo myservo_; 
    const uint8_t pinIrDist_;
    const uint8_t pinServo_;
    uint8_t servoMinPosDeg_;
    uint8_t servoMaxPosDeg_;
    uint8_t servoIdelPosDeg_;
    uint8_t servoStepDeg_;
    uint8_t servoPosDeg_;
    bool   turnClockwise_;
    
};


#endif // RANGE_SCANNER_H
