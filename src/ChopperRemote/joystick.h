#ifndef JOYSTICK_H_INCLUDED
#define JOYSTICK_H_INCLUDED

// boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread.hpp>
// std lib
#include <stdio.h>


/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
class joystick_listener
{
public:
    joystick_listener(const std::string &joystick_device = "/dev/input/js0");
//    joystick_listener(const joystick_listener &rhs);
    ~joystick_listener();

    void operator()(void);

    // joystick state query functions
    bool    BtnPressed(uint8_t nbr) const
    { assert(nbr < 11); boost::lock_guard<boost::mutex> lg(mtx_); return btn_[nbr]; }
    int16_t AxisVal(uint8_t nbr) const
    { assert(nbr < 4);  boost::lock_guard<boost::mutex> lg(mtx_); return ax_[nbr]; }
    const std::string getCmdString() const;

private:
    void read_joystick_status();

    const int joystick_fd_;
    mutable boost::mutex mtx_;
    // joystick state
    bool    btn_[11];
    int16_t ax_[6];

};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A


#endif // JOYSTICK_H_INCLUDED
