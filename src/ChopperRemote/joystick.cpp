
// chopper
#include "joystick.h"
// std lib
#include <iostream>
#include <sstream>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
joystick_listener::joystick_listener(const std::string &joystick_device)
    : joystick_fd_(open(joystick_device.c_str(), O_RDONLY | O_NONBLOCK))
{
	if(joystick_fd_ < 0)
        throw std::runtime_error("failed to open joystick " + joystick_device);

    for(size_t i=0; i<11; ++i)
        btn_[i] = false;
    for(size_t i=0; i<6; ++i)
        ax_[i] = 0;
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
/*
joystick_listener::joystick_listener(const joystick_listener &rhs)
    : joystick_fd_(rhs.joystick_fd_)
{
    for(size_t i=0; i<11; ++i)
        btn_[i] = false;
    for(size_t i=0; i<6; ++i)
        ax_[i] = 0;
}
*/
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
joystick_listener::~joystick_listener()
{
//    close(joystick_fd_);
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void joystick_listener::operator()(void)
{
    while("not crashed")
    {
        read_joystick_status();
        usleep(1000);
    }
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
struct js_event
{
	unsigned int time;	    // event timestamp in milliseconds
	short value;            // value
	unsigned char type;     // event type
	unsigned char number;   // axis/button number
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void joystick_listener::read_joystick_status()
{
    static const size_t JS_EVENT_BUTTON = 0x01;    // button pressed/released
    static const size_t JS_EVENT_AXIS   = 0x02;    // joystick moved
    static const size_t JS_EVENT_INIT   = 0x80;    // initial state of device
	js_event jse;

	while(read(joystick_fd_, &jse, sizeof(js_event)) == sizeof(js_event))
	{
        jse.type &= ~JS_EVENT_INIT; // ignore synthetic events

		if(jse.type == JS_EVENT_AXIS)
		{
            boost::lock_guard<boost::mutex> lg(mtx_);
		    if(jse.number < 6)
                ax_[jse.number] = jse.value;
		}
		else if(jse.type == JS_EVENT_BUTTON)
		{
            boost::lock_guard<boost::mutex> lg(mtx_);
			if(jse.number < 11)
                btn_[jse.number] = jse.value;
		}
	}
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
const std::string joystick_listener::getCmdString() const
{
    std::stringstream sstr;
/*
    static size_t counter = 0;

    switch(counter++)
    {
    case 0:
        sstr << "fb" << AxisVal(1) / 365 + 90;         // front back
        break;
    case 1:
         sstr << "lr" << AxisVal(0) / 365 + 90;         // left right
         break;
    default:
         sstr << "ms" << std::max<int>(0, 256 - (AxisVal(3) / 256 + 128)); // rotor throttle (motor speed)
         counter = 0;
    }
*/
    const double scalingFactor = 0.5;

    sstr << "fb" << AxisVal(1) * scalingFactor / 365 + 90 << "\n"               // front back
         << "lr" << AxisVal(0) * scalingFactor / 365 + 90 << "\n"               // left right
         << "ms" << std::max<int>(0, 256 - (AxisVal(3) / 256 + 128)) << "\n";   // rotor throttle (motor speed)

    return sstr.str();
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A


