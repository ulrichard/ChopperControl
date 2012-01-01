#ifndef MAINFORM_H
#define MAINFORM_H

// Qt
#include <QtGui/QMainWindow>
// boost
#include <boost/thread.hpp>



class ProximityRadar;
class QCheckBox;
class QCheckBox;
class QPlainTextEdit;
class QwtCompass;
class AttitudeIndicator;
class QwtScaleWidget;
class Communication;
class joystick_listener;
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
class ChopperMain : public QMainWindow
{
    Q_OBJECT;
public:
    ChopperMain();
    virtual ~ChopperMain();

private slots:
    void enableRanger(int state);
    void enableLogging(int state);
    void enableControl(int state);

private:
    void sendCommands();

    ProximityRadar    *radar;
    QCheckBox         *cbSweep;
    QCheckBox         *cbLog;
    QCheckBox         *cbControl;
    QPlainTextEdit    *plainTextEdit;
    QwtCompass        *Heading;
    AttitudeIndicator *Attitude;
    QwtScaleWidget    *Vario;
    Communication     *sercomm;
    joystick_listener *joyslis;

    bool               doSendControl_;
    boost::thread      inputListener;
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A


#endif // MAINFORM_H
