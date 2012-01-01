#include "mainwindow.h"
#include "ProximityRadar.h"
#include "Attitude.h"
#include "Communication.h"
#ifdef __arm__

#else
 #include "joystick.h"
#endif
// Qt
#include <QtGui/QCheckBox>
#include <QtGui/QPlainTextEdit>
#include <QMessageBox>
#include <qwt_compass.h>
#include <qwt_compass_rose.h>
#include <qwt_dial_needle.h>
#include <qwt_dial.h>
#include <qwt_scale_widget.h>
//boost
#include <boost/tuple/tuple.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/ref.hpp>
#include <boost/timer.hpp>

namespace bfs = boost::filesystem;

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
struct CompasValSetter
{
    CompasValSetter(QwtCompass &comp) : comp_(comp) { }

    void operator()(const boost::any &val)
    {
        comp_.setValue(boost::any_cast<size_t>(val));
    }
private:
    QwtCompass &comp_;
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
struct TextAdder
{
    TextAdder(QPlainTextEdit &edit) : edit_(edit) { }

    void operator()(const boost::any &val)
    {
        edit_.appendPlainText(QString(boost::any_cast<std::string>(val).c_str()));
        edit_.appendPlainText("\n");
    }
private:
    QPlainTextEdit &edit_;
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
struct AttitudeSetter
{
    AttitudeSetter(AttitudeIndicator &attit) : attit_(attit) { }

    void operator()(const boost::any &val)
    {
        const boost::tuple<size_t, size_t, size_t> accXYZ = boost::any_cast<boost::tuple<size_t, size_t, size_t> >(val);
        const double accX = (accXYZ.get<0>() - 520.0) / 230.0;
        const double accY = (accXYZ.get<1>() - 520.0) / 230.0;
        const double accZ = (accXYZ.get<2>() - 520.0) / 230.0;
        const double angl = acos(accY) - M_PI / 2;
        const double grd  = acos(accX) - M_PI / 2;

 //       std::cout << accX << " " << accY << " " << accZ << " A" << angl << " G" << grd << std::endl;

        attit_.setGradient(-grd);
        attit_.setAngle(angl * 180.0 / M_PI);
        attit_.update();
    }
private:
    AttitudeIndicator &attit_;
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
ChopperMain::ChopperMain()
    : doSendControl_(false)
{
    setWindowTitle("ChopperRemote");
    resize(480, 550);

    QWidget *centralwidget = new QWidget(this);
    centralwidget->setObjectName(QString::fromUtf8("centralwidget"));

    radar = new ProximityRadar(200, centralwidget);
    radar->setGeometry(QRect(0, 0, 400, 200));

    cbSweep = new QCheckBox(centralwidget);
    cbSweep->setGeometry(QRect(10, 380, 201, 22));
    cbSweep->setText("sweep");

    cbLog = new QCheckBox(centralwidget);
    cbLog->setGeometry(QRect(10, 410, 191, 22));
    cbLog->setText("log");

    cbControl = new QCheckBox(centralwidget);
    cbControl->setGeometry(QRect(10, 440, 220, 22));
    cbControl->setText("control");

    plainTextEdit = new QPlainTextEdit(centralwidget);
    plainTextEdit->setGeometry(QRect(0, 470, 481, 80));

    QPalette palComp;
    palComp.setColor(QPalette::Base, Qt::darkBlue);
    palComp.setColor(QPalette::Foreground, QColor(Qt::darkBlue).dark(120));
    palComp.setColor(QPalette::Text, Qt::white);
    Heading = new QwtCompass(centralwidget);
    Heading->setGeometry(QRect(10, 230, 141, 131));
    Heading->setLineWidth(4);
    Heading->setValue(0);
    Heading->setScaleOptions(QwtDial::ScaleTicks | QwtDial::ScaleLabel);
    Heading->setScaleTicks(0, 0, 3);
    Heading->setScale(36, 5, 0);
    Heading->setNeedle(new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, true, Qt::red, QColor(Qt::gray).light(130)));
    Heading->setPalette(palComp);

    QPalette palAtt;
    palAtt.setColor(QPalette::Base, Qt::darkBlue);
    palAtt.setColor(QPalette::Foreground, QColor(255,128,0).dark(120));
    palAtt.setColor(QPalette::Text, Qt::white);
    Attitude = new AttitudeIndicator(centralwidget);
    Attitude->setGeometry(QRect(160, 230, 141, 131));
    Attitude->setPalette(palAtt);

    Vario = new QwtScaleWidget(centralwidget);
    Vario->setGeometry(QRect(440, 10, 20, 421));

    setCentralWidget(centralwidget);

	try
	{
		sercomm = new Communication("/dev/rfcomm0");
		sercomm->addListener(Communication::LST_RANGER,  boost::bind(&ProximityRadar::addReadingAny, radar, ::_1));
		sercomm->addListener(Communication::LST_COMPASS, CompasValSetter(*Heading));
		sercomm->addListener(Communication::LST_ACCEL,   AttitudeSetter(*Attitude));
	 //   sercomm->addListener(Communication::LST_TEXT,    TextAdder(*plainTextEdit));
	}
	catch(std::exception &ex)
	{
		std::stringstream sstr;
		sstr << "Could not establish a bluetooth connection to the chopper on \"/dev/rfcomm0\" with the following message:\n" << ex.what();
		QMessageBox::critical(this, "No connection to the chopper", sstr.str().c_str(), QMessageBox::Ok);
	}

    connect(cbSweep,   SIGNAL(stateChanged(int)), this, SLOT(enableRanger(int)));
    connect(cbLog,     SIGNAL(stateChanged(int)), this, SLOT(enableLogging(int)));
    connect(cbControl, SIGNAL(stateChanged(int)), this, SLOT(enableControl(int)));

    TextAdder ta(*plainTextEdit);
    ta(boost::any(std::string("Welcome")));

#ifdef _DEBUG
    enableRanger(Qt::Checked);
//    enableLogging(Qt::Checked);
#endif

#ifdef __arm__
    // assuming we are on the freerunner, so read the accelerometers

#else
    // assuming we are on a notebook or computer, see if we have a joystick to read
	try
	{
		joyslis = new joystick_listener("/dev/input/js0");
		sendCommands();
		inputListener = boost::thread(boost::ref(*joyslis));

		sercomm->addListener(Communication::LST_TEXT, boost::bind(&ChopperMain::sendCommands, this));
	}
	catch(std::exception& ex)
	{
		std::stringstream sstr;
		sstr << "Could not connect to the joystick on \"/dev/input/js0\" with the following message:\n" << ex.what();
		QMessageBox::critical(this, "No connection to the joystick", sstr.str().c_str(), QMessageBox::Ok);
	}

#endif

}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void ChopperMain::sendCommands()
{
    if(!doSendControl_)
        return;
/*
    static boost::mutex mtx;
    boost::lock_guard<boost::mutex> lg(mtx);
    static boost::timer btim;
    if(btim.elapsed() < 0.1)
        return;
    btim.restart();
*/

    std::stringstream sstr;

    // front back
    static size_t lastFB = 90;
    const size_t fb = joyslis->AxisVal(1) / 365 + 90;
//    if(fb != lastFB)
        sstr << "fb" << fb << "\n";
    lastFB = fb;

    // left right
    static size_t lastLR = 90;
    const size_t lr = joyslis->AxisVal(0) / 365 + 90;
//    if(lr != lastLR)
        sstr << "lr" << lr << "\n";
    lastLR = lr;

    // rotor throttle (motor speed)
    static size_t lastMS = 0;
    const size_t ms = std::max<int>(0, 256 - (joyslis->AxisVal(3) / 256 + 128));
//    if(ms != lastMS)
        sstr << "ms" << ms << "\n";
    lastMS = ms;

    if(sstr.str().length())
        sercomm->sendCommand(sstr.str());
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
ChopperMain::~ChopperMain()
{
    enableRanger(0);
    enableLogging(0);
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void ChopperMain::enableRanger(int state)
{
    sercomm->sendCommand(state == Qt::Checked ? "swon\n": "swoff\n");
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void ChopperMain::enableLogging(int state)
{
    sercomm->sendCommand(state == Qt::Checked ? "lon\n" : "loff\n");
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void ChopperMain::enableControl(int state)
{
    doSendControl_ = (state == Qt::Checked);
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A

