
// Chopper
#include "ProximityRadar.h"
// Qt
#include <QtGui/QPainter>
#include <QtGui/QApplication>
// boost
#include <boost/lexical_cast.hpp>
// std lib
#include <cmath>
#include <set>
#include <iostream>

ProximityRadar::ProximityRadar(const size_t max_radius, QWidget *parent)
    : QWidget(parent), max_radius_(max_radius)
{

}

ProximityRadar::~ProximityRadar()
{

}

void ProximityRadar::addReading(const size_t angle, const size_t distval)
{
    if(angle < -360.0 || angle > 720.0)
        return;
    const size_t angle_mod = 90 + (90 - angle); // mirror
    const float volts  = distval * 5.0 / 1024;
    const float dist = 65 * pow(volts, -1.1);
    boost::mutex::scoped_lock  lock(mut_);
    readings_[angle_mod] = dist;
    scan_pos_ = angle_mod;
    update();
}

void ProximityRadar::addReadingAny(const boost::any &angvalpair)
{
    std::pair<size_t, size_t> angval = boost::any_cast<std::pair<size_t, size_t> >(angvalpair);
    addReading(angval.first, angval.second);
}

void ProximityRadar::paintEvent(QPaintEvent *event)
{
    QRect bbox = geometry();
    bbox = QRect(bbox.x() + 2, bbox.y() + 2, bbox.width() - 4, 2 * bbox.height() - 4);
    const size_t radiusmaxPx = bbox.width() / 2;
    const QPoint center   = QPoint(bbox.center().x(), bbox.bottom());

    QPainter painter(this);
    painter.eraseRect(bbox);
    paintBeams(painter, bbox.center(), radiusmaxPx);
    paintCircles(painter, bbox.center(), radiusmaxPx);

    painter.setPen(QPen(Qt::red, 2, Qt::SolidLine));
    painter.drawLine(bbox.center(), QPoint(bbox.center().x() + (radiusmaxPx + 10) * cos(scan_pos_ * 2 * M_PI / 360),
                                           bbox.center().y() - (radiusmaxPx + 10) * sin(scan_pos_ * 2 * M_PI / 360)));
}

void ProximityRadar::paintBeams(QPainter &painter, const QPoint center, const size_t radiusmaxPx)
{
    boost::mutex::scoped_lock lock(mut_);
    if(readings_.empty())
        return;

    const size_t step = ((--readings_.end())->first - readings_.begin()->first) / readings_.size();
    painter.setPen(QPen(Qt::green, 1, Qt::SolidLine));
    painter.setBrush(QBrush(Qt::green, Qt::SolidPattern));
    for(std::map<size_t, size_t>::const_iterator it = readings_.begin(); it != readings_.end(); ++it)
    {
        const size_t radiusPx = it->second * max_radius_ / radiusmaxPx ;
        const QRect bbox(center.x() - radiusPx, center.y() - radiusPx, radiusPx * 2, radiusPx * 2);
        painter.drawPie(bbox, (it->first - step / 2) * 16, step * 16);
    }
}

void ProximityRadar::paintCircles(QPainter &painter, const QPoint center, const size_t radiusmaxPx)
{
    painter.setPen(QPen(Qt::darkGreen, 1, Qt::SolidLine));
    painter.setFont(QFont("Times", 8));

    // find the scanning range
    size_t angl_min = 180, angl_max = 0, val_max = 0;
    for(std::map<size_t, size_t>::const_iterator it = readings_.begin(); it != readings_.end(); ++it)
    {
        angl_min = std::min(angl_min, it->first);
        angl_max = std::max(angl_max, it->first);
        val_max  = std::max(val_max,  it->second);
    }
    if(angl_min > angl_max)
        std::swap(angl_min, angl_max);

//    std::cout << "ProximityRadar::paintCircles (" << readings_.size() << " / " << angl_min << " / " << angl_max << " / " << val_max << ")" << std::endl;

    // draw the circles
    for(size_t i=20; i<=max_radius_; i += 20)
    {
        const size_t radiusPx = i * radiusmaxPx / max_radius_ ;
        const QRect bbox(center.x() - radiusPx, center.y() - radiusPx, radiusPx * 2, radiusPx * 2);
        painter.drawArc(bbox, angl_min * 16, (angl_max - angl_min) * 16);
        painter.drawText(center.x() + 2, center.y() - radiusPx - 2, boost::lexical_cast<std::string>(i).c_str());
    }

    // draw the angle lines
    const size_t textdist = 10;
    std::set<size_t> angles;
    angles.insert(angl_min);
    angles.insert(angl_max);
    for(size_t an = 0; an < 180; an += 30)
        if(an > angl_min && an < angl_max)
            angles.insert(an);
    for(std::set<size_t>::const_iterator it = angles.begin(); it != angles.end(); ++it)
    {
        painter.drawLine(center, QPoint(center.x() + radiusmaxPx * cos(*it * 2 * M_PI / 360), center.y() - radiusmaxPx * sin(*it * 2 * M_PI / 360)));

        painter.drawText(center.x() + (radiusmaxPx + textdist) * cos(*it * 2 * M_PI / 360), center.y() - (radiusmaxPx + textdist) * sin(*it * 2 * M_PI / 360),
            boost::lexical_cast<std::string>(*it).c_str());
    }
}


