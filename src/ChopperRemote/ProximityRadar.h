#ifndef PROXIMITYRADAR_H_INCLUDED
#define PROXIMITYRADAR_H_INCLUDED

// qt
#include <QtGui/QWidget>
// boost
#include <boost/any.hpp>
#include <boost/thread/mutex.hpp>
// std lib
#include <map>


class ProximityRadar : public QWidget
{
    Q_OBJECT;
public:
    ProximityRadar(const size_t max_radius = 100, QWidget *parent = 0);
    virtual ~ProximityRadar();

    void addReading(const size_t angle, const size_t distval);
    void addReadingAny(const boost::any &angvalpair);

protected:
    void paintEvent(QPaintEvent *event);

private:
    void paintBeams(QPainter &painter, const QPoint center, const size_t radiusmaxPx);
    void paintCircles(QPainter &painter, const QPoint center, const size_t radiusmaxPx);


    const size_t max_radius_;
    std::map<size_t, size_t> readings_; // degrees, centimeters
    boost::mutex mut_;
    size_t scan_pos_;
};

#endif // PROXIMITYRADAR_H_INCLUDED
