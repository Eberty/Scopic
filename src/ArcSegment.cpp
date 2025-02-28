/*
 * Copyright (c) 2025, Eberty Alves
 */

#include "ArcSegment.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ArcSegment::ArcSegment(
    const Point& _start, const Point& _end, double _radius, bool _positiveDirection, bool _useShortestArc)
    : m_center(Point(0, 0)), m_radius(_radius)
{
    m_center = findCircleCenter(_start, _end, _positiveDirection);

    m_startAngle = normalizeAngle(std::atan2(_start.y - m_center.y, _start.x - m_center.x));
    m_endAngle   = normalizeAngle(std::atan2(_end.y - m_center.y, _end.x - m_center.x));

    const double twoPi = 2 * M_PI;

    double delta = m_endAngle - m_startAngle;
    if (delta > M_PI)
    {
        delta -= twoPi;
    }
    else if (delta < -M_PI)
    {
        delta += twoPi;
    }

    if (!_useShortestArc)
    {
        delta += (delta >= 0 ? -twoPi : twoPi);
    }

    m_endAngle = m_startAngle + delta;
}

Point ArcSegment::pointAt(double _angle) const
{
    return Point(m_center.x + m_radius * std::cos(_angle), m_center.y + m_radius * std::sin(_angle));
}

Point ArcSegment::getStart() const
{
    return pointAt(m_startAngle);
}

Point ArcSegment::getEnd() const
{
    return pointAt(m_endAngle);
}

std::unique_ptr<Segment> ArcSegment::clone() const
{
    return std::make_unique<ArcSegment>(*this);
}

Point ArcSegment::getCircleCenter() const
{
    return m_center;
}

double ArcSegment::getCircleRadius() const
{
    return m_radius;
}

double ArcSegment::getStartAngle() const
{
    return m_startAngle;
}

double ArcSegment::getEndAngle() const
{
    return m_endAngle;
}

Point ArcSegment::findCircleCenter(const Point& _a, const Point& _b, bool _positiveDirection)
{
    Point  mid  = (_a + _b) / 2.0;
    Point  d    = (_a - _b) / 2.0;
    double dist = d.length();

    if (m_radius < dist)
    {
        m_radius = dist;
    }

    double pDist = std::sqrt(m_radius * m_radius - dist * dist);
    Point  m     = _positiveDirection ? Point(d.y * pDist / dist, -d.x * pDist / dist)
                                 : Point(-d.y * pDist / dist, d.x * pDist / dist);
    return mid + m;
}

double ArcSegment::normalizeAngle(double _angle) const
{
    const double twoPi = 2.0 * M_PI;

    _angle = std::fmod(_angle, twoPi);
    if (_angle < 0)
    {
        _angle += twoPi;
    }

    return _angle;
}
