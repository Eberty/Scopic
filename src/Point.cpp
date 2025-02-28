/*
 * Copyright (c) 2025, Eberty Alves
 */

#include "Point.h"

#include <cmath>

Point::Point(double _x, double _y) : x(_x), y(_y)
{
}

bool Point::operator==(const Point& _other) const
{
    return x == _other.x && y == _other.y;
}

bool Point::operator!=(const Point& _other) const
{
    return !(*this == _other);
}

Point Point::operator-(const Point& _other) const
{
    return Point(x - _other.x, y - _other.y);
}

Point Point::operator+(const Point& _other) const
{
    return Point(x + _other.x, y + _other.y);
}

Point& Point::operator+=(const Point& _other)
{
    x += _other.x;
    y += _other.y;
    return *this;
}

Point Point::operator*(double _d) const
{
    return Point(x * _d, y * _d);
}

Point& Point::operator*=(double _d)
{
    x *= _d;
    y *= _d;
    return *this;
}

Point Point::operator/(double _d) const
{
    return Point(x / _d, y / _d);
}

Point& Point::operator/=(double _d)
{
    x /= _d;
    y /= _d;
    return *this;
}

std::ostream& operator<<(std::ostream& _out, const Point& _point)
{
    _out << "(" << _point.x << ", " << _point.y << ")";
    return _out;
}

double Point::dot(const Point& _other) const
{
    return x * _other.x + y * _other.y;
}

double Point::cross(const Point& _other) const
{
    return x * _other.y - y * _other.x;
}

double Point::length() const
{
    return std::sqrt(x * x + y * y);
}

double Point::distanceTo(const Point& _other) const
{
    return std::sqrt(((x - _other.x) * (x - _other.x)) + ((y - _other.y) * (y - _other.y)));
}
