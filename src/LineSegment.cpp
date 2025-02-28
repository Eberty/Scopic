/*
 * Copyright (c) 2025, Eberty Alves
 */

#include "LineSegment.h"

LineSegment::LineSegment(const Point& _start, const Point& _end) : m_start(_start), m_end(_end)
{
}

Point LineSegment::getStart() const
{
    return m_start;
}

Point LineSegment::getEnd() const
{
    return m_end;
}

std::unique_ptr<Segment> LineSegment::clone() const
{
    return std::make_unique<LineSegment>(*this);
}
