/*
 * Copyright (c) 2025, Eberty Alves
 */

#include "Contour.h"

#ifndef EPSILON
#define EPSILON 1e-6
#endif

Contour::Contour() : m_cacheValid(false), m_cachedResult(false)
{
}

Contour::Contour(const Contour& _other) : m_cacheValid(_other.m_cacheValid), m_cachedResult(_other.m_cachedResult)
{
    for (const auto& seg : _other.m_segments)
    {
        m_segments.push_back(seg->clone());
    }
}

Contour::Contour(Contour&& _other) noexcept
    : m_cacheValid(_other.m_cacheValid), m_cachedResult(_other.m_cachedResult), m_segments(std::move(_other.m_segments))
{
    _other.m_cacheValid = false;
}

Contour& Contour::operator=(const Contour& _other)
{
    if (this != &_other)
    {
        m_segments.clear();
        for (const auto& seg : _other.m_segments)
        {
            m_segments.push_back(seg->clone());
        }

        m_cacheValid   = _other.m_cacheValid;
        m_cachedResult = _other.m_cachedResult;
    }

    return *this;
}

Contour& Contour::operator=(Contour&& _other) noexcept
{
    if (this != &_other)
    {
        m_segments          = std::move(_other.m_segments);
        m_cacheValid        = _other.m_cacheValid;
        m_cachedResult      = _other.m_cachedResult;
        _other.m_cacheValid = false;
    }

    return *this;
}

void Contour::addSegment(std::unique_ptr<Segment> _seg)
{
    m_segments.push_back(std::move(_seg));
    invalidateCache();
}

void Contour::insertSegment(size_t _index, std::unique_ptr<Segment> _seg)
{
    if (_index > m_segments.size())
    {
        _index = m_segments.size();
    }

    m_segments.insert(m_segments.begin() + _index, std::move(_seg));
    invalidateCache();
}

bool Contour::removeSegment(size_t _index)
{
    if (_index < m_segments.size())
    {
        m_segments.erase(m_segments.begin() + _index);
        invalidateCache();
        return true;
    }

    return false;
}

size_t Contour::segmentCount() const
{
    return m_segments.size();
}

const std::vector<std::unique_ptr<Segment>>& Contour::getSegments() const
{
    return m_segments;
}

std::vector<std::unique_ptr<Segment>>& Contour::getSegments()
{
    return m_segments;
}

bool Contour::isValid() const
{
    if (m_cacheValid)
    {
        return m_cachedResult;
    }

    if (m_segments.empty())
    {
        m_cacheValid   = true;
        m_cachedResult = true;
        return false;
    }

    bool valid = true;
    for (size_t i = 0; i < m_segments.size() - 1; i++)
    {
        Point endPoint  = m_segments[i]->getEnd();
        Point nextStart = m_segments[i + 1]->getStart();
        if (endPoint.distanceTo(nextStart) > EPSILON)
        {
            valid = false;
            break;
        }
    }

    m_cacheValid   = true;
    m_cachedResult = valid;

    return valid;
}

void Contour::invalidateCache()
{
    m_cacheValid = false;
}
