/*
 * Copyright (c) 2025, Eberty Alves
 */

#pragma once

#include "Point.h"

#include <memory>

/**
 * @brief Abstract base class for a segment.
 *
 * This class represents an abstract base class for a segment. Segments are considered directional, and the order of
 * points is important. For example, a line segment from point A to point B is different from a line segment from point
 * B to point A.
 */
class Segment
{
public:
    /**
     * @brief Virtual destructor for Segment.
     */
    virtual ~Segment() = default;

    /**
     * @brief Get the starting point of the segment.
     *
     * @return The starting point of the segment.
     */
    virtual Point getStart() const = 0;

    /**
     * @brief Get the ending point of the segment.
     *
     * @return The ending point of the segment.
     */
    virtual Point getEnd() const = 0;

    /**
     * @brief Clones the segment (for deep copy).
     *
     * @return A unique pointer to the cloned segment.
     */
    virtual std::unique_ptr<Segment> clone() const = 0;
};
