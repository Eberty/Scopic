/*
 * Copyright (c) 2025, Eberty Alves
 */

#pragma once

#include "Segment.h"

/**
 * @brief Class representing a line segment.
 *
 * This class represents a line segment with a start and end point.
 */
class LineSegment : public Segment
{
public:
    /**
     * @brief Constructor for LineSegment.
     *
     * @param _start The start point of the line segment.
     * @param _end The end point of the line segment.
     */
    LineSegment(const Point& _start, const Point& _end);

    /**
     * @brief Get the starting point of the line segment.
     *
     * @return The starting point of the segment.
     */
    virtual Point getStart() const override;

    /**
     * @brief Get the ending point of the line segment.
     *
     * @return The ending point of the line segment.
     */
    virtual Point getEnd() const override;

    /**
     * @brief Clone the line segment.
     *
     * @return A unique pointer to the cloned line segment.
     */
    virtual std::unique_ptr<Segment> clone() const override;

private:
    Point m_start; /**< The starting point of the line segment. */
    Point m_end;   /**< The ending point of the line segment. */
};
