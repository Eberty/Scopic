/*
 * Copyright (c) 2025, Eberty Alves
 */

#pragma once

#include "Segment.h"

/**
 * @brief Class representing an arc segment.
 *
 * This class represents an arc segment with a center, radius, and start and end angles. The arc is defined by a circle
 * with a center and radius, and start and end angles.
 */
class ArcSegment : public Segment
{
public:
    /**
     * @brief Constructor for ArcSegment.
     *
     * This constructor computes the center of the arc based on the start/end points and a given radius. It also
     * computes the start and end angles based on the start/end points. Angles are normalized to the range [-pi, pi].
     *
     * @param _start The start point of the arc segment.
     * @param _end The end point of the arc segment.
     * @param _radius The radius of the arc segment, that can be adjusted if needed.
     * @param _positiveDirection Flag indicating if the arc is in the positive direction.
     * @param _useShortestArc Flag indicating if the shortest arc should be used.
     */
    ArcSegment(const Point& _start, const Point& _end, double _radius, bool _positiveDirection, bool _useShortestArc);

    /**
     * @brief Compute a point on the complete circle at a given angle. Can also return values not belonging to the arc.
     *
     * @param _angle The angle (in radians).
     *
     * @return The point on the circle at the given angle.
     */
    Point pointAt(double _angle) const;

    /**
     * @brief Get the starting point of the arc segment.
     *
     * @return The starting point of the arc segment.
     */
    virtual Point getStart() const override;

    /**
     * @brief Get the ending point of the arc segment.
     *
     * @return The ending point of the arc segment.
     */
    virtual Point getEnd() const override;

    /**
     * @brief Clone the arc segment.
     *
     * @return A unique pointer to the cloned arc segment.
     */
    virtual std::unique_ptr<Segment> clone() const override;

    /**
     * @brief Get the center of the circle that the arc is part of.
     *
     * @return The center of the circle that the arc is part of.
     */
    Point getCircleCenter() const;

    /**
     * @brief Get the radius of the circle that the arc is part of.
     *
     * @return The radius of the circle that the arc is part of.
     */
    double getCircleRadius() const;

    /**
     * @brief Get the start angle of the arc.
     *
     * @return The start angle of the arc (in radians).
     */
    double getStartAngle() const;

    /**
     * @brief Get the end angle of the arc.
     *
     * @return The end angle of the arc (in radians).
     */
    double getEndAngle() const;

private:
    /**
     * @brief Find the center of a circle given two points on the circle.
     *
     * This function finds the center of a circle given two points and a radius. It also updates the radius if the
     * distance between the two points is greater than the current radius. The center is calculated using the midpoint
     * of the two points and the perpendicular distance from the midpoint to the center of the circle.
     *
     * @see https://stackoverflow.com/questions/20025805/calculating-the-center-of-a-circle-touching-two-points
     *
     * @param _a The first point.
     * @param _b The second point.
     * @param _positiveDirection Flag indicating if the arc is in the positive direction.
     *
     * @return The center of the circle.
     */
    Point findCircleCenter(const Point& _a, const Point& _b, bool _positiveDirection);

    /**
     * @brief Normalize an angle to the range [0, 2pi).
     *
     * @param _angle The angle to normalize (in radians).
     *
     * @return The normalized angle.
     */
    double normalizeAngle(double _angle) const;

    Point  m_center;     /**< The center of the arc. */
    double m_radius;     /**< The radius of the arc. */
    double m_startAngle; /**< The start angle of the arc (in radians). */
    double m_endAngle;   /**< The end angle of the arc (in radians). */
};
