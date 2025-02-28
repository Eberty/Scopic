/*
 * Copyright (c) 2025, Eberty Alves
 */

#pragma once

#include "Contour.h"
#include "Point.h"

#include <vector>

/**
 * @brief Convert degrees to radians.
 *
 * @param _deg The angle in degrees.
 *
 * @return The angle in radians.
 */
double deg2rad(double _deg);

/**
 * @brief Convert radians to degrees.
 *
 * @param _rad The angle in radians.
 *
 * @return The angle in degrees.
 */
double rad2deg(double _rad);

/**
 * @brief Create a contour from a polyline.
 *
 * This function creates a contour from a polyline represented by a vector of points. It creates line segments between
 * consecutive points in the polyline. If the polyline is closed, it also creates a line segment between the last and
 * first points.
 *
 * @param _points The points in the polyline.
 * @param _closed Flag indicating if the polyline is closed.
 *
 * @return The contour created from the polyline.
 */
Contour createContourFromPolyline(const std::vector<Point>& _points, bool _closed = false);

/**
 * @brief Routine to test contours asynchronously.
 *
 * This function creates 4 hardcoded contours (few of them are valid, few invalid) and then asynchronously checks if
 * each contour is valid or not. It does this by creating two asynchronous tasks: one to get pointers to valid contours
 * and the other to get pointers to invalid contours. It then retrieves the results from both tasks and prints the
 * results. The function also checks if the union of the two sets of contours contains all the original contours.
 *
 * @return The vector of contours created for testing.
 */
std::vector<Contour> contourRoutine();
