/*
 * Copyright (c) 2025, Eberty Alves
 */

#pragma once

#include <iostream>

/**
 * @brief Struct representing a 2D point.
 *
 * This struct represents a 2D point with x and y coordinates.
 */
struct Point
{
    double x;
    double y;

    /**
     * @brief Constructor for Point.
     *
     * @param _x The x coordinate of the point.
     * @param _y The y coordinate of the point.
     */
    Point(double _x = 0.0, double _y = 0.0);

    /**
     * @brief Equality operator for Point.
     *
     * @param _other The other point to compare with.
     *
     * @return True if the two points are equal, false otherwise.
     */
    bool operator==(const Point& _other) const;

    /**
     * @brief Inequality operator for Point.
     *
     * @param _other The other point to compare with.
     *
     * @return True if the two points are different, false otherwise.
     */
    bool operator!=(const Point& _other) const;

    /**
     * @brief Subtraction operator for Point.
     *
     * @param _other The other point to subtract.
     *
     * @return The difference between the two points.
     */
    Point operator-(const Point& _other) const;

    /**
     * @brief Addition operator for Point.
     *
     * @param _other The other point to add.
     *
     * @return The sum of the two points.
     */
    Point operator+(const Point& _other) const;

    /**
     * @brief Addition assignment operator for Point.
     *
     * @param _other The other point to add.
     *
     * @return The sum of the two points.
     */
    Point& operator+=(const Point& _other);

    /**
     * @brief Multiplication operator for Point.
     *
     * @param _d The scalar to multiply the point by.
     *
     * @return The scaled point.
     */
    Point operator*(double _d) const;

    /**
     * @brief Multiplication assignment operator for Point.
     *
     * @param _d The scalar to multiply the point by.
     *
     * @return The scaled point.
     */
    Point& operator*=(double _d);

    /**
     * @brief Division operator for Point.
     *
     * @param _d The scalar to divide the point by.
     *
     * @return The scaled point.
     */
    Point operator/(double _d) const;

    /**
     * @brief Division assignment operator for Point.
     *
     * @param _d The scalar to divide the point by.
     *
     * @return The scaled point.
     */
    Point& operator/=(double _d);

    /**
     * @brief Overload the << operator for Point.
     *
     * This function overloads the << operator for Point to allow printing it to an output stream.
     *
     * @param _out The output stream.
     * @param _point The point to print.
     *
     * @return The output stream.
     */
    friend std::ostream& operator<<(std::ostream& _out, const Point& _point);

    /**
     * @brief Dot product of two points.
     *
     * @param _other The other point.
     *
     * @return The dot product of the two points.
     */
    double dot(const Point& _other) const;

    /**
     * @brief Cross product of two points.
     *
     * @param _other The other point.
     *
     * @return The cross product of the two points.
     */
    double cross(const Point& _other) const;

    /**
     * @brief Compute the length of the point.
     *
     * @return The length of the point.
     */
    double length() const;

    /**
     * @brief Compute Euclidean distance to another point.
     *
     * @param _other The other point.
     *
     * @return The distance between the two points.
     */
    double distanceTo(const Point& _other) const;
};
