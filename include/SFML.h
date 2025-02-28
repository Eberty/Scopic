/*
 * Copyright (c) 2025, Eberty Alves
 */

#pragma once

#ifdef SFML_AVAILABLE

#include "ArcSegment.h"
#include "Contour.h"
#include "Point.h"
#include "Segment.h"

#include <limits>
#include <vector>

#include <SFML/Graphics.hpp>

/**
 * @brief Struct representing a bounding box.
 *
 * This struct represents a bounding box with minimum and maximum x and y coordinates.
 */
struct BoundingBox
{
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
};

/**
 * @brief Get points from an arc.
 *
 * This function samples points from an arc and returns them in a vector. It samples points along the arc based on the
 * start and end angles.
 *
 * @param _arc The arc to sample points from.
 *
 * @return A vector of points sampled from the arc.
 */
std::vector<Point> sampleArcPoints(const ArcSegment* _arc);

/**
 * @brief Get points from a segment.
 *
 * This function samples points from a segment (line or arc) and returns them in a vector. For an arc, it samples points
 * along the arc based on the start and end angles.
 *
 * @param _seg The segment to sample points from.
 *
 * @return A vector of points sampled from the segment.
 */
std::vector<Point> getSegmentPoints(const Segment* _seg);

/**
 * @brief Calculate the bounding box of a contour.
 *
 * This function calculates the bounding box of a contour based on its segments.
 *
 * @param _contour The contour to calculate the bounding box for.
 *
 * @return The bounding box of the contour.
 */
BoundingBox calculateBoundingBox(const Contour& _contour);

/**
 * @brief Print the coordinates of a mouse click.
 *
 * This function prints the coordinates of a mouse click in an SFML window.
 *
 * @param _window The SFML window.
 * @param _mouseEvent The mouse click event.
 */
void printMouseClickCoordinate(sf::RenderWindow& _window, const sf::Event::MouseButtonEvent& _mouseEvent);

/**
 * @brief Calculate the total bounding box of contours.
 *
 * This function calculates the total bounding box of a vector of contours with a margin.
 *
 * @param _contours The contours to calculate the bounding box for.
 * @param _margin The margin to add to the final bounding box.
 *
 * @return The total bounding box of the contours with a margin.
 */
BoundingBox calculateBoundingBox(const std::vector<Contour>& _contours, double _margin);

/**
 * @brief Create an SFML vertex from a point and color.
 *
 * This function creates an SFML vertex from a point and color.
 *
 * @param _point The point to create the vertex from.
 * @param _color The color of the vertex.
 *
 * @return The SFML vertex.
 */
sf::Vertex createVertex(const Point& _point, sf::Color _color);

/**
 * @brief Create an SFML vertex from coordinates and color.
 *
 * Convenience function to create an SFML vertex from coordinates and color.
 *
 * @param _x The x-coordinate of the vertex.
 * @param _y The y-coordinate of the vertex.
 * @param _color The color of the vertex.
 *
 * @return The SFML vertex.
 */
sf::Vertex createVertex(double _x, double _y, sf::Color _color);

/**
 * @brief Create vertices for an arc.
 *
 * This function creates vertices for an arc segment, approximating it with a line strip.
 *
 * @param _arc The arc segment to create vertices for.
 * @param _color The color of the vertices.
 *
 * @return A vector of vertices for the arc.
 */
std::vector<sf::Vertex> createArcVertices(const ArcSegment* _arc, sf::Color _color);

/**
 * @brief Draw an arrow in an SFML window.
 *
 * This function draws an arrow in an SFML window from a start point to an end point.
 *
 * @param _start The start point of the arrow.
 * @param _end The end point of the arrow.
 * @param _color The color of the arrow.
 * @param _window The SFML window to draw the arrow in.
 *
 * @return True if the arrow was drawn, false otherwise.
 */
bool drawArrow(const Point& start, const Point& end, sf::Color _color, sf::RenderWindow& _window);

/**
 * @brief Draw a contour in an SFML window.
 *
 * This function draws a contour in an SFML window using lines and arcs. An arrow is drawn at the end of each segment to
 * indicate the direction of the segment.
 *
 * @param _contour The contour to draw.
 * @param _color The color to draw the contour in.
 * @param _window The SFML window to draw the contour in.
 */
void drawContour(const Contour& _contour, sf::Color _color, sf::RenderWindow& _window);

/**
 * @brief Set the view of an SFML window from a bounding box.
 *
 * This function sets the view of an SFML window based on a bounding box, adjusting the view to maintain the aspect
 * ratio of the bounding box.
 *
 * @param _window The SFML window to set the view for.
 * @param _bbox The bounding box to set the view from.
 *
 * @return The adjusted bounding box with the aspect ratio maintained.
 */
BoundingBox setViewFromBoundingBox(sf::RenderWindow& _window, const BoundingBox& _bbox);

/**
 * @brief Draw axes in an SFML window.
 *
 * This function draws axes in an SFML window based on a bounding box.
 *
 * @param _window The SFML window to draw the axes in.
 * @param _bbox The bounding box to draw the axes from.
 */
void drawAxes(sf::RenderWindow& _window, const BoundingBox& _bbox);

/**
 * @brief Get default colors to use for contours.
 *
 * @return A vector of default colors.
 */
std::vector<sf::Color> defaultColors();

/**
 * @brief Handle window events.
 *
 * @param _window The SFML window.
 * @param _event The current event.
 * @param _bbox The original bounding box.
 * @param _adjustedBbox The adjusted bounding box.
 *
 * @return True if window should close, false otherwise.
 */
bool handleEvents(sf::RenderWindow& _window, sf::Event& _event, const BoundingBox& _bbox, BoundingBox& _adjustedBbox);

/**
 * @brief Draw all contours in the window.
 *
 * @param _window The SFML window.
 * @param _contours The contours to draw.
 * @param _colors The colors to use.
 * @param _adjustedBbox The adjusted bounding box.
 */
void renderContours(sf::RenderWindow&           _window,
                    const std::vector<Contour>& _contours,
                    std::vector<sf::Color>&     _colors,
                    const BoundingBox&          _adjustedBbox);

/**
 * @brief Visualize contours in an SFML window.
 *
 * @param _contours The contours to visualize.
 */
void visualizeContours(const std::vector<Contour>& _contours);

#endif
