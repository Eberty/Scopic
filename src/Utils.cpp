/*
 * Copyright (c) 2025, Eberty Alves
 */

#include "Utils.h"

#include "LineSegment.h"
#include "ArcSegment.h"

#include <algorithm>
#include <future>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double deg2rad(double _deg)
{
    return _deg * M_PI / 180.0;
}

double rad2deg(double _rad)
{
    return _rad * 180.0 / M_PI;
}

Contour createContourFromPolyline(const std::vector<Point>& _points, bool _closed)
{
    Contour contour;

    if (_points.size() < 2)
    {
        return contour;
    }

    for (size_t i = 0; i < _points.size() - 1; i++)
    {
        contour.addSegment(std::make_unique<LineSegment>(_points[i], _points[i + 1]));
    }

    if (_closed && _points.size() > 2)
    {
        contour.addSegment(std::make_unique<LineSegment>(_points.back(), _points.front()));
    }

    return contour;
}

std::vector<Contour> contourRoutine()
{
    std::cout << "Starting contour routine..." << std::endl;

    std::vector<Contour> contours;

    // Contour 1: Valid polyline (points connected correctly)
    {
        std::vector<Point> pts = {{0, 0}, {1, 0}, {1, 1}};
        contours.push_back(createContourFromPolyline(pts));
    }

    // Contour 2: Invalid - disconnected segments
    {
        Contour contour;
        contour.addSegment(std::make_unique<LineSegment>(Point{0, 0}, Point{-1, 0}));
        // Next segment starts with gap: (-2,0) instead of (-1,0)
        contour.addSegment(std::make_unique<LineSegment>(Point{-2, 0}, Point{-2, -1}));
        contours.push_back(std::move(contour));
    }

    // Contour 3: Valid - combination of a line segment and an arc, connected correctly
    {
        Contour contour;
        contour.addSegment(std::make_unique<LineSegment>(Point{2, -0.5}, Point{1, 0}));
        contour.addSegment(std::make_unique<ArcSegment>(Point{1, 0}, Point{1, 1}, 1.0, true, true));
        contours.push_back(std::move(contour));
    }

    // Contour 4: Invalid - disconnected segments
    {
        Contour contour;
        contour.addSegment(std::make_unique<LineSegment>(Point{0.25, 0.25}, Point{1, 0}));
        // Second segment starts at (1,0.09) instead of (1,0)
        contour.addSegment(std::make_unique<LineSegment>(Point{1, 0.09}, Point{2, 1}));
        contours.push_back(std::move(contour));
    }

    // Asynchronous task to get pointers to valid contours
    auto futureValid = std::async(std::launch::async, [&contours]() -> std::vector<const Contour*> {
        std::vector<const Contour*> validContours;
        for (const auto& contour : contours)
        {
            if (contour.isValid())
            {
                validContours.push_back(&contour);
            }
        }

        return validContours;
    });

    // Asynchronous task to get pointers to invalid contours
    auto futureInvalid = std::async(std::launch::async, [&contours]() -> std::vector<const Contour*> {
        std::vector<const Contour*> invalidContours;
        for (const auto& contour : contours)
        {
            if (!contour.isValid())
            {
                invalidContours.push_back(&contour);
            }
        }

        return invalidContours;
    });

    // Retrieve results from both asynchronous tasks
    std::vector<const Contour*> validContours   = futureValid.get();
    std::vector<const Contour*> invalidContours = futureInvalid.get();

    // Verify that each set contains unique elements
    auto isUnique = [](const std::vector<const Contour*>& vec) {
        std::vector<const Contour*> copy = vec;
        std::sort(copy.begin(), copy.end());
        return std::adjacent_find(copy.begin(), copy.end()) == copy.end();
    };

    bool validUnique   = isUnique(validContours);
    bool invalidUnique = isUnique(invalidContours);

    // Verify that the union of the two sets contains all original contours
    std::vector<const Contour*> unionContours = validContours;
    unionContours.insert(unionContours.end(), invalidContours.begin(), invalidContours.end());

    bool allIncluded = (unionContours.size() == contours.size());

    // Output test results
    std::cout << "Total contours: " << contours.size() << std::endl;
    std::cout << "Valid contours: " << validContours.size() << std::endl;
    std::cout << "Invalid contours: " << invalidContours.size() << std::endl;
    std::cout << "Valid set unique: " << (validUnique ? "Yes" : "No") << std::endl;
    std::cout << "Invalid set unique: " << (invalidUnique ? "Yes" : "No") << std::endl;
    std::cout << "Union covers all contours: " << (allIncluded ? "Yes" : "No") << std::endl;

    return contours;
}
