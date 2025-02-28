/*
 * Copyright (c) 2025, Eberty Alves
 *
 * To run these tests from the command line, build the project and run "ctest" to execute all tests. Include the "-C
 * Release" flag if run on Windows.
 *
 * To run the tests in Microsoft Visual Studio, open the Test Explorer by clicking Test > Test Explorer in the menu.
 * Then, select the tests you want to run and click the "Run" button to execute them.
 */

#include <cassert>
#include <cmath>
#include <sstream>
#include <vector>
#include <future>

#include "ArcSegment.h"
#include "Contour.h"
#include "LineSegment.h"
#include "Point.h"
#include "Segment.h"
#include "Utils.h"

static const double EPSILON = 1e-5;

void testPointOperations()
{
    Point p1(1, 2);
    Point p2(1, 2);
    Point p3(3, 4);
    assert(p1 == p2);
    assert(p1 != p3);

    Point sum = p1 + p3;
    assert(std::abs(sum.x - 4.0) < EPSILON && std::abs(sum.y - 6.0) < EPSILON);

    Point diff = p3 - p1;
    assert(std::abs(diff.x - 2.0) < EPSILON && std::abs(diff.y - 2.0) < EPSILON);

    Point prod = p1 * 3;
    assert(std::abs(prod.x - 3.0) < EPSILON && std::abs(prod.y - 6.0) < EPSILON);

    Point div = prod / 3;
    assert(std::abs(div.x - 1.0) < EPSILON && std::abs(div.y - 2.0) < EPSILON);

    [[maybe_unused]] double dot = p1.dot(p3);
    assert(std::abs(dot - 11.0) < EPSILON);

    [[maybe_unused]] double cross = p1.cross(p3);
    assert(std::abs(cross - (1 * 4 - 2 * 3)) < EPSILON);

    std::stringstream ss;
    ss << p1;
    assert(ss.str() == "(1, 2)");
}

void testDistance()
{
    Point                   p1(0, 0);
    Point                   p2(3, 4);
    [[maybe_unused]] double d = p1.distanceTo(p2);
    assert(std::abs(d - 5.0) < EPSILON);
}

void testLineSegment()
{
    LineSegment ls(Point(0, 0), Point(1, 1));
    assert(ls.getStart() == Point(0, 0));
    assert(ls.getEnd() == Point(1, 1));

    auto cloneLS = ls.clone();
    assert(cloneLS->getStart() == ls.getStart());
    assert(cloneLS->getEnd() == ls.getEnd());
}

void testArcSegment()
{
    ArcSegment arc(Point(1, 0), Point(1, 1), 1.0, true, true);
    Point      start = arc.getStart();
    Point      end   = arc.getEnd();

    assert(std::abs(start.x - 1.0) < EPSILON && std::abs(start.y - 0.0) < EPSILON);
    assert(std::abs(end.x - 1.0) < EPSILON && std::abs(end.y - 1.0) < EPSILON);

    auto cloneArc = arc.clone();
    assert(cloneArc->getStart() == arc.getStart());
    assert(cloneArc->getEnd() == arc.getEnd());
}

void testCreateContourFromPolyline()
{
    std::vector<Point> pts = {Point(0, 0), Point(1, 0), Point(1, 1)};

    Contour contourOpen = createContourFromPolyline(pts, false);
    assert(contourOpen.segmentCount() == 2);

    Contour contourClosed = createContourFromPolyline(pts, true);
    assert(contourClosed.segmentCount() == 3);

    [[maybe_unused]] LineSegment* lastSeg = dynamic_cast<LineSegment*>(contourClosed.getSegments().back().get());
    assert(lastSeg != nullptr);
    assert(lastSeg->getStart() == Point(1, 1));
    assert(lastSeg->getEnd() == Point(0, 0));
}

void testContourValidity()
{
    std::vector<Point> pts = {Point(0, 0), Point(1, 0), Point(1, 1)};

    Contour validContour = createContourFromPolyline(pts);
    assert(validContour.isValid() == true);

    Contour invalidContour;
    invalidContour.addSegment(std::make_unique<LineSegment>(Point(0, 0), Point(-1, 0)));
    invalidContour.addSegment(std::make_unique<LineSegment>(Point(-2, 0), Point(-2, -1)));
    assert(invalidContour.isValid() == false);
}

void testContourCopyAndMove()
{
    std::vector<Point> pts      = {Point(0, 0), Point(1, 0), Point(1, 1)};
    Contour            original = createContourFromPolyline(pts, true);

    Contour copyContour(original);
    assert(copyContour.segmentCount() == original.segmentCount());

    original.removeSegment(0);
    assert(copyContour.segmentCount() != original.segmentCount());

    Contour movedContour(std::move(copyContour));
    assert(movedContour.segmentCount() > 0);
}

void testInsertRemoveSegment()
{
    std::vector<Point>      pts          = {Point(0, 0), Point(1, 0), Point(1, 1)};
    Contour                 contour      = createContourFromPolyline(pts, false);
    [[maybe_unused]] size_t initialCount = contour.segmentCount();

    contour.insertSegment(1, std::make_unique<LineSegment>(Point(0.5, 0.5), Point(0.75, 0.75)));
    assert(contour.segmentCount() == initialCount + 1);

    [[maybe_unused]] bool removed = contour.removeSegment(1);
    assert(removed);
    assert(contour.segmentCount() == initialCount);
}

void testAsyncFiltering()
{
    std::vector<Contour> contours;

    std::vector<Point> pts1 = {Point(0, 0), Point(1, 0), Point(1, 1)};
    contours.push_back(createContourFromPolyline(pts1));

    Contour valid;
    valid.addSegment(std::make_unique<LineSegment>(Point(0, 0), Point(1, 0)));
    valid.addSegment(std::make_unique<ArcSegment>(Point(1, 0), Point(2, 1), -1.0, true, true));
    contours.push_back(std::move(valid));

    Contour invalid;
    invalid.addSegment(std::make_unique<LineSegment>(Point(0, 0), Point(1, 0)));
    invalid.addSegment(std::make_unique<LineSegment>(Point(2, 0), Point(2, 1)));
    contours.push_back(std::move(invalid));

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

    std::vector<const Contour*> validContours   = futureValid.get();
    std::vector<const Contour*> invalidContours = futureInvalid.get();

    assert(validContours.size() == 2);
    assert(invalidContours.size() == 1);

    std::vector<const Contour*> unionContours = validContours;
    unionContours.insert(unionContours.end(), invalidContours.begin(), invalidContours.end());
    assert(unionContours.size() == contours.size());
}

int main()
{
    testPointOperations();
    testDistance();
    testLineSegment();
    testArcSegment();
    testCreateContourFromPolyline();
    testContourValidity();
    testContourCopyAndMove();
    testInsertRemoveSegment();
    testAsyncFiltering();

    return 0;
}
