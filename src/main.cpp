/*
 * Copyright (c) 2025, Eberty Alves
 */

#include <algorithm>
#include <cmath>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#ifdef SFML_AVAILABLE
#include <SFML/Graphics.hpp>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846 /**< Pi constant. */
#endif

static const double EPSILON = 1e-6; /**< Epsilon for floating-point comparisons. */

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
    Point(double _x = 0.0, double _y = 0.0) : x(_x), y(_y)
    {
    }

    /**
     * @brief Equality operator for Point.
     *
     * @param _other The other point to compare with.
     *
     * @return True if the two points are equal, false otherwise.
     */
    bool operator==(const Point& _other) const
    {
        return x == _other.x && y == _other.y;
    }

    /**
     * @brief Inequality operator for Point.
     *
     * @param _other The other point to compare with.
     *
     * @return True if the two points are different, false otherwise.
     */
    bool operator!=(const Point& _other) const
    {
        return !(*this == _other);
    }

    /**
     * @brief Subtraction operator for Point.
     *
     * @param _other The other point to subtract.
     *
     * @return The difference between the two points.
     */
    Point operator-(const Point& _other) const
    {
        return Point(x - _other.x, y - _other.y);
    }

    /**
     * @brief Addition operator for Point.
     *
     * @param _other The other point to add.
     *
     * @return The sum of the two points.
     */
    Point operator+(const Point& _other) const
    {
        return Point(x + _other.x, y + _other.y);
    }

    /**
     * @brief Addition assignment operator for Point.
     *
     * @param _other The other point to add.
     *
     * @return The sum of the two points.
     */
    Point& operator+=(const Point& _other)
    {
        x += _other.x;
        y += _other.y;
        return *this;
    }

    /**
     * @brief Multiplication operator for Point.
     *
     * @param _d The scalar to multiply the point by.
     *
     * @return The scaled point.
     */
    Point operator*(double _d) const
    {
        return Point(x * _d, y * _d);
    }

    /**
     * @brief Multiplication assignment operator for Point.
     *
     * @param _d The scalar to multiply the point by.
     *
     * @return The scaled point.
     */
    Point& operator*=(double _d)
    {
        x *= _d;
        y *= _d;
        return *this;
    }

    /**
     * @brief Division operator for Point.
     *
     * @param _d The scalar to divide the point by.
     *
     * @return The scaled point.
     */
    Point operator/(double _d) const
    {
        return Point(x / _d, y / _d);
    }

    /**
     * @brief Division assignment operator for Point.
     *
     * @param _d The scalar to divide the point by.
     *
     * @return The scaled point.
     */
    Point& operator/=(double _d)
    {
        x /= _d;
        y /= _d;
        return *this;
    }

    /**
     * @brief Dot product of two points.
     *
     * @param _other The other point.
     *
     * @return The dot product of the two points.
     */
    double dot(const Point& _other) const
    {
        return x * _other.x + y * _other.y;
    }

    /**
     * @brief Cross product of two points.
     *
     * @param _other The other point.
     *
     * @return The cross product of the two points.
     */
    double cross(const Point& _other) const
    {
        return x * _other.y - y * _other.x;
    }

    /**
     * @brief Compute the length of the point.
     *
     * @return The length of the point.
     */
    double length() const
    {
        return std::sqrt(x * x + y * y);
    }

    /**
     * @brief Compute Euclidean distance to another point.
     *
     * @param _other The other point.
     *
     * @return The distance between the two points.
     */
    double distanceTo(const Point& _other) const
    {
        return std::sqrt(((x - _other.x) * (x - _other.x)) + ((y - _other.y) * (y - _other.y)));
    }
};

/**
 * @brief Overload the << operator for Point.
 *
 * This function overloads the << operator for Point to allow printing it to an output stream.
 *
 * @param _os The output stream.
 * @param _p The point to print.
 *
 * @return The output stream.
 */
std::ostream& operator<<(std::ostream& _os, const Point& _p)
{
    _os << "(" << _p.x << ", " << _p.y << ")";
    return _os;
}

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
    LineSegment(const Point& _start, const Point& _end) : m_start(_start), m_end(_end)
    {
    }

    /**
     * @brief Get the starting point of the line segment.
     *
     * @return The starting point of the segment.
     */
    virtual Point getStart() const override
    {
        return m_start;
    }

    /**
     * @brief Get the ending point of the line segment.
     *
     * @return The ending point of the line segment.
     */
    virtual Point getEnd() const override
    {
        return m_end;
    }

    /**
     * @brief Clone the line segment.
     *
     * @return A unique pointer to the cloned line segment.
     */
    virtual std::unique_ptr<Segment> clone() const override
    {
        return std::make_unique<LineSegment>(*this);
    }

private:
    Point m_start; /**< The starting point of the line segment. */
    Point m_end;   /**< The ending point of the line segment. */
};

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
    ArcSegment(const Point& _start, const Point& _end, double _radius, bool _positiveDirection, bool _useShortestArc)
        : m_center(Point(0, 0)), m_radius(_radius)
    {
        m_center = findCircleCenter(_start, _end, _positiveDirection);

        m_startAngle = normalizeAngle(std::atan2(_start.y - m_center.y, _start.x - m_center.x));
        m_endAngle   = normalizeAngle(std::atan2(_end.y - m_center.y, _end.x - m_center.x));

        const double twoPi = 2 * M_PI;

        double delta = m_endAngle - m_startAngle;
        if (delta > M_PI)
        {
            delta -= twoPi;
        }
        else if (delta < -M_PI)
        {
            delta += twoPi;
        }

        if (!_useShortestArc)
        {
            delta += (delta >= 0 ? -twoPi : twoPi);
        }

        m_endAngle = m_startAngle + delta;
    }

    /**
     * @brief Compute a point on the complete circle at a given angle. Can also return values not belonging to the arc.
     *
     * @param _angle The angle (in radians).
     *
     * @return The point on the circle at the given angle.
     */
    Point pointAt(double _angle) const
    {
        return Point(m_center.x + m_radius * std::cos(_angle), m_center.y + m_radius * std::sin(_angle));
    }

    /**
     * @brief Get the starting point of the arc segment.
     *
     * @return The starting point of the arc segment.
     */
    virtual Point getStart() const override
    {
        return pointAt(m_startAngle);
    }

    /**
     * @brief Get the ending point of the arc segment.
     *
     * @return The ending point of the arc segment.
     */
    virtual Point getEnd() const override
    {
        return pointAt(m_endAngle);
    }

    /**
     * @brief Clone the arc segment.
     *
     * @return A unique pointer to the cloned arc segment.
     */
    virtual std::unique_ptr<Segment> clone() const override
    {
        return std::make_unique<ArcSegment>(*this);
    }

    /**
     * @brief Get the center of the circle that the arc is part of.
     *
     * @return The center of the circle that the arc is part of.
     */
    Point getCircleCenter() const
    {
        return m_center;
    }

    /**
     * @brief Get the radius of the circle that the arc is part of.
     *
     * @return The radius of the circle that the arc is part of.
     */
    double getCircleRadius() const
    {
        return m_radius;
    }

    /**
     * @brief Get the start angle of the arc.
     *
     * @return The start angle of the arc (in radians).
     */
    double getStartAngle() const
    {
        return m_startAngle;
    }

    /**
     * @brief Get the end angle of the arc.
     *
     * @return The end angle of the arc (in radians).
     */
    double getEndAngle() const
    {
        return m_endAngle;
    }

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
    Point findCircleCenter(const Point& _a, const Point& _b, bool _positiveDirection)
    {
        Point  mid  = (_a + _b) / 2.0;
        Point  d    = (_a - _b) / 2.0;
        double dist = d.length();

        if (m_radius < dist)
        {
            m_radius = dist;
        }

        double pDist = std::sqrt(m_radius * m_radius - dist * dist);
        Point  m     = _positiveDirection ? Point(d.y * pDist / dist, -d.x * pDist / dist)
                                     : Point(-d.y * pDist / dist, d.x * pDist / dist);
        return mid + m;
    }

    /**
     * @brief Normalize an angle to the range [0, 2pi).
     *
     * @param _angle The angle to normalize (in radians).
     *
     * @return The normalized angle.
     */
    double normalizeAngle(double _angle) const
    {
        const double twoPi = 2.0 * M_PI;

        _angle = std::fmod(_angle, twoPi);
        if (_angle < 0)
        {
            _angle += twoPi;
        }

        return _angle;
    }

    Point  m_center;     /**< The center of the arc. */
    double m_radius;     /**< The radius of the arc. */
    double m_startAngle; /**< The start angle of the arc (in radians). */
    double m_endAngle;   /**< The end angle of the arc (in radians). */
};

/**
 * @brief Class representing a contour composed of multiple segments.
 *
 * Contour is a copyable and moveable class and holds a vector of segments, anticipate support more segment types in the
 * future. This class provides methods to add, insert, and remove segments. It also provides a method to check if the
 * contour is valid (i.e., all segments are connected sequentially) and caches the result of the isValid() method to
 * improve performance.
 */
class Contour
{
public:
    /**
     * @brief Default constructor for Contour.
     */
    Contour() : m_cacheValid(false), m_cachedResult(false)
    {
    }

    /**
     * @brief Copy constructor for Contour.
     *
     * This constructor performs a deep copy of the segments in the other contour.
     *
     * @param _other The other contour to copy.
     */
    Contour(const Contour& _other) : m_cacheValid(_other.m_cacheValid), m_cachedResult(_other.m_cachedResult)
    {
        for (const auto& seg : _other.m_segments)
        {
            m_segments.push_back(seg->clone());
        }
    }

    /**
     * @brief Move constructor for Contour.
     *
     * This constructor moves the segments from the other contour.
     *
     * @param _other The other contour to move.
     */
    Contour(Contour&& _other) noexcept
        : m_segments(std::move(_other.m_segments))
        , m_cacheValid(_other.m_cacheValid)
        , m_cachedResult(_other.m_cachedResult)
    {
        _other.m_cacheValid = false;
    }

    /**
     * @brief Copy assignment operator for Contour.
     *
     * This operator performs a deep copy of the segments in the other contour.
     *
     * @param _other The other contour to copy.
     *
     * @return A reference to this contour.
     */
    Contour& operator=(const Contour& _other)
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

    /**
     * @brief Move assignment operator for Contour.
     *
     * This operator moves the segments from the other contour.
     *
     * @param _other The other contour to move.
     *
     * @return A reference to this contour.
     */
    Contour& operator=(Contour&& _other) noexcept
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

    /**
     * @brief Add a segment to the contour.
     *
     * This method adds a segment to the end of the contour and invalidates the cache.
     *
     * @param _seg The segment to add.
     */
    void addSegment(std::unique_ptr<Segment> _seg)
    {
        m_segments.push_back(std::move(_seg));
        invalidateCache();
    }

    /**
     * @brief Insert a segment at a specified index.
     *
     * This method inserts a segment at the specified index in the contour and invalidates the cache. If the index is
     * greater than the number of segments, the segment is added to the end of the contour segment list.
     *
     * @param _index The index to insert the segment at.
     * @param _seg The segment to insert.
     */
    void insertSegment(size_t _index, std::unique_ptr<Segment> _seg)
    {
        if (_index > m_segments.size())
        {
            _index = m_segments.size();
        }

        m_segments.insert(m_segments.begin() + _index, std::move(_seg));
        invalidateCache();
    }

    /**
     * @brief Remove a segment at a specified index.
     *
     * This method removes a segment at the specified index in the contour and invalidates the cache.
     *
     * @param _index The index to remove the segment at.
     *
     * @return True if the segment was removed, false otherwise.
     */
    bool removeSegment(size_t _index)
    {
        if (_index < m_segments.size())
        {
            m_segments.erase(m_segments.begin() + _index);
            invalidateCache();
            return true;
        }

        return false;
    }

    /**
     * @brief Get the number of segments in the contour.
     *
     * @return The number of segments in segments vector.
     */
    size_t segmentCount() const
    {
        return m_segments.size();
    }

    /**
     * @brief Get read-only access to the segments.
     *
     * @return A vector of unique pointers to the segments.
     */
    const std::vector<std::unique_ptr<Segment>>& getSegments() const
    {
        return m_segments;
    }

    /**
     * @brief Get modifiable access to the segments. Alowing iteration and modification of them.
     *
     * @return A vector of unique pointers to the segments.
     */
    std::vector<std::unique_ptr<Segment>>& getSegments()
    {
        return m_segments;
    }

    /**
     * @brief Check if the contour is valid.
     *
     * A contour is considered valid if all segments are connected sequentially (end of segment i to start of segment
     * i+1). We use epsilon for a small margin of error. It is calculated over demand and cached, such that consequent
     * calls to isValid() doesn’t calculate it again. An empty contour is considered invalid.
     *
     * @return True if the contour is valid, false otherwise.
     */
    bool isValid() const
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

private:
    /**
     * @brief Invalidate cached result when segments change.
     *
     * This method invalidates the cache, marking the cached result of the isValid() method as invalid.
     */
    void invalidateCache()
    {
        m_cacheValid = false;
    }

    mutable bool m_cacheValid;   /**< Flag indicating if the cache is valid. */
    mutable bool m_cachedResult; /**< Cached result of the isValid() method. */

    std::vector<std::unique_ptr<Segment>> m_segments; /**< Storage for segments (using polymorphism). */
};

/**
 * @brief Convert degrees to radians.
 *
 * @param _deg The angle in degrees.
 *
 * @return The angle in radians.
 */
double deg2rad(double _deg)
{
    return _deg * M_PI / 180.0;
}

/**
 * @brief Convert radians to degrees.
 *
 * @param _rad The angle in radians.
 *
 * @return The angle in degrees.
 */
double rad2deg(double _rad)
{
    return _rad * 180.0 / M_PI;
}

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
Contour createContourFromPolyline(const std::vector<Point>& _points, bool _closed = false)
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
std::vector<Point> sampleArcPoints(const ArcSegment* _arc)
{
    static const unsigned int samples = 20;

    std::vector<Point> points;

    double angleStep = (_arc->getEndAngle() - _arc->getStartAngle()) / samples;
    for (unsigned int i = 0; i <= samples; i++)
    {
        double angle = _arc->getStartAngle() + i * angleStep;
        points.push_back(_arc->pointAt(angle));
    }

    return points;
}

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
std::vector<Point> getSegmentPoints(const Segment* _seg)
{
    std::vector<Point> points;
    if (auto line = dynamic_cast<const LineSegment*>(_seg))
    {
        points.push_back(line->getStart());
        points.push_back(line->getEnd());
    }
    else if (auto arc = dynamic_cast<const ArcSegment*>(_seg))
    {
        auto arcPoints = sampleArcPoints(arc);
        points.insert(points.end(), arcPoints.begin(), arcPoints.end());
    }

    return points;
}

/**
 * @brief Calculate the bounding box of a contour.
 *
 * This function calculates the bounding box of a contour based on its segments.
 *
 * @param _contour The contour to calculate the bounding box for.
 *
 * @return The bounding box of the contour.
 */
BoundingBox calculateBoundingBox(const Contour& _contour)
{
    BoundingBox bbox;
    for (const auto& seg : _contour.getSegments())
    {
        std::vector<Point> points = getSegmentPoints(seg.get());
        for (const Point& p : points)
        {
            if (p.x < bbox.minX)
            {
                bbox.minX = p.x;
            }
            if (p.y < bbox.minY)
            {
                bbox.minY = p.y;
            }
            if (p.x > bbox.maxX)
            {
                bbox.maxX = p.x;
            }
            if (p.y > bbox.maxY)
            {
                bbox.maxY = p.y;
            }
        }
    }

    return bbox;
}

#ifdef SFML_AVAILABLE

/**
 * @brief Print the coordinates of a mouse click.
 *
 * This function prints the coordinates of a mouse click in an SFML window.
 *
 * @param _window The SFML window.
 * @param _mouseEvent The mouse click event.
 */
void printMouseClickCoordinate(sf::RenderWindow& _window, const sf::Event::MouseButtonEvent& _mouseEvent)
{
    sf::Vector2i pixelPos(_mouseEvent.x, _mouseEvent.y);
    sf::Vector2f worldPos = _window.mapPixelToCoords(pixelPos);
    std::cout << std::fixed << std::setprecision(3) << Point(worldPos.x, worldPos.y) << std::endl;
}

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
BoundingBox calculateBoundingBox(const std::vector<Contour>& _contours, double _margin)
{
    BoundingBox bbox;

    for (const auto& contour : _contours)
    {
        BoundingBox contourBbox = calculateBoundingBox(contour);
        bbox.minX               = std::min(bbox.minX, contourBbox.minX);
        bbox.minY               = std::min(bbox.minY, contourBbox.minY);
        bbox.maxX               = std::max(bbox.maxX, contourBbox.maxX);
        bbox.maxY               = std::max(bbox.maxY, contourBbox.maxY);
    }

    bbox.minX -= _margin;
    bbox.minY -= _margin;
    bbox.maxX += _margin;
    bbox.maxY += _margin;

    return bbox;
}

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
sf::Vertex createVertex(const Point& _point, sf::Color _color)
{
    return sf::Vertex(sf::Vector2f(static_cast<float>(_point.x), static_cast<float>(_point.y)), _color);
}

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
sf::Vertex createVertex(double _x, double _y, sf::Color _color)
{
    return createVertex(Point(_x, _y), _color);
}

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
std::vector<sf::Vertex> createArcVertices(const ArcSegment* _arc, sf::Color _color)
{
    std::vector<sf::Vertex> vertices;

    auto points = sampleArcPoints(_arc);
    for (const auto& p : points)
    {
        vertices.push_back(createVertex(p, _color));
    }

    return vertices;
}

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
bool drawArrow(const Point& start, const Point& end, sf::Color _color, sf::RenderWindow& _window)
{
    Point  d      = end - start;
    double length = d.length();

    if (length < EPSILON)
    {
        return false;
    }

    double ux = d.x / length;
    double uy = d.y / length;

    double arrowLength = 0.03;
    double arrowWidth  = arrowLength;

    Point base = {end.x - arrowLength * ux, end.y - arrowLength * uy};

    double perpX = -uy;
    double perpY = ux;

    Point left  = {base.x + arrowWidth * perpX, base.y + arrowWidth * perpY};
    Point right = {base.x - arrowWidth * perpX, base.y - arrowWidth * perpY};

    sf::ConvexShape arrow;
    arrow.setPointCount(3);
    arrow.setPoint(0, sf::Vector2f(static_cast<float>(end.x), static_cast<float>(end.y)));
    arrow.setPoint(1, sf::Vector2f(static_cast<float>(left.x), static_cast<float>(left.y)));
    arrow.setPoint(2, sf::Vector2f(static_cast<float>(right.x), static_cast<float>(right.y)));
    arrow.setFillColor(_color);

    _window.draw(arrow);

    return true;
}

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
void drawContour(const Contour& _contour, sf::Color _color, sf::RenderWindow& _window)
{
    for (const auto& seg : _contour.getSegments())
    {
        if (auto line = dynamic_cast<LineSegment*>(seg.get()))
        {
            std::vector<sf::Vertex> lineVertices = {createVertex(line->getStart(), _color),
                                                    createVertex(line->getEnd(), _color)};
            _window.draw(&lineVertices[0], lineVertices.size(), sf::Lines);

            drawArrow(line->getStart(), line->getEnd(), _color, _window);
        }
        else if (auto arc = dynamic_cast<ArcSegment*>(seg.get()))
        {
            std::vector<sf::Vertex> arcVertices = createArcVertices(arc, _color);
            _window.draw(&arcVertices[0], arcVertices.size(), sf::LineStrip);

            for (int i = static_cast<int>(arcVertices.size()) - 2; i >= 0; i--)
            {
                Point pPrev = {arcVertices[i].position.x, arcVertices[i].position.y};
                if (drawArrow(pPrev, arc->getEnd(), _color, _window))
                {
                    break;
                }
            }
        }
    }
}

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
BoundingBox setViewFromBoundingBox(sf::RenderWindow& _window, const BoundingBox& _bbox)
{
    double bboxWidth  = _bbox.maxX - _bbox.minX;
    double bboxHeight = _bbox.maxY - _bbox.minY;

    float windowRatio = static_cast<float>(_window.getSize().x) / _window.getSize().y;
    float bboxRatio   = static_cast<float>(bboxWidth / bboxHeight);

    BoundingBox adjustedBbox = _bbox;
    if (bboxRatio < windowRatio)
    {
        double newWidth = bboxHeight * windowRatio;
        double delta    = newWidth - bboxWidth;
        adjustedBbox.minX -= delta / 2.0;
        adjustedBbox.maxX += delta / 2.0;
    }
    else if (bboxRatio > windowRatio)
    {
        double newHeight = bboxWidth / windowRatio;
        double delta     = newHeight - bboxHeight;
        adjustedBbox.minY -= delta / 2.0;
        adjustedBbox.maxY += delta / 2.0;
    }

    sf::View view(sf::FloatRect(static_cast<float>(adjustedBbox.minX),
                                static_cast<float>(adjustedBbox.minY),
                                static_cast<float>(adjustedBbox.maxX - adjustedBbox.minX),
                                static_cast<float>(adjustedBbox.maxY - adjustedBbox.minY)));
    _window.setView(view);

    return adjustedBbox;
}

/**
 * @brief Draw axes in an SFML window.
 *
 * This function draws axes in an SFML window based on a bounding box.
 *
 * @param _window The SFML window to draw the axes in.
 * @param _bbox The bounding box to draw the axes from.
 */
void drawAxes(sf::RenderWindow& _window, const BoundingBox& _bbox)
{
    sf::Color white(255, 255, 255, 128);

    if (_bbox.minX <= 0 && _bbox.maxX >= 0)
    {
        sf::Vertex verticalLine[] = {createVertex(0.0f, _bbox.minY, white), createVertex(0.0f, _bbox.maxY, white)};
        _window.draw(verticalLine, 2, sf::Lines);
    }

    if (_bbox.minY <= 0 && _bbox.maxY >= 0)
    {
        sf::Vertex horizontalLine[] = {createVertex(_bbox.minX, 0.0f, white), createVertex(_bbox.maxX, 0.0f, white)};
        _window.draw(horizontalLine, 2, sf::Lines);
    }
}

/**
 * @brief Get default colors to use for contours.
 *
 * @return A vector of default colors.
 */
std::vector<sf::Color> defaultColors()
{
    std::vector<sf::Color> colors = {
        sf::Color::Red, sf::Color::Green, sf::Color::Blue, sf::Color::Yellow, sf::Color::Magenta, sf::Color::Cyan};
    return colors;
}

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
bool handleEvents(sf::RenderWindow& _window, sf::Event& _event, const BoundingBox& _bbox, BoundingBox& _adjustedBbox)
{
    if (_event.type == sf::Event::Closed)
    {
        _window.close();
        return true;
    }
    else if (_event.type == sf::Event::Resized)
    {
        _adjustedBbox = setViewFromBoundingBox(_window, _bbox);
    }
    else if (_event.type == sf::Event::MouseButtonPressed)
    {
        printMouseClickCoordinate(_window, _event.mouseButton);
    }

    return false;
}

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
                    const BoundingBox&          _adjustedBbox)
{
    _window.clear(sf::Color::Black);
    drawAxes(_window, _adjustedBbox);

    size_t colorIndex = 0;
    for (const auto& contour : _contours)
    {
        if (colorIndex >= _colors.size())
        {
            auto newColor = sf::Color{static_cast<sf::Uint8>(rand() % 256),
                                      static_cast<sf::Uint8>(rand() % 256),
                                      static_cast<sf::Uint8>(rand() % 256)};
            _colors.push_back(newColor);
        }
        auto color = _colors[colorIndex % _colors.size()];
        colorIndex++;

        drawContour(contour, color, _window);
    }

    _window.display();
}

/**
 * @brief Visualize contours in an SFML window.
 *
 * @param _contours The contours to visualize.
 */
void visualizeContours(const std::vector<Contour>& _contours)
{
    srand(static_cast<unsigned int>(time(nullptr)));

    sf::RenderWindow window(sf::VideoMode(800, 600), "Scopic Test Task - Eberty Alves");
    window.setVerticalSyncEnabled(true);
    window.setFramerateLimit(10);

    auto colors       = defaultColors();
    auto bbox         = calculateBoundingBox(_contours, 0.5);
    auto adjustedBbox = setViewFromBoundingBox(window, bbox);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (handleEvents(window, event, bbox, adjustedBbox))
            {
                return;
            }
        }

        renderContours(window, _contours, colors, adjustedBbox);
    }
}

#endif

int main()
{
    [[maybe_unused]] auto contours = contourRoutine();

#ifdef SFML_AVAILABLE
    visualizeContours(contours);
#endif

    return 0;
}
