/*
 * Copyright (c) 2025, Eberty Alves
 */

#include "SFML.h"

#ifdef SFML_AVAILABLE

#include "LineSegment.h"

#include <iomanip>

#ifndef EPSILON
#define EPSILON 1e-6
#endif

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

void printMouseClickCoordinate(sf::RenderWindow& _window, const sf::Event::MouseButtonEvent& _mouseEvent)
{
    sf::Vector2i pixelPos(_mouseEvent.x, _mouseEvent.y);
    sf::Vector2f worldPos = _window.mapPixelToCoords(pixelPos);
    std::cout << std::fixed << std::setprecision(3) << Point(worldPos.x, worldPos.y) << std::endl;
}

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

sf::Vertex createVertex(const Point& _point, sf::Color _color)
{
    return sf::Vertex(sf::Vector2f(static_cast<float>(_point.x), static_cast<float>(_point.y)), _color);
}

sf::Vertex createVertex(double _x, double _y, sf::Color _color)
{
    return createVertex(Point(_x, _y), _color);
}

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

std::vector<sf::Color> defaultColors()
{
    std::vector<sf::Color> colors = {
        sf::Color::Red, sf::Color::Green, sf::Color::Blue, sf::Color::Yellow, sf::Color::Magenta, sf::Color::Cyan};
    return colors;
}

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
        // Used to debug the coordinates
        // printMouseClickCoordinate(_window, _event.mouseButton);
    }

    return false;
}

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
