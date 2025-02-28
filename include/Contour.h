/*
 * Copyright (c) 2025, Eberty Alves
 */

#pragma once

#include "Segment.h"

#include <memory>
#include <vector>

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
    Contour();

    /**
     * @brief Copy constructor for Contour.
     *
     * This constructor performs a deep copy of the segments in the other contour.
     *
     * @param _other The other contour to copy.
     */
    Contour(const Contour& _other);

    /**
     * @brief Move constructor for Contour.
     *
     * This constructor moves the segments from the other contour.
     *
     * @param _other The other contour to move.
     */
    Contour(Contour&& _other) noexcept;

    /**
     * @brief Copy assignment operator for Contour.
     *
     * This operator performs a deep copy of the segments in the other contour.
     *
     * @param _other The other contour to copy.
     *
     * @return A reference to this contour.
     */
    Contour& operator=(const Contour& _other);

    /**
     * @brief Move assignment operator for Contour.
     *
     * This operator moves the segments from the other contour.
     *
     * @param _other The other contour to move.
     *
     * @return A reference to this contour.
     */
    Contour& operator=(Contour&& _other) noexcept;

    /**
     * @brief Add a segment to the contour.
     *
     * This method adds a segment to the end of the contour and invalidates the cache.
     *
     * @param _seg The segment to add.
     */
    void addSegment(std::unique_ptr<Segment> _seg);

    /**
     * @brief Insert a segment at a specified index.
     *
     * This method inserts a segment at the specified index in the contour and invalidates the cache. If the index is
     * greater than the number of segments, the segment is added to the end of the contour segment list.
     *
     * @param _index The index to insert the segment at.
     * @param _seg The segment to insert.
     */
    void insertSegment(size_t _index, std::unique_ptr<Segment> _seg);

    /**
     * @brief Remove a segment at a specified index.
     *
     * This method removes a segment at the specified index in the contour and invalidates the cache.
     *
     * @param _index The index to remove the segment at.
     *
     * @return True if the segment was removed, false otherwise.
     */
    bool removeSegment(size_t _index);

    /**
     * @brief Get the number of segments in the contour.
     *
     * @return The number of segments in segments vector.
     */
    size_t segmentCount() const;

    /**
     * @brief Get read-only access to the segments.
     *
     * @return A vector of unique pointers to the segments.
     */
    const std::vector<std::unique_ptr<Segment>>& getSegments() const;

    /**
     * @brief Get modifiable access to the segments. Alowing iteration and modification of them.
     *
     * @return A vector of unique pointers to the segments.
     */
    std::vector<std::unique_ptr<Segment>>& getSegments();

    /**
     * @brief Check if the contour is valid.
     *
     * A contour is considered valid if all segments are connected sequentially (end of segment i to start of segment
     * i+1). We use epsilon for a small margin of error. It is calculated over demand and cached, such that consequent
     * calls to isValid() doesn’t calculate it again. An empty contour is considered invalid.
     *
     * @return True if the contour is valid, false otherwise.
     */
    bool isValid() const;

private:
    /**
     * @brief Invalidate cached result when segments change.
     *
     * This method invalidates the cache, marking the cached result of the isValid() method as invalid.
     */
    void invalidateCache();

    mutable bool m_cacheValid;   /**< Flag indicating if the cache is valid. */
    mutable bool m_cachedResult; /**< Cached result of the isValid() method. */

    std::vector<std::unique_ptr<Segment>> m_segments; /**< Storage for segments (using polymorphism). */
};
