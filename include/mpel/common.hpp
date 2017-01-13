/**
 * \file common.hpp
 * \brief Contains helper functions which can be used as a part of larger algorithms.
 */
#ifndef MPEL_COMMON_H
#define MPEL_COMMON_H
#include "types.hpp"
#include <exception>
#include <opencv2/opencv.hpp>

namespace mpel {

/// Exception thrown whenever an unimplemented feature is used
struct UnimplementedException : public std::exception {
    const char* what() const throw() { return "Unimplemented Functionality"; }
};

/// Helper function to load a map stored as an image
Map load_map_from_image(std::string filename);

/// Compute squared distance between two points
double sqr_distance(PointRef a, PointRef b);
/// Compute distance between two points
double distance(PointRef a, PointRef b);
/// Compute perpendicular distance between a segment and a point
double distance(SegmentRef s, PointRef a);
/// Compute perpendicular distance between a segment and a point
double distance(PointRef a, SegmentRef s);

/// Check if a point is colliding with an obstacle in a map
bool is_collision(MapRef map, PointRef pt);

/// Check if a segment is colliding with an obstacle in a map
bool is_collision(MapRef map, SegmentRef s);

/// Check if a given point lies in a segment
bool on_segment(SegmentRef s, PointRef p);

/**
 * \brief Get a list of obstacle boundaries in the map.
 *
 * \param eps Specifies the maximum allowed error in pixels when
 * approximating curved obstacles in the map
 */
std::vector<Segment> get_map_segments(MapRef map, double eps = 10);

Path subdivide_path(PathRef path, double len = 40);

/// Returns a random non-colliding point in the map
Point random_free_space_point(const Map& map);

/// Opens a window to mark a point in a map
Point mark_point(const Workspace& ws);
}
#endif
