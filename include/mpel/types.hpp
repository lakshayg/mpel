#ifndef MPEL_TYPES_H
#define MPEL_TYPES_H

#include <opencv2/core/core.hpp>
#include <unordered_map>
#include <vector>

namespace mpel {
/// The point datatype
typedef cv::Point Point;
typedef const Point& PointRef;
}

namespace std {
/// Hash for point type. Useful when using a map of points
template <> struct hash<mpel::Point> {
    size_t operator()(mpel::PointRef pt) const { return 10000 * pt.x + pt.y; }
};
}

namespace mpel {
/// The segment datatype
struct Segment {
    Point p0, p1;
    Segment(PointRef a, PointRef b)
        : p0(a)
        , p1(b)
    {
    }
};
typedef const Segment& SegmentRef;

/// The path datatype
typedef std::vector<Point> Path;
typedef const Path& PathRef;

/// The map datatype
typedef cv::Mat Map;
typedef const Map& MapRef;

// A very very limited and inefficient implementation of graph
//
// each node contains a Point
// only add_edge operation is required
// -ve cost indicates not connected
/**
 * \brief The graph datatype.
 *
 * This class implements a weighted, undirected graph. The graph
 * weights are always positive. negative weights represent the
 * absence of an edge between two vertices.
 */
class Graph {
public:
    typedef size_t vertex_t;

    /// The default constructor
    Graph();

    ~Graph();

    /// Adds vertex to the graph
    /// Vertices are unique and added only if it is not already present
    /// \return The vertex descriptor
    vertex_t add_vertex(Point pt);

    /// Adds an edge between two vertices
    /// Only one edge can be added between two vertices, the older edge
    /// is replaced if multiple edges are added. If the vertices are not
    /// already present in the graph, they are inerted as well.
    /// \param pt1 First vertex
    /// \param pt2 Second vertex
    /// \param weight The weight of this edge
    void add_edge(Point pt1, Point pt2, double weight = 0);

    /// Returns descriptor of the vertex
    size_t descriptor(PointRef pt) const;

    /// Returns number of vertices in the graph
    size_t num_vertices() const;

    /// Returns the vertex given its descriptor
    Point vertex(size_t n) const;

    /// Returns weight of the edge joining vertices a and b
    /// A negative value is returned if the vertices are not connected
    double weight(PointRef a, PointRef b) const;

    /// Returns weight of the edge joining two vertices given their descriptors
    /// A negative value is returned if the vertices are not connected
    double weight(size_t a, size_t b) const;

    /// Returns a vector of vertices in the graph
    const std::vector<Point>& vertex_list() const;

    /// Returns true if the graph is connected
    bool connected() const;

private:
    std::vector<std::vector<double>> g;
    std::vector<Point> vertices;
    std::unordered_map<Point, vertex_t> vdesc;
    size_t nvertex;
    void allocate_vertex();
};
typedef const Graph& GraphRef;

/// The workspace datatype
struct Workspace {
    Map map;
};

/// The problem definition datatype
struct ProblemDefinition {
    Point start;
    std::vector<Point> via; // via points (unimplemented)
    Point goal;
};
}

#endif
