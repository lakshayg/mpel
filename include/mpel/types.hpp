#ifndef MPEL_TYPES_H
#define MPEL_TYPES_H

#include <functional>
#include <opencv2/core/core.hpp>
#include <unordered_map>

namespace mpel {
typedef cv::Point Point;
typedef const Point& PointRef;
}

namespace std {
template <> struct hash<mpel::Point> {
    size_t operator()(mpel::PointRef pt) const { return 10000 * pt.x + pt.y; }
};
}

namespace mpel {
struct Segment {
    Point p0, p1;
    Segment(PointRef a, PointRef b)
        : p0(a)
        , p1(b)
    {
    }
};
typedef const Segment& SegmentRef;

typedef std::vector<Point> Path;
typedef const Path& PathRef;

typedef cv::Mat Map;
typedef const Map& MapRef;

// A very very limited and inefficient implementation of graph
//
// each node contains a Point
// only add_edge operation is required
// -ve cost indicates not connected
class Graph {
public:
    typedef size_t vertex_t;

    Graph();
    ~Graph();
    vertex_t add_vertex(Point pt);
    void add_edge(Point pt1, Point pt2, double weight = 0);
    size_t descriptor(PointRef pt) const;
    size_t num_vertices() const;
    Point vertex(size_t n) const;
    double weight(PointRef a, PointRef b) const;
    double weight(size_t a, size_t b) const;
    const std::vector<Point>& vertex_list() const;
    bool connected() const;

private:
    std::vector<std::vector<double>> g;
    std::vector<Point> vertices;
    std::unordered_map<Point, vertex_t> vdesc;
    size_t nvertex;
    void allocate_vertex();
};
typedef const Graph& GraphRef;

typedef std::function<double(PointRef, PointRef)> Metric;
typedef std::function<Graph(MapRef)> GraphBuilderFn;
typedef std::function<Path(GraphRef, PointRef, PointRef)> GraphSearchFn;
typedef std::function<Path(MapRef, PathRef)> InterpolatorFn;

struct Workspace {
    Map map;
};

struct ProblemDefinition {
    Point start;
    Point goal;
};
}

#endif
