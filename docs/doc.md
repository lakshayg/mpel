# API Reference

This section gives a detailed description of the API. It is strongly
suggested that you go through the [Primer](basics.md) before
reading the API Reference. If you are looking for tutorials, they
are available [here](tut.md).

MPEL utilizes [OpenCV](http://opencv.org/) for a lot a operations in
the background. Several datatypes are borrowed directly from OpenCV.
In such cases the code provided if merely a representation of the
actual implementation.

## **`mpel/core.hpp`**

This header contains everything which is required for implementing
one's own planning algorithm. The header is further divided into
related sections

### Datatypes

#### Point
```
struct Point {
	int x, y;  // NOTE: Only integer coordinates are allowed
	Point();
	Point(int X, int Y) : x(X), y(Y) {}
};
typedef const Point& PointRef;
```
The `Point` class supports several operations like addition,
subtraction and multiplication with scalar.

#### Segment
```
struct Segment {
	Point p0, p1;
	Segment(PointRef a, PointRef b) : p0(a), p1(b) {}
};
typedef const Segment& SegmentRef;
```
The `Segment` datatype is a struct containing two points.

#### Path
```
typedef std::vector<Point> Path;
typedef const Path& PathRef;
```
The `Path` datatype is implemented as a vector of points and therefore
supports all the operations allowed on a `std::vector`.

#### Map

```
typedef cv::Mat Map;
typedef const Map& MapRef;
```
A `Map` is a 2D matrix which represents the workspace as an occupancy
grid. Map supports the `clone()` operation which is required whenever
a Map needs to be copied.

#### Graph

```
class Graph {
public:
	typedef size_t vertex_t;

	Graph();
	~Graph();
	void add_edge(Point pt1, Point pt2, double weight = 0);
	size_t descriptor(PointRef pt) const;
	size_t num_vertices() const;
	Point vertex(size_t descriptor) const;
	double weight(PointRef a, PointRef b) const;
	double weight(size_t a, size_t b) const;
	const std::vector<Point>& vertex_list() const;
private:
	...
};
typedef const Graph& GraphRef;
```

The current implementation of `Graph` is an _incidence matrix_. It
supports the following operations:

- `add_edge(Point pt1, Point pt2, double weight = 0)` - Add a new edge
in the graph. Two nodes can have only a single edge between them. A
negative weight indicates that no edge is present.
- `size_t descriptor(PointRef pt) const` - Every vertex is assigned a
descriptor in the graph. This function returns the vertex descriptor.
- `size_t num_vertices() const` - Returns the number of vertices in the graph.
- `Point vertex(size_t descriptor) const` - Returns the vertex (Point)
corresponding to the descriptor given as input
- `double weight(PointRef a, PointRef b) const` - Returns the weight of
edge given two vertices. A negative weight indicates that the verties
are not connected.
- `double weight(size_t a, size_t b) const` - Returns the weight of edge
given two vertex descriptors.
- `const std::vector<Point>& vertex_list() const` - Returns the list of
vertices present in the graph.

#### Workspace
```
struct Workspace {
	Map map;
};
```

#### ProblemDefinition

```
struct ProblemDefinition {
	Point start;
	Point goal;
};
```
This datatype is used to specify the planning problem and pass it to
the motion planner.

### Utility Functions

### Motion Planner

### Visualization

## **`mpel/builtins.hpp`**

This header file contains definitions of all the pre-built components
provided with the library. The section is divided into related sections.

### Graph Builders

### Graph Searches

### Interpolators

### Planner Configs
