#include "types.hpp"
#include <iostream>
#include <queue>

namespace mpel {
Graph::Graph()
    : nvertex(0)
{
}

Graph::~Graph() {}

Graph::vertex_t Graph::add_vertex(Point pt)
{
    vertex_t p;
    auto v = vdesc.find(pt);                       // check if pt is already present
    if (v == vdesc.end()) {                        // point not present
        vdesc.insert(std::make_pair(pt, nvertex)); // insert in descriptor list
        vertices.push_back(pt);
        p = nvertex;
        allocate_vertex(); // increments nvertex
    } else {
        p = v->second;
    }
    return p;
}

void Graph::add_edge(Point pt1, Point pt2, double weight)
{
    vertex_t p1, p2;
    p1 = add_vertex(pt1);
    p2 = add_vertex(pt2);
    // connect the vertices and add cost
    g[p1][p2] = weight;
    g[p2][p1] = weight;
}

size_t Graph::descriptor(PointRef pt) const { return vdesc.at(pt); }

size_t Graph::num_vertices() const { return nvertex; }

Point Graph::vertex(size_t n) const { return vertices.at(n); }

double Graph::weight(size_t a, size_t b) const { return g.at(a).at(b); }

double Graph::weight(PointRef a, PointRef b) const
{
    auto p1 = descriptor(a);
    auto p2 = descriptor(b);
    return g.at(p1).at(p2);
}

const std::vector<Point>& Graph::vertex_list() const { return vertices; }

bool Graph::connected() const
{
    if (num_vertices() == 0) return true;
    // check if the graph is connected by performing a BFS
    std::queue<size_t> Q;
    std::vector<bool> visited(num_vertices());
    visited[0] = true;
    size_t nvisited = 0;

    Q.push(0);
    while (not Q.empty()) {
        nvisited++;
        size_t curr = Q.front();
        Q.pop();
        for (size_t i = 0; i < num_vertices(); ++i) {
            if (weight(curr, i) < 0)
                continue; // not connected
            else if (not visited[i]) {
                visited[i] = true;
                Q.push(i);
            }
        }
    }
    return nvisited == num_vertices();
}

void Graph::allocate_vertex()
{
    std::vector<double> v(nvertex + 1, -1);
    g.push_back(v);
    for (size_t i = 0; i < nvertex; ++i) g[i].push_back(-1);
    nvertex++;
}
}
