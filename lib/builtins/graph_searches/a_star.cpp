#include "common.hpp"
#include "types.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <mpel/builtins.hpp>
#include <queue>
#include <vector>

#define LOG(msg)                                                                                                       \
    {                                                                                                                  \
        std::cout << msg << std::endl;                                                                                 \
    }

namespace mpel {
namespace builtin {
    namespace graph_search {
        // a star search
        a_star::a_star() {}

        double dist(GraphRef g, size_t i, size_t j)
        {
            Point a = g.vertex(i);
            Point b = g.vertex(j);
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            return sqrt(dx * dx + dy * dy);
        }

        bool contains(const std::vector<size_t>& v, size_t e)
        {
            for (size_t x : v)
                if (x == e) return true;
            return false;
        }

        Path a_star::operator()(GraphRef g, PointRef a, PointRef b)
        {
            Path path;

            size_t in = g.descriptor(a);
            size_t out = g.descriptor(b);

            std::vector<size_t> open, closed;
            open.reserve(g.num_vertices());
            closed.reserve(g.num_vertices());

            std::vector<double> fcost(g.num_vertices(), std::numeric_limits<double>::max());
            std::vector<double> gcost(g.num_vertices(), std::numeric_limits<double>::max());
            std::vector<size_t> parent(g.num_vertices(), g.num_vertices());
            fcost[in] = dist(g, in, out);
            gcost[in] = 0;
            parent[in] = in;
            open.push_back(in);

            while (not open.empty()) {
                // pick the element in open list with minimum fcost
                size_t curr = open.front();
                double min_cost = fcost[curr];
                for (auto e : open) {
                    if (fcost[e] <= min_cost) {
                        min_cost = fcost[e];
                        curr = e;
                    }
                }

                // remove curr from open list and insert in the closed list
                open.erase(std::remove(open.begin(), open.end(), curr), open.end());
                closed.push_back(curr);

                if (curr == out) {
                    while (parent[curr] != curr) {
                        path.push_back(g.vertex(curr));
                        curr = parent[curr];
                    }
                    path.push_back(g.vertex(curr));
                    std::reverse(path.begin(), path.end());
                    return path;
                }

                // assign cost to all neighbors of curr
                for (size_t n = 0; n < g.num_vertices(); ++n) {
                    if (g.weight(n, curr) <= 0) continue;
                    if (contains(closed, n)) continue;

                    double tmp_g = gcost[curr] + g.weight(curr, n);
                    if (tmp_g < gcost[n]) {
                        gcost[n] = tmp_g;
                        fcost[n] = gcost[n] + dist(g, out, n);
                        parent[n] = curr;
                    }
                    if (not contains(open, n)) open.push_back(n);
                }
            }

            return path;
        }
    }
}
}
