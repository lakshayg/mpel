#include "common.hpp"
#include "types.hpp"
#include <mpel/builtins.hpp>
#include <queue>
#include <vector>

namespace mpel {
namespace builtin {
    namespace graph_search {
        // breadth first search
        breadth_first::breadth_first() {}
        Path breadth_first::operator()(GraphRef g, PointRef a, PointRef b)
        {
            Path path;

            size_t in = g.descriptor(a);
            size_t out = g.descriptor(b);

            double max_dist = std::numeric_limits<double>::max();
            std::vector<double> dist(g.num_vertices(), max_dist);
            std::vector<size_t> parent(g.num_vertices());
            std::queue<size_t> Q;
            Q.push(in);

            dist[in] = 0;
            parent[in] = in;

            while (not Q.empty()) {
                size_t curr = Q.front();
                Q.pop();

                for (size_t i = 0; i < g.num_vertices(); ++i) {
                    if (g.weight(curr, i) < 0) continue; // not connected
                    if (dist[i] == max_dist) {
                        dist[i] = dist[curr] + g.weight(curr, i);
                        parent[i] = curr;
                        Q.push(i);
                        // if this is the goal
                        if (curr == out) {
                            while (parent[curr] != curr) {
                                path.push_back(g.vertex(curr));
                                curr = parent[curr];
                            }
                            path.push_back(g.vertex(curr));
                            std::reverse(path.begin(), path.end());
                            return path;
                        }
                    }
                }
            }
            return path;
        }
    }
}
}
