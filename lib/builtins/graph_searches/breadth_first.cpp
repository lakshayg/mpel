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

            std::vector<size_t> parent(g.num_vertices());
            std::vector<bool> visited(g.num_vertices());
            std::queue<size_t> Q;
            Q.push(in);
            visited[in] = true;
            parent[in] = g.num_vertices();

            while (not Q.empty()) {
                size_t curr = Q.front();
                Q.pop();
                if (curr == out) {
                    while (parent[curr] != g.num_vertices()) {
                        path.push_back(g.vertex(curr));
                        curr = parent[curr];
                    }
                    path.push_back(g.vertex(curr));
                    std::reverse(path.begin(), path.end());
                    return path;
                }

                // push all the neighbors of curr into the queue
                for (size_t i = 0; i < g.num_vertices(); ++i) {
                    // if not connected or already visited
                    if ((g.weight(curr, i) < 0) or visited[i]) continue;
                    visited[i] = true;
                    parent[i] = curr;
                    Q.push(i);
                }
            }
            return path;
        }
    }
}
}
