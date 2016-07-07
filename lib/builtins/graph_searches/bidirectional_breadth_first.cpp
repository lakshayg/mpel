#include "common.hpp"
#include "types.hpp"
#include <mpel/builtins.hpp>
#include <queue>
#include <vector>

namespace mpel {
namespace builtin {
    namespace graph_search {

        bidirectional_breadth_first::bidirectional_breadth_first() {}
        Path bidirectional_breadth_first::operator()(GraphRef g, PointRef a, PointRef b)
        {
            size_t in = g.descriptor(a);
            size_t out = g.descriptor(b);

            std::vector<size_t> parent_in(g.num_vertices());
            std::vector<size_t> parent_out(g.num_vertices());
            std::vector<bool> visited_in(g.num_vertices(), 0);
            std::vector<bool> visited_out(g.num_vertices(), 0);
            std::queue<size_t> Q_in, Q_out;

            Q_in.push(in);
            Q_out.push(out);
            parent_in[in] = g.num_vertices();
            parent_out[out] = g.num_vertices();

            while (not(Q_in.empty() and Q_out.empty())) {
                size_t curr_in = Q_in.front();
                Q_in.pop();
                visited_in[curr_in] = true;
                size_t curr_out = Q_out.front();
                Q_out.pop();
                visited_out[curr_out] = true;

                if (visited_out[curr_in] or visited_in[curr_out]) {
                    size_t tmp;
                    if (visited_out[curr_in])
                        tmp = curr_in;
                    else /*(visited_in[curr_out])*/
                        tmp = curr_out;

                    Path p1;
                    size_t curr = tmp;
                    while (parent_in[curr] != g.num_vertices()) {
                        p1.push_back(g.vertex(curr));
                        curr = parent_in[curr];
                    }
                    p1.push_back(a);
                    std::reverse(p1.begin(), p1.end());
                    curr = tmp;
                    while (parent_out[curr] != g.num_vertices()) {
                        p1.push_back(g.vertex(curr));
                        curr = parent_out[curr];
                    }
                    p1.push_back(b);
                    return p1;
                }

                // push unvisited neighbors of curr_{in,out} in the queue
                for (size_t i = 0; i < g.num_vertices(); ++i) {
                    if ((g.weight(curr_in, i) >= 0) and not visited_in[i]) {
                        visited_in[i] = true;
                        parent_in[i] = curr_in;
                        Q_in.push(i);
                    }
                    if ((g.weight(curr_out, i) >= 0) and not visited_out[i]) {
                        visited_out[i] = true;
                        parent_out[i] = curr_out;
                        Q_out.push(i);
                    }
                }
            }
            Path path;
            return path;
        }
    }
}
}
