#include "common.hpp"
#include "types.hpp"
#include <iostream>
#include <mpel/builtins.hpp>
#include <random>
#include <vector>

namespace mpel {
namespace builtin {
    namespace graph_builder {

        randomtrees::randomtrees(Point root, size_t n, size_t len)
            : _root(root), _n(n), _len(len)
        {
        }

        Point randomtrees::closest_point(GraphRef g, PointRef p) {
            auto ret = g.vertex(0);
            double min_dist = sqr_distance(ret, p);
            for (auto v: g.vertex_list()) {
                double d = sqr_distance(v, p);
                if (d < min_dist) {
                    min_dist = d;
                    ret = v;
                }
            }
            return ret;
        }

        Graph randomtrees::operator()(MapRef map)
        {
            // determine number of nodes
            size_t num = (_n > 0 ? _n : (map.rows * map.cols) / (15 * 15));
            Graph g; g.add_vertex(_root);

            auto partition_segment = [](PointRef start, PointRef goal, double len) {
                double dist = distance(start, goal);
                return start + (len/dist) * (goal-start);
            };

            // sample random configurations in the graph
            for (size_t node_count = 1; node_count < num; ) {
                Point pt = random_free_space_point(map);
                
                // find the closest node in the tree
                Point neigh = closest_point(g, pt);
                double dist = distance(neigh, pt);
                if (dist > _len) {
                    pt = partition_segment(neigh, pt, _len);
                }

                int q = 5;
                for (; q > 0 and is_collision(map, Segment(neigh, pt)); --q) {
                    dist /= 2;
                    pt = partition_segment(neigh, pt, dist);
                }

                // add the computed node in the tree
                if (q) {
                    g.add_edge(neigh, pt, dist);
                    node_count++;
                }
            }

            return g;
        }
    }
}
}
