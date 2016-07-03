#ifndef MPEL_BUILTINS_H
#define MPEL_BUILTINS_H

#include "types.hpp"
#include "planner.hpp"

namespace mpel {
	namespace builtin {

		namespace metric {
			// Distance metrics
			struct euclidean {
				double operator()(PointRef a, PointRef b);
			};

			struct manhattan {
				double operator()(PointRef a, PointRef b);
			};

			struct chebychev {
				double operator()(PointRef a, PointRef b);
			};
		}

		// Graph builders
		namespace graph_builder {
			struct none {
				none();
				Graph operator()(MapRef map);
			};

			struct voronoi {
				voronoi(double eps = 10);
				Graph operator()(MapRef map);
				private:
				double _eps; // parameter for approximating the workspace
			};

			struct probabilistic {
				probabilistic(size_t n = 0);
				Graph operator()(MapRef map);
				private:
				size_t _n; // number of nodes in the graph 0 => automatically determined
			};
		}

		// Graph searches
		namespace graph_search {
			struct none { // does nothing meaningful (only for debugging)
				none();
				Path operator()(GraphRef g, PointRef a, PointRef b);
			};

			struct a_star {
				a_star();
				Path operator()(GraphRef g, PointRef a, PointRef b);
			};

			struct dijkstra {
				dijkstra();
				Path operator()(GraphRef g, PointRef a, PointRef b);
			};

			struct breadth_first {
				breadth_first();
				Path operator()(GraphRef g, PointRef a, PointRef b);
			};
		}

		// Interpolators
		namespace interpolator {
			struct none {
				none();
				Path operator()(MapRef map, PathRef path);
			};

			struct bug2 {
				bug2(double step = 2);
				Path operator()(MapRef map, PathRef path);
				private:
				double _step;
			};

			struct potential_field {
				potential_field();
				Path operator()(MapRef map, PathRef path);
			};

			struct a_star {
				a_star();
				Path operator()(MapRef map, PathRef path);
				private:
				cv::Mat _dt; // distance transform image
			};
		}

		// Planner configs
		struct voronoi_planner_config : Planner::Config {
			voronoi_planner_config();
		};

		struct PRM_planner_config : Planner::Config {
			PRM_planner_config();
		};
	}
}
#endif

