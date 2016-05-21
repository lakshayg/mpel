#include "voronoi_boost.h"
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;

// void sample_curved_edge(voronoi_edge edge, std::vector<point_type>* sampled_edge) {
// 	coordinate_type max_dist = 1E-3 * (xh(brect_) - xl(brect_));
// 	point_type point = edge.cell()->contains_point() ?
//     retrieve_point(*edge.cell()) :
//     retrieve_point(*edge.twin()->cell());
// 	segment_type segment = edge.cell()->contains_point() ? retrieve_segment(*edge.twin()->cell()) : retrieve_segment(*edge.cell());
// 	voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_dist, sampled_edge);
// }


//   point_type retrieve_point(const cell_type& cell) {
//     source_index_type index = cell.source_index();
//     source_category_type category = cell.source_category();
//     if (category == SOURCE_CATEGORY_SINGLE_POINT) {
//       return point_data_[index];
//     }
//     index -= point_data_.size();
//     if (category == SOURCE_CATEGORY_SEGMENT_START_POINT) {
//       return low(segment_data_[index]);
//     } else {
//       return high(segment_data_[index]);
//     }
//   }

//   segment_type retrieve_segment(const cell_type& cell) {
//     source_index_type index = cell.source_index() - point_data_.size();
//     return segment_data_[index];
//   }

//   point_type shift_;
//   std::vector<point_type> point_data_;
//   std::vector<segment_type> segment_data_;
//   rect_type brect_;
//   VB vb_;
//   VD vd_;
//   bool brect_initialized_;
//   bool primary_edges_only_;
//   bool internal_edges_only_;
// };
