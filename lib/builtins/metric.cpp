#include "mpel/builtins.hpp"
#include "types.hpp"

namespace mpel {
namespace builtin {
    namespace metric {
        double euclidean::operator()(PointRef a, PointRef b)
        {
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            return sqrt(dx * dx + dy * dy);
        }

        double manhattan::operator()(PointRef a, PointRef b)
        {
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            return std::abs(dx) + std::abs(dy);
        }

        double chebychev::operator()(PointRef a, PointRef b)
        {
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            return std::max(std::abs(dx), std::abs(dy));
        }
    }
}
}
