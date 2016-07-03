#include "test.hpp"
#include <iostream>
#include <mpel/core.hpp>

#define npoints 20

using namespace mpel;

void random_test()
{
    View v("Random Points");
    Map m = load_map_from_image("../../workspace/004.bmp");
    v.add_layer(m);
    for (size_t i = 0; i < npoints; ++i) {
        Point pt = random_free_space_point(m);
        v.add_layer(pt);
    }
    View::stay();
}

int main()
{
    MPEL_TEST(random_test);
    return 0;
}
