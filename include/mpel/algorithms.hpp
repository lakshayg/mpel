#ifndef MPEL_ALGORITHMS_HPP
#define MPEL_ALGORITHMS_HPP

#include <vector>

/* function to select k best options from a container, the elements are compared
 * using a comparison function and the function returns a vector containing these elements
 * the  comparison function must return true whenever the first argument of the input is
 * better or same as the second argument
 */
template <typename Iter, typename Compare>
std::vector<typename Iter::value_type> k_best(Iter begin, Iter end, size_t k, Compare comp)
{
    std::vector<typename Iter::value_type> out(begin, begin + k);
    std::make_heap(out.begin(), out.end(), comp);
    for (Iter it = begin + k; it != end; ++it) {
        if (comp(*it, out.front())) {
            std::pop_heap(out.begin(), out.end(), comp);
            out.back() = *it;
            std::push_heap(out.begin(), out.end(), comp);
        }
    }
    return out;
}

#endif
