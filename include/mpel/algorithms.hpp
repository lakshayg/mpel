/**
 * \file algorithms.hpp
 * \brief This file contains generic implementations of algorithms
 *        used at various places in the library
 */
#ifndef MPEL_ALGORITHMS_HPP
#define MPEL_ALGORITHMS_HPP

#include <vector>

/**
 * \param begin Interator to the first element in the range
 * \param end Iterator to the of the container
 * \param k Number of elements to select
 * \param comp A comparison function to select better of two elements. It must return
 * true when the first argument is better than or same as second argument
 * \return A vector containing k best items from the elements specified by iterators
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
