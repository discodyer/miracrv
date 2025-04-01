#ifndef BOUND_HPP
#define BOUND_HPP

template <typename T>
constexpr T Bound(T const& a, T const& limit) {
    return std::max(std::min(a, limit), -limit);
}

#endif // BOUND_HPP