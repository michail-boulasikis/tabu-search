//
// Created by mike on 2022-12-23.
//

#ifndef TABU_SEARCH_EIGHTQUEENS_HPP
#define TABU_SEARCH_EIGHTQUEENS_HPP

#include <array>
#include <functional>
#include <vector>

namespace tabu_examples {

class EightQueens {
public:
    // The point in our space is the locations of the queens. The numbers in
    // this array are from 1 to 64, each number representing a square on the
    // chess board.
    using point_type = std::array<int, 8>;

private:
    // Used as storage for the current point.
    point_type _queens;

public:
    // Just a regular constructor.
    explicit EightQueens(const point_type& queens);

    // Returns the configuration of the queens. For debugging or access purposes.
    [[nodiscard]] const point_type& queens() const;

    // Overloading the [] operator for easy access to the queens, as well as
    // ability to set their squares.
    int operator[](int i) const;

    int& operator[](int i);

    // Function required by the tabu search algorithm. It returns the set of
    // possible moves from the supplied point.
    [[nodiscard]] std::vector<point_type> get_moves(const point_type& queens);

    // Returns the number of queens that are attacking each other, used as the
    // cost function of the tabu search algorithm.
    [[nodiscard]] static double eval(const point_type& queens);

    [[nodiscard]] static point_type get_random_point();
};

void print(const EightQueens::point_type& queens);

} // namespace tabu_examples

// overload operator== for arrays so that it checks all elements for equality
// and returns true if all are equal. Why this is not in the standard library
// is beyond me, especially considering that most containers have this.
template <typename T, std::size_t N> bool operator==(const std::array<T, N>& lhs, const std::array<T, N>& rhs)
{
    for (int i = 0; i < N; i++) {
        if (lhs[i] != rhs[i]) {
            return false;
        }
    }
    return true;
}

#endif // TABU_SEARCH_EIGHTQUEENS_HPP
