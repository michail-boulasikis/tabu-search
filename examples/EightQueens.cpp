//
// Created by mike on 2022-12-23.
//

#include "EightQueens.hpp"

namespace tabu_examples {

double EightQueens::eval(const EightQueens::point_type &queens)
{
    double cost{0.0};
    for (int i = 0; i < 8; i++) {
        int row = (queens[i] - 1) / 8;
        int col = (queens[i] - 1) % 8;
        for (int j = i + 1; j < 8; j++) {
            int row2 = (queens[j] - 1) / 8;
            int col2 = (queens[j] - 1) % 8;
            if (row == row2 || col == col2 || row - col == row2 - col2 || row + col == row2 + col2) { cost += 1.0; }
        }
    }
    return cost;
}
std::vector<EightQueens::point_type> EightQueens::get_moves(const EightQueens::point_type &queens)
{
    std::vector<EightQueens::point_type> moves;
    // First, create an array with all the free squares on the board.
    std::array<bool, 64> free_squares{};
    free_squares.fill(true);
    for (int i = 0; i < 8; i++) { free_squares[queens[i] - 1] = false; }
    // Then, for each queen, create all moves that move it to each free
    // square.
    for (int i = 0; i < 8; i++) {
        for (int j = 1; j <= 64; j++) {
            if (free_squares[j - 1]) {
                EightQueens::point_type new_queens = queens;
                new_queens[i] = j;
                moves.emplace_back(new_queens);
            }
        }
    }
    // Finally, return the moves.
    return moves;
}

void print(const EightQueens::point_type& pt)
{
    std::array<bool, 64> free_squares{};
    free_squares.fill(true);
    for (int i = 0; i < 8; i++) { free_squares[pt[i] - 1] = false; }
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            if (!free_squares[i * 8 + j]) {
                std::cout << "Q ";
            } else {
                std::cout << "* ";
            }
        }
        std::cout << std::endl;
    }
}
EightQueens::point_type EightQueens::get_random_point() {
        EightQueens::point_type queens;
        for (int i = 0; i < 8; i++) {
                queens[i] = i * 8 + rand() % 8 + 1;
        }
        return queens;
}

}  // namespace tabu_examples