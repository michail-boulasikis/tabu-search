#include <iostream>

#include "TabuSearch.hpp"
#include "examples/EightQueens.hpp"

int main()
{
    // Initialize the problem by placing all queens on the first row
    tabu_examples::EightQueens problem { std::array<int, 8> { 1, 2, 3, 4, 5, 6, 7, 8 } };

    // Create the tabu algorithm configuration, with the problem, the tabu list
    // size, and the neighborhood exploration percentage.
    tabu::TabuSearch<tabu_examples::EightQueens> tabu_search { problem, 2000, 0.15 };
    // Seed the random number generator with a constant value for reproducibility.
    tabu_search.seed(tabu::TEST_SEED);
    // When the algorithm starts, run this callback to get the performance of
    // our starting point.
    tabu_search.set_callback_on_start(
        []() { std::cout << "Starting search with initial solution performance 28." << std::endl; });
    // When we find a new best solution, run this callback to get the
    // performance of the new best solution.
    tabu_search.set_callback_on_new_best([]() { std::cout << "New best solution found." << std::endl; });
    // Set the randomization threshold to 10% of the iteration count, so that the algorithm has a chance to reset if it
    // gets stuck on a local minimum.
    tabu_search.set_randomize_threshold(0.1);
    // Called when the algorithm randomizes the current search point.
    tabu_search.set_callback_on_randomize([]() { std::cout << "Randomizing search point." << std::endl; });
    // Run the tabu search algorithm for 100 iterations.
    auto solution = tabu_search.run(problem.queens(), 400);
    // Print the solution.
    std::cout << "Approximate (or exact) Solution: " << std::endl;
    tabu_examples::print(solution);

    return 0;
}
