#include <iostream>

#include "TabuSearch.hpp"
#include "examples/EightQueens.hpp"

int main() {
    // Initialize the problem
    tabu_examples::EightQueens problem{};

    // Create the options for the tabu search
    tabu::TabuSearch<tabu_examples::EightQueens>::SearchOptions options{};
    // The size of the tabu list, i.e. the number of moves that are not allowed
    options.tabu_list_size = 2000;
    // The percentage of steps that need to run without an improvement before the search is randomly restarted.
    options.randomize_threshold = 0.1;
    // The percentage of the neighborhood that is explored in each step.
    options.neighborhood_coverage = 0.15;
    // The seed of the random number generator.
    options.seed = tabu::TEST_SEED;

    // Create the tabu algorithm configuration, with the problem, the tabu list
    // size, and the neighborhood exploration percentage.
    tabu::TabuSearch<tabu_examples::EightQueens> tabu_search{problem, options};
    // When the algorithm starts, run this callback.
    tabu_search.set_callback_on_start(
            [](const auto &x) { std::cout << "Starting search with cost = " << x.lowest_cost << std::endl; });
    // When we find a new best solution, run this callback to get the performance of the new best solution.
    tabu_search.set_callback_on_new_best(
            [](const auto &x) {
                std::cout << "New best solution found with cost = " << x.lowest_cost << " on iteration "
                          << x.iteration_count << std::endl;
            });
    // Called when the algorithm randomizes the current search point, so that we know it happened.
    tabu_search.set_callback_on_randomize([](const auto &x) {
        std::cout << "Randomizing search point on iteration " << x.iteration_count << std::endl;
    });
    // Set an early stopping callback which stops the search when the cost becomes 0 (i.e. we found a solution)
    tabu_search.set_callback_early_stopping([](const auto &x) {
        if (x.lowest_cost == 0.0) {
            std::cout << "Solution found early! Stopping." << std::endl;
            return true;
        }
        return false;
    });
    // Run the tabu search algorithm for 400 iterations. Use an all queens on the first row initial condition.
    auto solution = tabu_search.run({1, 2, 3, 4, 5, 6, 7, 8}, 400);
    // Print the solution.
    std::cout << "Solution: " << std::endl;
    tabu_examples::print(solution);

    return 0;
}
