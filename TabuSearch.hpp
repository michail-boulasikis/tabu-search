//
// Created by mike on 2022-12-22.
//

#ifndef TABU_SEARCH_TABUSEARCH_HPP
#define TABU_SEARCH_TABUSEARCH_HPP

#include <concepts>
#include <deque>
#include <functional>
#include <random>

namespace tabu {

// Clang-format is still kind of bad at formatting concepts, so we have to disable it here.
// clang-format off
/*
 * Type which describes a point in space, which must overload operator==(this
 * is used to check the tabu list) and be copy and move constructible.
 */
    template<typename T>
    concept SpacePoint = requires {
        {
        std::equality_comparable<T>
        };
        {
        std::copy_constructible<T>
        };
        {
        std::move_constructible<T>
        };
    };

/*
 * A lazy exploration space can be queried for neighbors of a given point, without having to store all of them.
 * This is useful for large spaces (or spaces where the point is very large), where storing all neighbors would be infeasible.
 * In the future, co-routines could be used to make this more efficient, but I don't want to delve into that yet.
 */
    template<typename T>
    concept ExplorableSpaceLazy = requires(T space) {
        {
        space.set_current_point(typename T::point_type{})
        } -> std::same_as<void>;
        {
        space.get_move()
        } -> std::same_as<typename T::point_type>;
        {
        space.eval(typename T::point_type{})
        } -> std::same_as<double>;
    };

/*
 * A space which can be explored by storing all neighbors of a given point.
 * This is useful for small spaces, where storing all neighbors is feasible.
 * This is a more straightforward implementation which can be used easily.
 */
    template<typename T>
    concept ExplorableSpaceBasic = requires(T space) {
        {
        space.get_moves(typename T::point_type{})
        } -> std::convertible_to<std::vector<typename T::point_type>>;
        {
        space.eval(typename T::point_type{})
        } -> std::same_as<double>;
    };

/*
 * Explorable Space is the minimum requirement for a class to be explored by
 * Tabu Search. For an object to be an explorable space, it needs to have a
 * point type. Furthermore, it is required to have a function which returns
 * the set of possible moves from the current point, and a function which
 * quantifies how "good" the current point is.
 */
    template<typename T>
    concept ExplorableSpace = SpacePoint<typename T::point_type> && (ExplorableSpaceLazy<T> || ExplorableSpaceBasic<T>);

/**
 * A hint providing space is an explorable space which can provide a hint as
 * to which direction the optimal solution is. The hint is a move which is
 * likely (but not necessary) to lead to a better solution.
 */
    template<typename T>
    concept HintProvidingSpace = ExplorableSpace<T> && requires(T space) {
        {
        space.get_hint(typename T::point_type{})
        } -> std::same_as<typename T::point_type>;
    };

/**
 * A randomizable space is an explorable space which can provide a random point to be used as a starting point
 * or to restart the search.
 */
    template<typename T>
    concept RandomizableSpace = ExplorableSpace<T> && requires(T space) {
        {
        space.get_random_point()
        } -> std::same_as<typename T::point_type>;
    };
// clang-format on

// The default size for the tabu list if none is given in the constructor
    constexpr uint32_t DEFAULT_TABU_LIST_SIZE = 10;
// Default neighborhood coverage if none is given in the constructor
    constexpr double DEFAULT_NEIGHBORHOOD_COVERAGE = 0.5;
// Test seed for the random number generator, to be used for testing the
// algorithm
    constexpr uint32_t TEST_SEED = 12345;
// Probability to use a supplied hint if one is given
    constexpr double HINT_PROBABILITY = 0.5;
// Default percentage of steps that have to pass without improvement before the search randomizes the point, if possible.
    constexpr double DEFAULT_RANDOMIZE_THRESHOLD = 0.2;

/**
 * A Tabu Search algorithm.
 * @tparam Space The explorable space to search.
 */
    template<ExplorableSpace Space>
    class TabuSearch {
    public:
        // The type of the points in the explorable space.
        // This needs to be here so that the type of the hint can be seen by the
        // whole class, as well as potential concepts that use this class.
        using point_type = typename Space::point_type;

        // Settings for the algorithm.
        struct SearchOptions {
            // The size of the tabu list.
            size_t tabu_list_size{DEFAULT_TABU_LIST_SIZE};
            // How much of the neighborhood to explore, a number which is assumed to be
            // in (0,1].
            double neighborhood_coverage{DEFAULT_NEIGHBORHOOD_COVERAGE};
            // Percentage of steps that have to pass without improvement before the search randomizes the point, if possible.
            double randomize_threshold{DEFAULT_RANDOMIZE_THRESHOLD};
            // The seed for the random number generator.
            uint32_t seed{std::random_device()()};
        };

        // Exploration data type, which is used to book keep and to interface with the callbacks.
        struct ExplorationData {
            // The current iteration base point's cost.
            double iteration_cost{std::numeric_limits<double>::infinity()};
            // The lowest cost discovered so far.
            double lowest_cost{std::numeric_limits<double>::infinity()};
            // The cost of the solution that was just tested.
            double tested_cost{std::numeric_limits<double>::infinity()};
            // The best point found so far.
            point_type best_point{};
            // The base point of this iteration.
            point_type iteration_base_point{};
            // Iteration counters.
            uint32_t iteration_count{0};
            uint32_t iterations_without_improvement{0};
        };

    private:

        // We use a pointer to the space so that we can have lambdas in the space
        // that point to the correct data in case they capture anything by reference.
        Space &_space;
        // The tabu list, implemented as a circular buffer.
        std::deque<point_type> _tabu_list;
        // Random engine used to make the choices.
        std::mt19937 _random_engine;

        // The options for the search.
        SearchOptions _options;
        // The exploration data.
        ExplorationData _data;

        // Callbacks used at various points of the algorithm.
        // If the user wants to make them change properties or record info, they
        // can capture outside objects by reference.
        std::function<void(const ExplorationData &)> _on_start{[](const ExplorationData &x) {}};
        std::function<void(const ExplorationData &)> _on_new_best{[](const ExplorationData &x) {}};
        std::function<void(const ExplorationData &)> _on_new_iteration{[](const ExplorationData &x) {}};
        std::function<void(const ExplorationData &)> _on_hint_accepted{[](const ExplorationData &x) {}};
        std::function<void(const ExplorationData &)> _on_hint_tabu{[](const ExplorationData &x) {}};
        std::function<void(const ExplorationData &)> _on_randomize{[](const ExplorationData &x) {}};
        std::function<bool(const ExplorationData &)> _early_stop_condition{
                [](const ExplorationData &x) { return false; }};


        // Internal function to update the tabu list with a new point.
        void _insert_tabu(point_type point) {
            // If the tabu list is full, remove the oldest element.
            if (_tabu_list.size() == _options.tabu_list_size) {
                _tabu_list.pop_front();
            }
            // Add the new element to the end of the list.
            _tabu_list.push_back(point);
        }

        // Internal function to remove all elements from the tabu list
        inline void _flush_tabu() { _tabu_list.clear(); }

        // Internal function to update the exploration data when a new point is not
        // in the tabu list
        void _accept_point_if_better(const point_type &pt, ExplorationData &data) {
            data.tested_cost = _space.eval(pt);
            // If the cost is lower than the current iteration cost, update the
            // iteration cost and point.
            if (data.tested_cost < data.iteration_cost) {
                data.iteration_cost = data.tested_cost;
                data.iteration_base_point = pt;
            }
        }

        void _accept_point_unconditionally(const point_type &pt, ExplorationData &data) {
            data.tested_cost = _space.eval(pt);
            // Update the iteration cost and point.
            data.iteration_cost = data.tested_cost;
            data.iteration_base_point = pt;
        }

        // Internal function that updates the best point
        bool _update_point_best(ExplorationData &data) {
            // If the cost is lower than the best cost, update the lowest cost and
            // the best point.
            if (data.iteration_cost < data.lowest_cost) {
                data.lowest_cost = data.iteration_cost;
                data.best_point = data.iteration_base_point;
                data.iterations_without_improvement = 0;
                _on_new_best(data);
                return true;
            }
            data.iterations_without_improvement++;
            return false;
        }

        // Internal function that updates the tabu list with the current iteration
        // point
        inline void _update_tabu(ExplorationData &data) {
            // Add the current point to the tabu list.
            _insert_tabu(data.iteration_base_point);
        }

        // Internal function to sample the neighborhood of a point
        std::vector<point_type> _sample_neighborhood(const point_type &pt) requires ExplorableSpaceBasic<Space> {
            // Get the set of moves from the space.
            std::vector<point_type> moves = _space.get_moves(pt);
            // Sample the moves.
            std::vector<point_type> sampled_moves;
            std::sample(moves.begin(), moves.end(), std::back_inserter(sampled_moves),
                        static_cast<long int>(std::ceil(moves.size() * _options.neighborhood_coverage)),
                        _random_engine);
            return sampled_moves;
        }

    public:
        // Setters for the callbacks. Note that these setters copy the function.
        void set_callback_on_start(const std::function<void(ExplorationData)> &onStart) { _on_start = onStart; }

        void
        set_callback_on_new_best(const std::function<void(ExplorationData)> &onNewBest) { _on_new_best = onNewBest; }

        void set_callback_on_new_iteration(const std::function<void(ExplorationData)> &onNewIteration) {
            _on_new_iteration = onNewIteration;
        }

        void set_callback_on_randomize(
                const std::function<void(ExplorationData)> &onRandomize) { _on_randomize = onRandomize; }

        void set_callback_on_hint_accepted(const std::function<void(ExplorationData)> &onHintAccepted) {
            _on_hint_accepted = onHintAccepted;
        }

        void set_callback_on_hint_tabu(
                const std::function<void(ExplorationData)> &onHintTabu) { _on_hint_tabu = onHintTabu; }

        void set_callback_early_stopping(const std::function<bool(ExplorationData)> &earlyStopCondition) {
            _early_stop_condition = earlyStopCondition;
        }

        // Constructor
        explicit TabuSearch(Space &space, const SearchOptions &options = SearchOptions())
                : _space(space), _options(options), _random_engine(options.seed) {}


        /** Checks whether a space point is tabu or not.
         * @param point The point to check.
         */
        inline bool is_tabu(const point_type &point) {
            return std::find(_tabu_list.begin(), _tabu_list.end(), point) != _tabu_list.end();
        }

        /** Run the Tabu Search algorithm.
         * @param maxIterations The maximum number of iterations to run.
         * @return The best point found.
         */
        point_type run(point_type initial_condition, uint32_t max_iterations) {
            _flush_tabu();
            auto randomize_iteration_threshold = static_cast<uint32_t>(_options.randomize_threshold * max_iterations);
            _data.iteration_base_point = initial_condition;
            _data.iteration_cost = _space.eval(_data.iteration_base_point);
            _on_start(_data);
            while (_data.iteration_count != max_iterations && !_early_stop_condition(_data)) {
                _on_new_iteration(_data);
                // Compile time evaluation - if the space can provide hints, this
                // code gets compiled.
                if constexpr (HintProvidingSpace<Space>) {
                    // If the space provides hints, use them with a probability.
                    if (std::uniform_real_distribution<double>(0, 1)(_random_engine) < HINT_PROBABILITY) {
                        // Get the point that the hint leads us to.
                        point_type hinted_point{_space.get_hint(_data.iteration_base_point)};
                        // If the point is not tabu, accept it without testing
                        // whether it is better.
                        if (!is_tabu(hinted_point)) {
                            _accept_point_unconditionally(hinted_point, _data);
                            _on_hint_accepted(_data);
                            _update_point_best(_data);
                            _update_tabu(_data);
                            // Continue to the next iteration, hint was accepted
                            // unconditionally.
                            ++_data.iteration_count;
                            continue;
                        } else {
                            // If the hinted point is tabu, we continue to the normal
                            // exploration.
                            _on_hint_tabu(_data);
                        }
                    }
                }
                // We first set the iteration cost to max, so that any move will be an improvement.
                _data.iteration_cost = std::numeric_limits<double>::infinity();
                // If the space does not provide hints (or if the hint was rejected),
                // just use the moves as normal
                if constexpr (ExplorableSpaceLazy<Space>) {
                    // In case the space is lazy, we sample the neighborhood by using the generator function.
                    // This code does not use the neighborhood coverage value, since the space is lazy, and we cannot know the size of the neighborhood.
                    // Set the current point as the root point for the neighborhood generator.
                    _space.set_current_point(_data.iteration_base_point);
                    for (point_type move; move != nullptr; move = _space.get_move()) {
                        // If the point is not tabu, test whether it is better than the
                        // current point.
                        if (!is_tabu(move)) {
                            _accept_point_if_better(move, _data);
                        }
                    }
                } else {
                    // If the space is not lazy, exploration is more straightforward.
                    // We can use the neighborhood coverage value and the neighborhood size.
                    for (const auto &move: _sample_neighborhood(_data.iteration_base_point)) {
                        // If the point is not tabu, test whether it is better than the
                        // current point.
                        if (!is_tabu(move)) {
                            _accept_point_if_better(move, _data);
                        }
                    }
                }
                // Update the tabu list with the current iteration point
                _update_tabu(_data);
                // Update the best point if necessary
                _update_point_best(_data);
                // If the space supports randomization, we do it if we cannot find a better point after a certain number of iterations.
                if constexpr (RandomizableSpace<Space>) {
                    if (_data.iterations_without_improvement == randomize_iteration_threshold) {
                        _data.iteration_base_point = _space.get_random_point();
                        _data.iteration_cost = _space.eval(_data.iteration_base_point);
                        _data.iterations_without_improvement = 0;
                        _on_randomize(_data);
                    }
                }
                // Increment the iteration counter.
                ++_data.iteration_count;
            }
            return _data.best_point;
        }
    };

} // namespace tabu

#endif // TABU_SEARCH_TABUSEARCH_HPP
