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
template <typename T>
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
template <typename T>
concept ExplorableSpaceLazy = requires(T space) {
    {
        space.set_current_point(typename T::point_type {})
    } -> std::same_as<void>;
    {
        space.get_move()
    } -> std::same_as<typename T::point_type>;
    {
        space.eval(typename T::point_type {})
    } -> std::same_as<double>;
};

/*
 * A space which can be explored by storing all neighbors of a given point.
 * This is useful for small spaces, where storing all neighbors is feasible.
 * This is a more straightforward implementation which can be used easily.
 */
template <typename T>
concept ExplorableSpaceBasic = requires(T space) {
    {
        space.get_moves(typename T::point_type {})
    } -> std::convertible_to<std::vector<typename T::point_type>>;
    {
        space.eval(typename T::point_type {})
    } -> std::same_as<double>;
};

/*
 * Explorable Space is the minimum requirement for a class to be explored by
 * Tabu Search. For an object to be an explorable space, it needs to have a
 * point type. Furthermore, it is required to have a function which returns
 * the set of possible moves from the current point, and a function which
 * quantifies how "good" the current point is.
 */
template <typename T>
concept ExplorableSpace = SpacePoint<typename T::point_type> && (ExplorableSpaceLazy<T> || ExplorableSpaceBasic<T>);

/**
 * A hint providing space is an explorable space which can provide a hint as
 * to which direction the optimal solution is. The hint is a move which is
 * likely (but not necessary) to lead to a better solution.
 */
template <typename T>
concept HintProvidingSpace = ExplorableSpace<T> && requires(T space) {
    {
       space.get_hint(typename T::point_type {})
    } -> std::same_as<typename T::point_type>;
};

/**
 * A randomizable space is an explorable space which can provide a random point to be used as a starting point
 * or to restart the search.
 */
template <typename T>
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
constexpr double RANDOMIZE_THRESHOLD = 0.2;

/**
 * A Tabu Search algorithm.
 * @tparam Space The explorable space to search.
 */
template <ExplorableSpace Space> class TabuSearch {
public:
    // The type of the points in the explorable space.
    // This needs to be here so that the type of the hint can be seen by the
    // whole class, as well as potential concepts that use this class.
    using point_type = typename Space::point_type;

private:
    // Internal struct storing the different costs used during exploration and
    // the best point, for bookkeeping.
    struct _exploration_data_t {
        double iteration { std::numeric_limits<double>::infinity() };
        double lowest { std::numeric_limits<double>::infinity() };
        double tested { std::numeric_limits<double>::infinity() };
        point_type best_point;
        point_type iteration_point;
        uint32_t iteration_count { 0 };
        uint32_t iterations_without_improvement { 0 };
    };

    // We use a pointer to the space so that we can have lambdas in the space
    // that point to the correct data in case they capture anything by reference.
    Space& _space;
    // The tabu list, implemented as a circular buffer.
    std::deque<point_type> _tabu_list;
    // Random engine used to make the choices.
    std::mt19937 _random_engine;

    // Callbacks used at various points of the algorithm.
    // If the user wants to make them change properties or record info, they
    // can capture outside objects by reference.
    std::function<void()> _on_start { []() {} };
    std::function<void()> _on_new_best { []() {} };
    std::function<void()> _on_new_iteration { []() {} };
    std::function<void()> _on_hint_accepted { []() {} };
    std::function<void()> _on_hint_tabu { []() {} };
    std::function<void()> _on_randomize { []() {} };
    std::function<bool()> _early_stop_condition { []() { return false; } };

    // Settings for the algorithm.
    // The size of the tabu list.
    size_t _tabu_list_size{};
    // How much of the neighborhood to explore, a number which is assumed to be
    // in (0,1].
    double _neighborhood_coverage { DEFAULT_NEIGHBORHOOD_COVERAGE };
    // Percentage of steps that have to pass without improvement before the search randomizes the point, if possible.
    double _randomize_threshold { RANDOMIZE_THRESHOLD };

    // Internal function to update the tabu list with a new point.
    void _insert_tabu(point_type point)
    {
        // If the tabu list is full, remove the oldest element.
        if (_tabu_list.size() == _tabu_list_size) {
            _tabu_list.pop_front();
        }
        // Add the new element to the end of the list.
        _tabu_list.push_back(point);
    }

    // Internal function to remove all elements from the tabu list
    inline void _flush_tabu() { _tabu_list.clear(); }

    // Internal function to update the exploration data when a new point is not
    // in the tabu list
    void _accept_point_if_better(const point_type& pt, _exploration_data_t& data)
    {
        data.tested = _space.eval(pt);
        // If the cost is lower than the current iteration cost, update the
        // iteration cost and point.
        if (data.tested < data.iteration) {
            data.iteration = data.tested;
            data.iteration_point = pt;
        }
    }

    void _accept_point_unconditionally(const point_type& pt, _exploration_data_t& data)
    {
        data.tested = _space.eval(pt);
        // Update the iteration cost and point.
        data.iteration = data.tested;
        data.iteration_point = pt;
    }

    // Internal function that updates the best point
    bool _update_point_best(_exploration_data_t& data)
    {
        // If the cost is lower than the best cost, update the lowest cost and
        // the best point.
        if (data.iteration < data.lowest) {
            data.lowest = data.iteration;
            data.best_point = data.iteration_point;
            data.iterations_without_improvement = 0;
            _on_new_best();
            return true;
        }
        data.iterations_without_improvement++;
        return false;
    }

    // Internal function that updates the tabu list with the current iteration
    // point
    inline void _update_tabu(_exploration_data_t& data)
    {
        // Add the current point to the tabu list.
        _insert_tabu(data.iteration_point);
    }

    // Internal function to sample the neighborhood of a point
    std::vector<point_type> _sample_neighborhood(const point_type& pt)
        requires ExplorableSpaceBasic<Space>
    {
        // Get the set of moves from the space.
        std::vector<point_type> moves = _space.get_moves(pt);
        // Sample the moves.
        std::vector<point_type> sampled_moves;
        std::sample(moves.begin(), moves.end(), std::back_inserter(sampled_moves),
            static_cast<long int>(std::ceil(moves.size() * _neighborhood_coverage)), _random_engine);
        return sampled_moves;
    }

public:
    // Setters for the callbacks. Note that these setters copy the function.
    void set_callback_on_start(const std::function<void()>& onStart) { _on_start = onStart; }

    void set_callback_on_new_best(const std::function<void()>& onNewBest) { _on_new_best = onNewBest; }

    void set_callback_on_new_iteration(const std::function<void()>& onNewIteration)
    {
        _on_new_iteration = onNewIteration;
    }

    void set_callback_on_randomize(const std::function<void()>& onRandomize) { _on_randomize = onRandomize; }

    void set_callback_on_hint_accepted(const std::function<void()>& onHintAccepted)
    {
        _on_hint_accepted = onHintAccepted;
    }

    void set_callback_on_hint_tabu(const std::function<void()>& onHintTabu) { _on_hint_tabu = onHintTabu; }

    void set_callback_early_stopping(const std::function<bool()>& earlyStopCondition)
    {
        _early_stop_condition = earlyStopCondition;
    }

    // Getter and setter for the tabu list size.
    [[nodiscard]] size_t get_tabu_list_size() const { return _tabu_list_size; }

    void set_tabu_list_size(size_t tabuListSize) { _tabu_list_size = tabuListSize; }

    // Getter and setter for the neighborhood coverage.
    [[nodiscard]] double get_neighborhood_coverage() const { return _neighborhood_coverage; }

    void set_neighborhood_coverage(double neighborhoodCoverage) { _neighborhood_coverage = neighborhoodCoverage; }

    // Getter and setter for the randomize threshold.
    [[nodiscard]] double get_randomize_threshold() const { return _randomize_threshold; }
    void set_randomize_threshold(double randomizeThreshold) { _randomize_threshold = randomizeThreshold; }

    /**
     * Constructor for the Tabu Search algorithm.
     * @param space The explorable space to search.
     * @param tabuListSize The size of the tabu list.
     * @param neighborhoodCoverage How much of the neighborhood to explore, a
     * number which is assumed to be in (0,1].
     */
    TabuSearch(Space& space, size_t tabuListSize, double neighborhoodCoverage)
        : _space(space)
        , _tabu_list(tabuListSize)
        , _random_engine(std::random_device {}())
        , _tabu_list_size(tabuListSize)
        , _neighborhood_coverage(neighborhoodCoverage)
    {
    }

    /** Constructor for the Tabu Search algorithm.
     * @param space The explorable space to search.
     * @param neighborhoodCoverage How much of the neighborhood to explore, a
     * number which is assumed to be in (0,1].
     */
    TabuSearch(Space& space, double neighborhoodCoverage)
        : TabuSearch(space, DEFAULT_TABU_LIST_SIZE, neighborhoodCoverage)
    {
    }

    /**
     * Constructor for the Tabu Search algorithm.
     * @param space The explorable space to search.
     * @param tabuListSize The size of the tabu list.
     */
    TabuSearch(Space& space, size_t tabuListSize)
        : TabuSearch(space, tabuListSize, DEFAULT_NEIGHBORHOOD_COVERAGE)
    {
    }

    /** Constructor for the Tabu Search algorithm.
     * @param space The explorable space to search.
     */
    explicit TabuSearch(Space& space)
        : TabuSearch(space, DEFAULT_TABU_LIST_SIZE, DEFAULT_NEIGHBORHOOD_COVERAGE)
    {
    }

    /** Seed the random engine with a given value.
     * @param seed The seed to use.
     */
    inline void seed(uint32_t seed) { _random_engine.seed(seed); }

    /** Checks whether a space point is tabu or not.
     * @param point The point to check.
     */
    inline bool is_tabu(const point_type& point)
    {
        return std::find(_tabu_list.begin(), _tabu_list.end(), point) != _tabu_list.end();
    }

    /** Run the Tabu Search algorithm.
     * @param maxIterations The maximum number of iterations to run.
     * @return The best point found.
     */
    point_type run(point_type initial_condition, uint32_t max_iterations)
    {
        _on_start();
        _flush_tabu();
        _exploration_data_t data;
        auto randomize_iteration_threshold = static_cast<uint32_t>(_randomize_threshold * max_iterations);
        data.iteration_point = initial_condition;
        data.iteration = _space.eval(data.iteration_point);
        while (data.iteration_count != max_iterations && !_early_stop_condition()) {
            _on_new_iteration();
            // Compile time evaluation - if the space can provide hints, this
            // code gets compiled.
            if constexpr (HintProvidingSpace<Space>) {
                // If the space provides hints, use them with a probability.
                if (std::uniform_real_distribution<double>(0, 1)(_random_engine) < HINT_PROBABILITY) {
                    // Get the point that the hint leads us to.
                    point_type hinted_point { _space.get_hint(data.iteration_point) };
                    // If the point is not tabu, accept it without testing
                    // whether it is better.
                    if (!is_tabu(hinted_point)) {
                        _accept_point_unconditionally(hinted_point, data);
                        _on_hint_accepted();
                        _update_point_best(data);
                        _update_tabu(data);
                        // Continue to the next iteration, hint was accepted
                        // unconditionally.
                        ++data.iteration_count;
                        continue;
                    } else {
                        // If the hinted point is tabu, we continue to the normal
                        // exploration.
                        _on_hint_tabu();
                    }
                }
            }
            // If the space does not provide hints (or if the hint was rejected),
            // just use the moves as normal
            if constexpr (ExplorableSpaceLazy<Space>) {
                // In case the space is lazy, we sample the neighborhood by using the generator function.
                // This code does not use the neighborhood coverage value, since the space is lazy, and we cannot know the size of the neighborhood.
                // We first set the iteration cost to max, so that any move will be an improvement.
                data.iteration = std::numeric_limits<double>::infinity();
                // Set the current point as the root point for the neighborhood generator.
                _space.set_current_point(data.iteration_point);
                for (point_type move; move != nullptr; move = _space.get_move()) {
                    // If the point is not tabu, test whether it is better than the
                    // current point.
                    if (!is_tabu(move)) {
                        _accept_point_if_better(move, data);
                    }
                }
            } else {
                // If the space is not lazy, exploration is more straightforward.
                // We can use the neighborhood coverage value and the neighborhood size.
                for (const auto& move : _sample_neighborhood(data.iteration_point)) {
                    // If the point is not tabu, test whether it is better than the
                    // current point.
                    if (!is_tabu(move)) {
                        _accept_point_if_better(move, data);
                    }
                }
            }
            // Update the tabu list with the current iteration point
            _update_tabu(data);
            // Update the best point if necessary
            _update_point_best(data);
            // If the space supports randomization, we do it if we cannot find a better point after a certain number of iterations.
            if constexpr (RandomizableSpace<Space>) {
                if (data.iterations_without_improvement == randomize_iteration_threshold) {
                    data.iteration_point = _space.get_random_point();
                    data.iteration = _space.eval(data.iteration_point);
                    data.iterations_without_improvement = 0;
                    _on_randomize();
                }
            }
            // Increment the iteration counter.
            ++data.iteration_count;
        }
        return data.best_point;
    }
};

} // namespace tabu

#endif // TABU_SEARCH_TABUSEARCH_HPP
