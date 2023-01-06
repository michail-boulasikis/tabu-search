## Tabu Search

This is a simple C++20 implementation of a general purpose tabu search algorithm which can be used to explore any space. Feel free to use this code in your own projects.

### Getting Started

This is a header only library, so you can just include the `TabuSearch.hpp` file in your project. The class, along with constants and the supplied concepts are under the namespace `tabu`. In order to use it, the first step is to describe your space in a way that the tabu search algorithm can understand. This is done by implementing the `ExplorableSpace` concept. You must make sure of the following:
* Your space class must define a type `point_type` which is the type of a point in the space. This type must be copy and move constructible, as well as comparable with others of its type (this is needed to check whether a point is in the tabu list).
* Your space must have a member function `double eval(point_type)` which returns a cost for the supplied point. The algorithm will try to minimize this cost.
* Your space must have a member function that returns the neighborhood of a point in the space. The simplest way this can be done is with a function `std::vector<point_type> get_moves(point_type)` which returns the points that can be reached from the supplied point.

Consult the classes in the `examples` folder for examples of how to implement the `ExplorationSpace` concept.

The `TabuSearch` class can then be constructed by supplying your space as a template parameter.

### Advanced Usage and Tips

The options are:
* `tabu_list_size`, the size of the tabu list.
* `neighborhood_coverage`, a number between 0 and 1 which determines the percentage of the neighborhood that will be explored. The higher the number, the more moves will be explored, but the more time it will take.
* `randomize_threshold`, the fraction of steps that the algorithm will wait before it restarts the search from a random point. This is useful to avoid getting stuck in local minima.
* `seed`, the seed of the random number generator. If you don't provide this, the random number generator will be seeded with a random device.

If your space has a `get_hint(point_type)` function that returns a hint move, the algorithm will use it to get a hint for every step of the search. There is a fixed probability that the hint will be rejected, but if it's accepted (and the landing point is not in the tabu list), the search will continue from there. This can be used to speed up the search, but it is not required. The hint does not need to always improve the solution, but it should be a good move in most cases.

Furthermore, callbacks can be set using the appropriate setter functions. The implemented callbacks are:
* `on_start`, called when the search starts.
* `on_new_best`, called when a new best point is found.
* `on_new_iteration`, called at the beginning of each iteration.
* `early_stop_condition`, called at the beginning of each iteration. If it returns `true`, the search will stop.
* `on_hint_accepted`, called when a hint is accepted.
* `on_hint_tabu`, called when a hint is tabu.
* `on_randomize`, called when a certain number of steps have been taken without finding a better point.
* `on_select_moves`, called when the moves are selected. The moves are passed as a reference to a set of lambdas. You can modify this set to change the moves that will be explored.

You can interface with the rest of the program by capturing by reference different variables that you would like the callbacks to modify. In the future, I might add more callbacks, or change the parameters supplied to these callbacks. If no callbacks are supplied, the default functions are simply the identity function for each case.

### Suggestions

The code is well documented, so please feel free to explore it. If you have any suggestions, feel free to open an issue. If you want to contribute, feel free to open a pull request. I will try to respond to all issues and pull requests as soon as possible. For any questions or comments, you can contact me at [my email](mailto:michail.boulasikis@cs.lth.se). I hope you find this implementation useful or at least interesting!