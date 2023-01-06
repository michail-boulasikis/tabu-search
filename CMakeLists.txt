cmake_minimum_required(VERSION 3.24)
project(tabu_search)

set(CMAKE_CXX_STANDARD 20)

add_executable(tabu_search main.cpp TabuSearch.hpp examples/EightQueens.cpp examples/EightQueens.hpp)
