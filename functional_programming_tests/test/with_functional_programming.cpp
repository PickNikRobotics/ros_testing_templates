#include <gtest/gtest.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <optional>

// production_code.h/cc
template<typename T>
using Map = std::vector<std::vector<T>>;

// X, Y position
struct Pos {
    size_t x;
    size_t y;
};

using Path = std::vector<Pos>;

// Operator overload to print path
std::ostream& operator<<(std::ostream& os, std::vector<Pos> const& path) {
    for (auto const& pose:path) {
        os << "(" << pose.x << ", " << pose.y << ")\n";
    }
    return os;
}

// Operator overload for position comparison
bool operator==(Pos const& lhs, Pos const& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

// From the Robot, goal and costmap, generate a trajectory (Deterministic calculation)
std::optional<Path> generate_global_path(Pos const& start, Pos const& goal , Map<unsigned char> const& costmap) { // Calculation
    // Some cool and nifty algorithm
    // What is the delta in position
    int const del_x = goal.x - start.x;
    int const del_y = goal.y - start.y;

    // What direction to move in for each dimension
    int const del_x_sign = std::copysign(1.0,del_x);
    int const del_y_sign = std::copysign(1.0,del_y);

    // Push start onto the path
    Path path;
    path.push_back(start);

    // Fails if there is any obstacle in the way
    // Move horizontally
    for (size_t i = 0; i < (std::abs(del_x)); ++i) {
        if (costmap.at(path.back().y).at(path.back().x + del_x_sign) == 1) {
            return std::nullopt;
        }        
        path.push_back({path.back().x+del_x_sign, path.back().y});
    }
    // Move vertically
    for (size_t i = 0; i < (std::abs(del_y)); i++) {
        if (costmap.at(path.back().y + del_y_sign).at(path.back().x) == 1) {
            return std::nullopt;
        }            
        path.push_back({path.back().x, path.back().y+del_y_sign});
    }

    return path;
}

// test.cc
TEST(generate_path, same_start_and_goal) {

    // GIVEN a costmap and the same start and goal
    Map<unsigned char> sample_costmap = {{0, 0, 0, 0, 0, 0, 0, 0},
                               {0, 0, 0, 1, 0, 0, 0, 0}, 
                               {0, 0, 0, 1, 0, 0, 0, 0}, 
                               {0, 0, 1, 1, 1, 0, 0, 0}, 
                               {0, 0, 1, 0, 1, 1, 0, 0}, 
                               {0, 0, 1, 0, 0, 0, 0, 0}, 
                               {0, 0, 0, 0, 0, 0, 0, 0}, 
                               {0, 0, 0, 0, 0, 0, 0, 0}};        

    Pos const start {0, 0};
    Pos const goal {0, 0};

    // WHEN the global path is produced
    auto const& path = generate_global_path(start, goal, sample_costmap);    

    // THEN the path should have one element, which is the start/goal position
    std::vector<Pos> expected {{0, 0}};
    EXPECT_EQ(path.value(), expected) << path.value();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}