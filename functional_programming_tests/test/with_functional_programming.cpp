#include <gtest/gtest.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <optional>

// production_code.h/cc
// X, Y position
struct Position {
    size_t x;
    size_t y;
};

template<typename T>
class Map
{
public:
    Map(std::vector<std::vector<T>> data): data_{data} {};
    T at(Position const& pos) const {
        return data_.at(pos.y).at(pos.x);
    }
private:
    std::vector<std::vector<T>> data_;
};

using Path = std::vector<Position>;

// Operator overload to print path
std::ostream& operator<<(std::ostream& os, std::vector<Position> const& path) {
    for (auto const& pose:path) {
        os << "(" << pose.x << ", " << pose.y << ")\n";
    }
    return os;
}

// Operator overload for position comparison
bool operator==(Position const& lhs, Position const& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

// From the Robot, goal and costmap, generate a trajectory (Deterministic calculation)
std::optional<Path> generate_global_path(Position const& start, Position const& goal , Map<unsigned char> const& costmap) { // Calculation
    // Some cool and nifty algorithm
    // What is the delta in position
    int const del_x = goal.x - start.x;
    int const del_y = goal.y - start.y;

    // What direction to move in for each dimension
    int const del_x_sign = std::copysign(1.0, del_x);
    int const del_y_sign = std::copysign(1.0, del_y);

    // Push start onto the path
    Path path;
    path.push_back(start);

    auto is_occupied = [&costmap](auto const x, auto const y) -> bool {
        return costmap.at(Position{x, y}) == 1;
    };

    // Fails if there is any obstacle in the way
    // Move horizontally
    for (size_t i = 0; i < (std::abs(del_x)); ++i) {
        if (is_occupied(path.back().x + del_x_sign, path.back().y)) {        
            return std::nullopt;
        }        
        path.push_back({path.back().x + del_x_sign, path.back().y});
    }
    // Move vertically
    for (size_t i = 0; i < (std::abs(del_y)); i++) {
        // if (costmap.at(Position{path.back().x, path.back().y + del_y_sign}) == 1) {
        if (is_occupied(path.back().x, path.back().y + del_y_sign)) {            
            return std::nullopt;
        }            
        path.push_back({path.back().x, path.back().y + del_y_sign});
    }

    return path;
}

// test.cc

/**
 * @brief      Gets the test costmap.
 *
 * @return     The test costmap.
 */
Map<unsigned char> get_test_costmap() {
    return {{{0, 0, 0, 0, 0, 0, 0, 0},
             {0, 0, 0, 1, 0, 0, 0, 0}, 
             {0, 0, 0, 1, 0, 0, 0, 0}, 
             {0, 0, 1, 1, 1, 0, 0, 0}, 
             {0, 0, 1, 0, 1, 1, 0, 0}, 
             {0, 0, 1, 0, 0, 0, 0, 0}, 
             {0, 0, 0, 0, 0, 0, 0, 0}, 
             {0, 0, 0, 0, 0, 0, 0, 0}}};
}

TEST(generate_path, same_start_and_goal) {

    // GIVEN a costmap and the same start and goal
    auto const sample_costmap = get_test_costmap();

    Position const start {0, 0};
    Position const goal {0, 0};

    // WHEN the global path is produced
    auto const& path = generate_global_path(start, goal, sample_costmap);    

    // THEN the path should have one element, which is the start/goal position
    std::vector<Position> expected {{0, 0}};
    EXPECT_EQ(path.value(), expected) << path.value();
}

TEST(generate_path, no_path) {

    // GIVEN a costmap AND a start and goal position
    auto const sample_costmap = get_test_costmap();

    Position const start {2, 2};
    Position const goal {5, 5};

    // WHEN the global path is produced
    auto const& path = generate_global_path(start, goal, sample_costmap);    

    // THEN the path should not have been generated
    EXPECT_FALSE(path.has_value()) << path.value();
}

TEST(generate_path, path_generated) {

    // GIVEN a costmap AND a start and goal position
    auto const sample_costmap = get_test_costmap();

    Position const start {0, 0};
    Position const goal {7, 7};

    // WHEN the global path is produced
    auto const& path = generate_global_path(start, goal, sample_costmap);    

    // THEN the path should have a valid path from start to the goal
    std::vector<Position> expected {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {7, 1}, {7, 2}, {7, 3}, {7, 4}, {7, 5}, {7, 6}, {7, 7}};
    EXPECT_EQ(path.value(), expected) << path.value();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
