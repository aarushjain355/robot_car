#include <iostream>
#include <stack>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <vector>
#include "path_planning/map.h"
using namespace std;

ros::NodeHandle nh;
ros::Publisher pub = nh.advertise<std_msgs::Float32>("cost", 10);
ros::Publisher pub2 = nh.advertise<std_msgs::Float32MultiArray>("velocity_commands", 10);
Config cfg;
Config new_cfg;
Position test_pos;
Velocity test_vel;
Point test_goal;
std::vector<float> weights_to_optimize;

amcl_data = None
gmapping_data = None

struct Config {

    float max_speed = 10;
    float min_speed = 10;
    float max_angle_rate = 10;
    float max_vel_accel = 10;
    float max_accel_yaw = 10;
    float vel_res = 10;
    float yawrate_res = 10;
    float dt = 10;
    float prediction_time = 10;
    float radius = 10;
    float heading = 10;
    float clearance = 10;
    float velocity = 10;
    float smoothness = 10;
};

struct Position {

    float x;
    float y;
    float yaw_angle;
    float lin_vel;
    float angle_vel;
};

struct Velocity {

    float linear_velocity;
    float angular_velocity;
};

struct DynamicWindow {

    std::vector<float> possibleVelocities;
    std::vector<float> possibleAngularVel;
    int possible_velocities;
    int possible_angularvelocities;
};

struct Point {

    float x;
    float y;
};

std::vector<int> index;
std::vector<geometry_msgs::Point> obstacle_coordinates;
Point starting_position;

std::vector<geometry_msgs::Point> mapCallback(path_planning::map::Request& req, path_planning::map::Response& res)
{
    int map_width = req->info.width;
    int map_height = req->info.height;
    const std::vector<int8_t>& map_data = req->data;
    double map_resolution = req->info.resolution;
    int thresold = req->info.thresold;
    std::vector<std::vector<int>> adjacencyList;
    std::vector<std::vector<float>> weights;
    int count = 0;

    for (int y = 0; y < map_height; ++y)
    {
        for (int x = 0; x < map_width; ++x)
        {
            int map_index = y * map_width + x;
            int8_t occupancy = map_data[map_index];

            if (occupancy > thresold)
            {
                float obstacle_x = x * map_resolution + req->info.origin.position.x;
                float obstacle_y = y * map_resolution + req->info.origin.position.y;

                // Create a geometry_msgs/Point message to store coordinates
                geometry_msgs::Point obstacle_point;
                obstacle_point.x = obstacle_x;
                obstacle_point.y = obstacle_y;
                obstacle_point.z = 0.0;  // Assuming 2D map
                std::vector<int> current_list;
                std::vector<float> current_weights;

                obstacle_coordinates.push_back(obstacle_point);
                index.push_back(map_index);
                current_list.push_back(map_index);
                current_weights.push_back(0);
                for (int row_range = -2; row_range <= 2; row_range++) {
                    for (int col_range = -2; col_range <= 2; col_range++) {
                        int row_pos = y + row_range;
                        int col_pos = x + col_range;

                        if (row_pos >= 0 && row_pos <= map_height && col_pos >= 0 && col_pos <= map_width) {

                            int index2 = row_pos * map_width + col_pos;
                            int8_t occupancy2 = map_data[index2];

                            if (occupancy2 > thresold) {
                                double weight_x = col_range * map_resolution + obstacle_coordinates.back().x;
                                double weight_y = row_range * map_resolution + obstacle_coordinates.back().y;
                                double distance = std::hypot(weight_x, weight_y);
                                current_list.push_back(index2);
                                current_weights.push_back(distance);
                            }
                        }

                    }
                }

                adjacencyList.push_back(current_list);
                weights.push_back(current_weights);
            }
        }
    }

    res.ok = 1;
    return obstacle_coordinates;

    
}


DynamicWindow createDynamicWindow(const Position& pos, const Config& cfg) {

    float min_vel_speed = std::max(cfg.min_speed, pos.lin_vel - cfg.max_vel_accel * dt);
    float max_vel_speed = std::min(cfg.max_speed, pos.lin_vel + cfg.max_vel_accel * dt);
    float min_angular_speed = std::max(-cfg.max_angle_rate, pos.angle_vel - cfg.max_accel_yaw * dt);
    float max_angular_speed = std::min(cfg.max_angle_rate, pos.angle_vel + cfg.max_accel_yaw * dt);

    DynamicWindow dw;

    dw.possible_velocities = (max_vel_speed - min_vel_speed) / cfg.vel_res;
    dw.possible_angularvelocities = (max_angular_seed - min_angular_seed) / cfg.yawrate_res;

    for (int i = min_vel_speed; i <= max_vel_speed; i+=cfg.vel_res) {
        dw.possibleVelocities.push_back(i);
    }

    for (int i = min_angular_sped; i <= max_angular_speed; i+=cfg.yawrate_res) {
        dw.possibleAngularVel.push_back(i);
    }

    return dw;
}



Position motion(const Position& pos, const Velocity& vel, const Config& cfg) {

    Position new_pos;
    new_pos.yaw_rate = pos.yaw_rate + vel.angular_velocity * cfg.dt;
    new_pos.x = pos.x + vel.linear_velocity * std::cos(new_pose.yaw_rate) * cfg.dt;
    new_pos.y = pos.y + vel.linear_velocity * std::sin(new_pose.yaw_rate) * cfg.dt;
    new_pose.lin_vel = vel.linear_velocity;
    new_pose.angle_vel = vel.angular_velocity;
    return new_pose;

}

float velocityCost(const Velocity& vel, const Config& cfg) {

    float cost = cfg.max_speed - vel.linear_velocity;
    return cost;
}

float headingCost(const Position& pos, const Velocity& vel, const Point& goal) {

    float dx = goal.x - pos.x;
    float dy = goal.y - pos.y;
    float angleError = std::atan2(dx, dy);
    float angleCost = angleError - pos.yaw_angle;
    float cost = std::abs(std::atan2( std::sin(angleCost), std::cos(angleCost)));
    return cost; 

}

void particle_swarm_optimization_test(const std_msgs::Float32MultiArray::ConstPtr& msg) {

    new_cfg.velocity = msg->data[0];
    new_cfg.clearance = msg->data[1];
    new_cfg.heading = msg->data[2];
    new_cfg.smoothness = msg->data[3];
    test_pos.x = 12.3;
    test_pos.y = 11.2;
    test_pos.yaw_angle = 30;
    test_pos.lin_vel = 5;
    test_pos.angle_vel = 30;
    test_goal.x = 30.3;
    test_goal.y = 32.1;
    test_vel.linear_velocity = 10;
    test_vel.angular_velocity 40;

    float velocity_cost = velocityCost(test_vel, new_cfg);
    float heading_cost = headingCost(test_pos, test_vel, test_goal);
    float smoothness_cost = smoothnessCost(test_pos, test_vel);
    float clearance_cost = clearanceCost(test_pos, test_vel, new_cfg, test_goal, obstacle_coordinates);
    float cost = velocity_cost * new_cfg.velocity + heading_cost * new_cfg.heading + smoothness_cost * new_cfg.smoothness + clearance_cost * new_cfg.clearance;
    pub.publish(cost);
}

float smoothnessCost(const Position& pos, const Velocity& vel) {

    float linear_diff = vel.linear_velocity - pos.lin_vel;
    float angular_diff = vel.angular_velocity - pos.angle_vel;
    float cost = std::abs(linear_dif) + std::abs(angular_diff);
    return cost;
}

float clearanceCost(const Position& pos, const Velocity& vel, const Config& cfg, const Point& goal, std::vector<geometry_msgs::Point> obstacle_coordinates) {

    Position new_position = pos;
    float min_radius = std::numeric_limits<float>::max();
    float time = 0.0;
    float current_radius = 0.0;
    float dx;
    float dy;
    float x;
    float y;

    while (time < cfg.prediction_time) {

        new_position = motion(new_position, vel, cfg);
        for (int i = 0; i < obstacle_coordinates.size(); i++) {
            
            if (obstacle_coordinates[i].x != goal.x || obstacle_coordinates[i].y != goal.y) {
                dx = obstacle_coordinates[i].x - new_position.x;
                dy = obstacle_coordinates[i].y - new_position.y;
                x = -1*(dx) * std::cos(new_position.yaw_angle) + -1*(dy) * std::sin(new_position.yaw_angle);
                y = dx * std::sin(new_position.yaw_angle) + -1*(dy) * std::cos(new_position.yaw_angle);
                current_radius = std::hypot(x, y);

                if (current_radius < min_radius) {
                    min_radius = current_radius;
                }

                if (min_radius < cfg.radius) {
                    return std::numeric_limits<float>::infinity();
                }
            }
        }
        time += cfg.dt;
    }
    return 1.0/min_radius;
}

Position evaluateTrajectory(const Position& pos, const Point& goal, const Config& cfg, std::vector<geometry_msgs::Point> obstacle_coordinates) {

    float min_cost = std::numeric_limits<float>::max();
    std::vector<float> velocities;
    DynamicWindow dw = createDynamicWindow(pos, cfg);
    Velocity current_vel;
    Velocity best_vel;
    Position pos2;

    for (int i = 0; i < dw.possibleVelocities.size(); i++) {

        current_vel.linear_velocity = dw.possibleVelocities[i];
        for (int x = 0; x < dw.possibleAngularVel.size(); x++) {

            current_vel.angular_velocity = dw.possibleAngularVel[x];
            float vel_cost = velocityCost(current_vel, cfg);
            float heading_cost = headingCost(pos, current_vel, goal);
            float smoothness_cost = smoothnessCost(pos, current_vel);
            float clearance_cost = clearanceCost(pos, current_vel, cfg, goal, obstacle_coordinates);
            float current_cost  = cfg.velocity * vel_cost + cfg.heading * heading_cost + cfg.smoothness * smoothness_cost + cfg.clearance * clearance_cost;

            if (current_cost < min_cost) {
                min_cost = current_cost;
                best_vel.linear_velocity = current_vel.linear_velocity;
                best_vel.angular_velocity = current_vel.angular_velocity;
            }    

        }
    }

    pos2 = motion(pos, best_vel, cfg);

    velocities[0] = best_vel.linear_velocity;
    velocities[1] = best_vel.angular_velocity;
    pub2.publish(velocities);
    return pos2
    
    // will publish the velocity commands to the arduino node so add this in future

}

void dwa_control(int map_index2, std::vector<geometry_msgs::Point> obstacles, geometry_msgs::Point goal, geometry_msgs::Point current_pos) {


    Point current_pos;
    if (map_index2 == -1) {
        current_pos.x = 0;
        current_pos.y = 0;
    } else {

        current_pos = obstacles[map_index2];
    }

    while (current_pos != goal) {

        weights_to_optimize.push_back(cfg.velocity);
        weights_to_optimize.push_back(cfg.smoothness);
        weights_to_optimize.push_back(cfg.clearance);
        weights_to_optimize.push_back(cfg.heading);
        current_pos = evaluateTrajectory(current_pos, goal, cfg, obstacles);

    }   

}

void receive_path_callback(path_planning::adjlist::Request& req, path_planning::adjlist::Response& res) {

    std::vector<int> path = req.input_vector;
    int map_index2;
    float prev_position = 0;
    for (int i = 0; i < path.size(); i++) {
        if (i == 0) {
            map_index2 = -1;
        } 
        for (int x = 0; x < index.size(); x++) {
            if (index[x] == path[i]) {
                dwa_control(map_index2, obstacle_coordinates, obstacle_coordinates[x], starting_position);
                starting_position = obstacle_coordinates[x];
                map_index2 = x;
                break;
            }
        }
    }


}

void final_weights_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

    cfg.smoothness = msg->data[0];
    cfg.clearance = msg->data[1];
    cfg.heading = msg->data[2];
    cfg.velocity = msg->data[3];
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "Dynamic Window Approach baby");

    ros::ServiceServer server = nh.advertiseService("map", mapCallback);
    ros::Subscriber sub = nh.subscribe("weights", 10, particle_swarm_optmization_test);
    ros::Subscriber sub = nh.subscribe("final_weights", 10, final_weights_callback);
  

    ros.spin();
    return 0;
}