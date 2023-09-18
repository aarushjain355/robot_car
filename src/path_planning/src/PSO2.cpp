#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <random>
#include <vector>
#include <random>
#include <stack>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

using namespace std;

int NUM_OF_PARTICLES = 20;
float global_best = 0;
int MINIMUM_BOUNDARY = 0;
float INERTIA_WEIGHT = 0.4;
int MAXIMUM_BOUNDARY = 10;
float SOCIAL_COMPONENT_CONSTANT = 0.5;
float COGNATIVE_COMPONENT_CONSTANT = 0.5;
int MAX_VELOCITY = 5;
int MIN_VELOCITY = -5;
int MAX_ITERATIONS = 100;
Position global_best_pos;
float cost = 0;


ros::NodeHandle nh;
ros::Publisher weights_pub = nh.advertise<std_msgs::Float32MultiArray>("weights", 10);
ros::Publisher final_weights_pub = nh.advertise<std_msgs::Float32MultiArray>("final_weights", 10);
ros::Subscriber cost_retrieval = nh.subscribe("costs", 10, cost_callback);
Particle* head;


struct Position {

    float smoothness_weight;
    float velocity_weight;
    float heading_weight;
    float clearance_weight;

};

struct Velocity {

    float smoothness_velocity;
    float velocity_velocity;
    float heading_velocity;
    float clearance_velocity;

};

struct Particle {

    Position pos;
    Velocity vel;
    Position local_best;
    float fitness;
    float value;
    Particle* right;
    Particle* left;

    Particle() {
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_real_distribution<double> floatDistribution(MINIMUM_BOUNDARY, MAXIMUM_BOUNDARY);
        std::uniform_real_distribution<int> IntDistribution(1, 100);
        std::uniform_real_distribution<double> floatDistribution2(MIN_VELOCITY/3, MAX_VELOCITY/3);
        double positionFloat1 = floatDistribution(generator);
        double positionFloat2 = floatDistribution(generator);
        double positionFloat3 = floatDistribution(generator);
        double positionFloat4 = floatDistribution(generator);
        int valueInt = IntDistribution(generator);
        double velocityFloat1 = floatDistribution2(generator);
        double velocityFloat2 = floatDistribution2(generator);
        double velocityFloat3 = floatDistribution2(generator);
        double velocityFloat4 = floatDistribution2(generator);
        pos.smoothness_weight = positionFloat1;
        pos.velocity_weight = positionFloat2;
        pos.heading_weight = positionFloat3;
        pos.clearance_weight = positionFloat4;
        vel.smoothness_velocity = velocityFloat1;
        vel.velocity_velocity = velocityFloat2;
        vel.heading_velocity = velocityFloat3;
        vel.clearance_velocity = velocityFloat4;
        fitness = 0;
        value = valueInt;
        local_best = pos;
        right = nullptr;
        left = nullptr;
    }
};

void cost_callback(const std_msgs::Float32::ConstPtr& msg) {

    cost = msg->data;
}

void insertParticle(Particle* head, Particle* insert) {

    Particle* current = head;
    Particle* parent = nullptr;

    while (current != nullptr) {

        parent = current;
        if (insert->value > current->value) {
            current = current->right;
        } else {
            current = current->left;
        }
    }

    if (parent == nullptr) {
        head = insert;

    } else if (parent->value > insert->value) {
        parent->left = insert;
    } else if (parent->value < insert->value) {
        parent->right = insert;
    }
}   

float fitness_function(Position pos) {

    std::vector<float> weights;
    
    weights.push_back(pos.velocity_weight);
    weights.push_back(pos.clearance_weight);
    weights.push_back(pos.heading_weight);
    weights.push_back(pos.smoothness_weight);
    
    weights_pub.publish(weights);
    return cost;
}

float return_final_value(float value, bool type) {

    if (type) {

        if (value > MAXIMUM_BOUNDARY) {
            value = MAXIMUM_BOUNDARY;
        } else if (value < MINIMUM_BOUNDARY) {
            value = MINIMUM_BOUNDARY;
        }
    } else {
        if (value > MAX_VELOCITY) {
            value = MAX_VELOCITY;
        } else if (value < MIN_VELOCITY) {
            value = MIN_VELOCITY;
        }
    }

    return value;
}

void evaluate_fitness(Particle* head) {

    if (head == nullptr) {
        return;
    } 

    float fitness_value = fitness_function(head->pos);
    std::random_device rd;  // Seed the random number generator
    std::mt19937 generator(rd());  // Initialize the random number generator
    std::uniform_real_distribution<double> distribution(0.0, 1.0); 

    std::stack<Particle*> stack;
    stack.push(head);

    while (!stack.empty()) {
        Particle* particle = stack.top();

        if (fitness_value < particle->fitness) {
            head->fitness = fitness_value;
            head->local_best = particle->local_best;
        } else {
            head->fitness = particle->fitness;
            head->local_best = particle->local_best;
        }

        if (head->fitness < global_best) {
        
            global_best = head->fitness;
            global_best_pos = head->local_best;
        }
        stack.pop();

        if (particle->left != nullptr) {
            stack.push(particle->left);
        } 

        if (particle->right != nullptr) {
            stack.push(particle->right);
        }
    }
    Position cognitive_difference;
    Position social_difference;
    cognitive_difference.velocity_weight = head->local_best.velocity_weight - head->pos.velocity_weight;
    cognitive_difference.smoothness_weight = head->local_best.smoothness_weight - head->pos.smoothness_weight;
    cognitive_difference.heading_weight = head->local_best.heading_weight - head->pos.heading_weight;
    cognitive_difference.clearance_weight = head->local_best.clearance_weight - head->pos.clearance_weight;
    social_difference.velocity_weight = global_best_pos.velocity_weight - head->pos.velocity_weight;
    social_difference.smoothness_weight = global_best_pos.smoothness_weight - head->pos.smoothness_weight;
    social_difference.heading_weight = global_best_pos.heading_weight - head->pos.heading_weight;
    social_difference.clearance_weight = global_best_pos.clearance_weight - head->pos.clearance_weight;

    head->vel.velocity_velocity = return_final_value(INERTIA_WEIGHT * head->vel.velocity_velocity + COGNATIVE_COMPONENT_CONSTANT * distribution(generator) * (cognitive_difference.velocity_weight) + SOCIAL_COMPONENT_CONSTANT * distribution(generator) * social_difference.velocity_weight, false);
    head->vel.clearance_velocity = return_final_value(INERTIA_WEIGHT * head->vel.clearance_velocity + COGNATIVE_COMPONENT_CONSTANT * distribution(generator) * (cognitive_difference.clearance_weight) + SOCIAL_COMPONENT_CONSTANT * distribution(generator) * social_difference.clearance_weight, false);
    head->vel.velocity_velocity = return_final_value(INERTIA_WEIGHT * head->vel.heading_velocity + COGNATIVE_COMPONENT_CONSTANT * distribution(generator) * (cognitive_difference.heading_weight) + SOCIAL_COMPONENT_CONSTANT * distribution(generator) * social_difference.heading_weight, false);
    head->vel.velocity_velocity = return_final_value(INERTIA_WEIGHT * head->vel.smoothness_velocity + COGNATIVE_COMPONENT_CONSTANT * distribution(generator) * (cognitive_difference.smoothness_weight) + SOCIAL_COMPONENT_CONSTANT * distribution(generator) * social_difference.smoothness_weight, false);

    head->pos.velocity_weight = return_final_value(head->pos.velocity_weight + head->vel.velocity_velocity, true);
    head->pos.smoothness_weight = return_final_value(head->pos.smoothness_weight + head->vel.smoothness_velocity, true);
    head->pos.clearance_weight = return_final_value(head->pos.clearance_weight + head->vel.clearance_velocity, true);
    head->pos.heading_weight = return_final_value(head->pos.heading_weight + head->vel.heading_velocity, true);

    evaluate_fitness(head->left);
    evaluate_fitness(head->right);
}



void start_PSO(const std_msgs::String::ConstPtr& msg) {

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        evaluate_fitness(head);
    }

    std::vector<float> final_weights;
    final_weights.push_back(global_best_pos.smoothness_weight);
    final_weights.push_back(global_best_pos.clearance_weight);
    final_weights.push_back(global_best_pos.heading_weight);
    final_weights.push_back(global_best_pos.velocity_weight);
    final_weights_pub.publish(final_weights)
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "Particle Swarm Optimization Node");

    for (int i = 0; i < NUM_OF_PARTICLES; i++) {

        Particle* new_particle = new Particle;
        insertParticle(head, new_particle);
        new_particle->fitness = fitness_function(new_particle->pos);
        new_particle->local_best = new_particle->pos;

        if (new_particle->fitness > global_best) {
            global_best = new_particle->fitness;
        }

    }

    ros::spin();
    return 0;
}

