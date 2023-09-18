#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <queue>
#include <limits>
using namespace std;


void global_path_planning_callback(std::vector<std::vector<int>> adjlist, std::vector<std::vector<float>> weights) {

    int numNodes = adjlist.size();
    std::vector<float> distance(numNodes, std::numeric_limits<float>::max());
    std::vector<int> previous(numNodes, -1);
    std::vector<bool> visited(numNodes, false);
    int startNode = adjlist[0][0];
    int endNode = adjlist[-1][0];
    distance[startNode] = 0;

    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> pq;
    pq.push(std::make_pair(0, startNode));

    while (!pq.empty()) {
        int currentNode = pq.top().second;
        pq.pop();

        if (visited[currentNode]) {
            continue;
        }

        visited[currentNode] = true;

        for (int i = 0; i < adjlist[currentNode].size(); i++) {
            int neighbor = adjlist[currentNode][i];
            float weight = weights[currentNode][i];

            if (!visited[neighbor] && distance[currentNode] + weight < distance[neighbor]) {
                distance[neighbor] = distance[currentNode] + weight;
                previous[neighbor] = currentNode;
                pq.push(std::make_pair(distance[neighbor], neighbor));
            }
        }
    }

    std::vector<int> path;
    int currentNode = endNode;
    while (currentNode != -1) {
        path.push_back(currentNode);
        currentNode = previous[currentNode];
    }

    std::reverse(path.begin(), path.end());

    ros::NodeHandle nh;
    ros::ServiceClient path_client = nh.ServiceClient<Path_Planning::adjlist>("final_path_service");
    path_planning::adjlist::Request req;

    req.input_vector = path;
    
    if (path_client.call(req)) {

        Path_Planning::adjlist::Response res = req.response;

    } else {

        ROS_ERROR("Failed");
    }
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "Global_Path_Planning_Node");
    ros::NodeHandle nh;

    ros::ServiceServer adjlist_weights_server = nh.advertiseService("Adjlist Service", global_path_planning_callback);
    
    ros::spin();

    return 0;

}


