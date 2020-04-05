#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(const RouteModel::Node *node) {
    return (*node).distance(*end_node);
}


// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    (*current_node).FindNeighbors();

    for(auto neighbor: (*current_node).neighbors)
    {
        (*neighbor).visited = true;
        (*neighbor).parent = current_node;
        (*neighbor).h_value = CalculateHValue(neighbor);
        (*neighbor).g_value = (*current_node).g_value + (*neighbor).distance(*current_node); //Distance between this node and current + current g val.

        open_list.push_back(neighbor);
    }
}


// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *nextNode;
    float lowestValue;

    std::sort(open_list.begin(), open_list.end(), [] (RouteModel::Node *startNode, RouteModel::Node *endNode) {
        float startSum = (*startNode).g_value + (*startNode).h_value;
        float endSum = (*endNode).g_value + (*endNode).h_value;

        return startSum < endSum;
    });

    auto result = open_list[0];

    return result;
}


// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    auto crawledNode = current_node;

    while(true)
    {
        path_found.push_back((*crawledNode));

        if(crawledNode != start_node)
        {
            distance += (*crawledNode).distance(*(*crawledNode).parent);

            crawledNode = (*crawledNode).parent;
        }
        else
        {
            break;
        }
    }
    
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    (*start_node).visited = true;
    open_list.push_back(start_node);

    while(open_list.size() > 0)
    {
        auto nextNode = NextNode();

        if(nextNode == end_node)
        {
            m_Model.path = ConstructFinalPath(nextNode);

            break;
        }
        else
        {
            AddNeighbors(nextNode);
        }

        open_list.erase(open_list.begin());
    }
}