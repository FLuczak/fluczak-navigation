
#include "AStar.hpp"

#include <queue>

#include "EuclideanGraph.hpp"

std::vector<std::reference_wrapper<const fluczak::Ai::Node>> fluczak::Ai::AStar::FindPath(const EuclideanGraph& graph, const Ai::Node& start, const Ai::Node& goal, float (*heuristicFunction)(const Ai::Node&, const Ai::Node&))
{
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareNodes> openSet;
    std::unordered_map<const Node*, float> scoresMap = {};
    std::unordered_map<const Node*, const Node*> predecessors = {};

    AStarNode startAStarNode = AStarNode(&start);
    startAStarNode.SetHeuristic(heuristicFunction(start, goal));

    predecessors[&start] = nullptr;
    openSet.push(startAStarNode);
    scoresMap.insert({ &start, startAStarNode.g });

    while (!openSet.empty())
    {
        // Get the node with the lowest F score from the open set
        const AStarNode current = openSet.top();
        openSet.pop();

        if (current.graphNode->position == goal.position)
        {
	        auto path = RetracePath(current, predecessors);
            return path;
        }

        auto edges = graph.GetAssociatedEdges(*current.graphNode);
        for (const Edge& edge : edges)
        {
            Node& neighbor = edge.b;
            const float potentialG = current.g + edge.GetCost();

            if (scoresMap.find(&neighbor) != scoresMap.end() && potentialG > scoresMap[&neighbor]) continue;

            AStarNode aStarNode(&neighbor);
            aStarNode.SetHeuristic(heuristicFunction(neighbor, goal));
            aStarNode.g = (potentialG);

            predecessors[aStarNode.graphNode] = current.graphNode;
            scoresMap[&neighbor] = aStarNode.g;
            openSet.push(aStarNode);
        }
    }

    return {};
}

void fluczak::Ai::AStar::DebugDrawPath(const std::vector<const Ai::Node*>& path)
{
}

void fluczak::Ai::AStar::DebugDrawPath(const std::vector<VectorMath::SimpleVector2D<float>>& path)
{
}

std::vector<std::reference_wrapper<const fluczak::Ai::Node>> fluczak::Ai::AStar::RetracePath(
	const AStarNode& end, const std::unordered_map<const Ai::Node*, const Ai::Node*>& predecessors)
{
    std::vector<std::reference_wrapper<const fluczak::Ai::Node>> path;
    const Node* currentNode = end.graphNode;

    while (currentNode != nullptr)
    {
        path.emplace_back(*currentNode);
        currentNode = predecessors.find(currentNode)->second;
    }

    std::reverse(path.begin(), path.end());
    return path;
}
