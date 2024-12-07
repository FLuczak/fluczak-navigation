#include "NavMesh.hpp"

#include <array>

#include "AStar.hpp"
#include "CDT/CDT.h"
#include "MathUtilities.hpp"
#include "CDT/CDTUtils.h"
#include "clipper2/clipper.h"


fluczak::Ai::NavMesh::NavMesh(float agentRadiusToSet) : agentRadius(agentRadiusToSet)
{

}

/**
 * \brief A main constructor for the navmesh. It takes in the floors and the obstacles vertices and bakes the navmesh
 * \param agentRadiusToSet - radius of the navmeshagent
 * \param floor - floor polygons
 * \param obstacles - obstacle polygons
 */
fluczak::Ai::NavMesh::NavMesh(float agentRadiusToSet, const std::vector<fluczak::VectorMath::SimplePolygon2D>& floor, const std::vector<fluczak::VectorMath::SimplePolygon2D>& obstacles)
{
    for (auto& path : floor)
    {
        Clipper2Lib::PathD floorPath;
        for (const auto point : path)
        {
            floorPath.push_back(Clipper2Lib::Point<double>(point.x, point.y));
        }
        floorPaths.push_back(floorPath);
    }

    for (auto& path : obstacles)
    {
        Clipper2Lib::PathD obstaclePath;
        for (const auto point : path)
        {
            obstaclePath.push_back(Clipper2Lib::Point<double>(point.x, point.y));
        }
        obstaclesPath.push_back(obstaclePath);
    }

    agentRadius = agentRadiusToSet;
    Bake();
}


/**
 * \brief A function that turns all holes in the floor into obstacles
 * \param levelPaths - A floor path with obstacles
 */
void fluczak::Ai::NavMesh::RemoveHolesFromPaths(Clipper2Lib::PathsD& levelPaths)
{
    if (levelPaths.size() <= 1) return;

    std::vector<int> toRemove;

    for (int i = 0; i < levelPaths.size(); i++)
    {
        for (int j = 0; j < levelPaths.size(); j++)
        {
            if (std::find(toRemove.begin(), toRemove.end(), i) != toRemove.end()) continue;
            if (i == j) continue;
            Clipper2Lib::PathsD clips;
            Clipper2Lib::PathsD subject;

            subject.push_back(levelPaths[i]);
            clips.push_back(levelPaths[j]);

            if (!Clipper2Lib::Intersect(subject, clips, Clipper2Lib::FillRule::NonZero).empty())
            {
                toRemove.push_back(j);
                obstaclesPath.push_back(levelPaths[j]);
            }
        }
    }

    int removed = 0;
    for (const int indexToRemove : toRemove)
    {
        levelPaths.erase(levelPaths.begin() + indexToRemove - removed);
        removed++;
    }
}

/**
 * \brief Given a level path- take previously generated floor paths and add them to the level
 * \param levelPaths - A reference to the level paths
 */
void fluczak::Ai::NavMesh::BakeFloorIntoSummarizedPath(Clipper2Lib::PathsD& levelPaths)
{
    levelPaths = Clipper2Lib::Union(levelPaths, floorPaths, Clipper2Lib::FillRule::NonZero);
    RemoveHolesFromPaths(levelPaths);
}

/**
 * \brief - Given a level path- take previously generated floor paths and add them to the level
 * \param levelPaths - A reference to the level paths
 */
void fluczak::Ai::NavMesh::BakeObstaclesIntoSummarizedPath(std::vector<Clipper2Lib::PathsD>& levelPaths, float offset) const
{
    auto scaledDuplicates = Clipper2Lib::Union(obstaclesPath, obstaclesPath, Clipper2Lib::FillRule::NonZero);

    scaledDuplicates = InflatePaths(scaledDuplicates, offset, Clipper2Lib::JoinType::Square, Clipper2Lib::EndType::Polygon);

    for (Clipper2Lib::PathsD& path : levelPaths)
    {
        path = Clipper2Lib::Difference(path, scaledDuplicates, Clipper2Lib::FillRule::Positive);
        Clipper2Lib::StripDuplicates(path, true);
        HandleSeparatedFloorsByObstacles(path, levelPaths);
    }
}

/**
 * \brief Function that separates floors that were divided by obstacles, creating new, separate floors with
 * their own obstacles.
 * \param path - A floor path with obstacles
 * \param levelPaths - Collective set of floors with obstacles
 */
void fluczak::Ai::NavMesh::HandleSeparatedFloorsByObstacles(Clipper2Lib::PathsD& path, std::vector<Clipper2Lib::PathsD>& levelPaths) const
{
    std::vector<int> toRemove;
    std::vector<Clipper2Lib::Path<double>> toAdd;

    for (int j = 1; j < path.size(); j++)
    {
        if (std::find(toRemove.begin(), toRemove.end(), j) != toRemove.end()) continue;
        Clipper2Lib::PathsD clips;
        Clipper2Lib::PathsD subject;

        subject.push_back(path[0]);
        clips.push_back(path[j]);

        if (Clipper2Lib::Intersect(subject, clips, Clipper2Lib::FillRule::NonZero).empty())
        {
            toRemove.push_back(j);
            toAdd.push_back({ path[j] });
        }
    }

    if (toAdd.empty()) return;
    int removed = 0;
    for (const int removeIndex : toRemove)
    {
        if (path.empty()) break;
        path.erase(path.begin() + removeIndex - removed);
        removed++;
    }

    const size_t addedIndex = levelPaths.size();
    levelPaths.emplace_back();
    for (Clipper2Lib::Path<double> separatedPath : toAdd)
    {
        levelPaths[addedIndex].push_back(separatedPath);
    }
}
/**
 * \brief A method that given a path on the navmesh graph finds overlapping triangle edges (portals)
 * and returns an array of portal structures with left and right points for them. Used for the funnel algorithm
 * \param path - A navmeshpath generated by findpath
 * \return - A vector of portals that were intersected along the path
 */
std::vector<fluczak::Ai::NavMesh::Portal> fluczak::Ai::NavMesh::FindPortalsAlongPath(NavMeshPath& path) const
{
    std::vector<fluczak::VectorMath::SimpleVector2D<float>> points = path.GetPoints();
    const std::vector<Node>& graphNodes = graph.GetNodes();

    if (points.empty()) return {};
    std::vector<Ai::NavMesh::Portal> toReturn;
    toReturn.emplace_back(points[0], points[0]);
    for (size_t j = 0; j < points.size() - 1; j++)
    {
        for (const Triangle& triangle : triangles)
        {
            for (int i = 0; i <= 2; i++)
            {
                if (graphNodes[triangle.nodeIndex].position != points[j]) continue;
                if (!fluczak::VectorMath::Utils::DoSegmentsIntersect(triangle.vertices[i], triangle.vertices[(i + 1) % triangle.vertices.size()], points[j], points[j + 1]))continue;

                if (IsLeft(points[j], points[j + 1], triangle.vertices[(i + 1) % triangle.vertices.size()]))
                {
                    toReturn.emplace_back(triangle.vertices[(i)], triangle.vertices[(i + 1) % triangle.vertices.size()]);
                }
                else
                {
                    toReturn.emplace_back(triangle.vertices[(i + 1) % triangle.vertices.size()], triangle.vertices[i]);
                }
            }
        }
    }
    toReturn.emplace_back(points.back(), points.back());
    return toReturn;
}

/**
 * \brief A method that given 2 points of a line segment and a third arbitrary point determines
 * if the point lies on the left of the given line segment
 * \param startPoint - start point of the line segment
 * \param endPoint - end point of the line segment
 * \param point - an arbitrary point
 * \return - A boolean whether the point is on the left of the line segment
 */
bool fluczak::Ai::NavMesh::IsLeft(fluczak::VectorMath::SimpleVector2D<float> startPoint, fluczak::VectorMath::SimpleVector2D<float> endPoint, fluczak::VectorMath::SimpleVector2D<float> point)
{
    return fluczak::VectorMath::Utils::IsPointLeftOfLine(point, startPoint, endPoint);
}

/**
 * \brief Initialize and triangulate the navigation mesh, create a graph.
 */
void fluczak::Ai::NavMesh::Bake()
{
    if (floorPaths.empty())
    {
        return;
    }

    isBaked = true;
    Clipper2Lib::PathsD tempLevel;
    std::vector<Clipper2Lib::PathsD> levelPaths;

    BakeFloorIntoSummarizedPath(tempLevel);

    for (std::vector<Clipper2Lib::Point<double>>& path : tempLevel)
    {
        levelPaths.emplace_back();
        levelPaths[levelPaths.size() - 1].push_back(path);
    }

    BakeObstaclesIntoSummarizedPath(levelPaths, agentRadius);
    TriangulateMesh(levelPaths);
    InitializeGraph();
}

/**
 \brief A method that while provided a start node from the graph and a end node from the graph returns
 * a path from start to goal node in navmesh graph. IMPORTANT: The navmesh needs to be baked before calling
 * this method
 * \param start - A Start node from the graph
 * \param goal - An end node from the graph
 * \return - A NavMeshPath from one node to another
 */
fluczak::Ai::NavMeshPath fluczak::Ai::NavMesh::FindPath(const Ai::Node& start, const Ai::Node& goal) const
{
    if (!isBaked)
    {
        return {};
    }

	auto path = AStar::FindPath(graph,start, goal, &AStar::DistanceHeuristic);
    return NavMeshPath(path);
}

/**
 * \brief A method that while provided a start world position from the graph and an end wolrd position return
 * a path of nodes from start node (closest to start position) to end node (closest to goal position)
 * returns an empty path if a path cannot be traced or any of the points are outside of the navmesh.
 * IMPORTANT: The navmesh needs to be baked to before calling this method.
 * \param startPos - a fluczak::VectorMath::SimpleVector2D<float> of a start position of the path
 *  \param endPos - a fluczak::VectorMath::SimpleVector2D<float> of an end position of the path
 * \return - A NavMeshPath from startPos to endPos
 */
fluczak::Ai::NavMeshPath fluczak::Ai::NavMesh::FindPath(fluczak::VectorMath::SimpleVector2D<float> startPos, fluczak::VectorMath::SimpleVector2D<float> endPos) const
{
    if (!isBaked)
    {
        return {};
    }

    const NavMeshSample sampleOne = SamplePosition(startPos);
    const NavMeshSample sampleTwo = SamplePosition(endPos);

    if (!sampleOne.DidHit() || !sampleTwo.DidHit()) return {};

    NavMeshPath path = FindPath(*sampleOne.GetHitNode(), *sampleTwo.GetHitNode());

    //set points in the path to end and start positions instead of startnode and endnode position
    if (!path.GetPoints().empty())
    {
        if (path.GetPoints().size() == 1)
        {
            path.GetPoints().push_back({ endPos });
        }

        path.GetPoints().back() = endPos;
        path.GetPoints()[0] = startPos;
    }

    return path;
}


/**
 * \brief Implementation of funnel algorithm - takes in a normal, a* created path and returns an array
 * of vectors from the shortened path. This is loosely based on this implementation in javascript:
 * https://pastebin.com/7jwrmw1i
 * \param path - Path of Ai nodes returned by (for example) FindPath
 * \return - a vector of fluczak::VectorMath::SimpleVector2D<float>s after applying the funnel algorithm
 */
std::vector<fluczak::VectorMath::SimpleVector2D<float>> fluczak::Ai::NavMesh::FunnelPath(NavMeshPath path) const
{
    if (path.GetPoints().empty()) return {};
    std::vector<Portal> portals = FindPortalsAlongPath(path);
    if (portals.empty()) return { path.GetPoints().back() };

    std::vector<fluczak::VectorMath::SimpleVector2D<float>> smoothedPath;
    fluczak::VectorMath::SimpleVector2D<float> portalApex, portalLeft, portalRight;

    portalApex = portals[0].leftPoint;
    portalLeft = portals[0].leftPoint;
    portalRight = portals[0].rightPoint;
    int apexIndex = 0, leftIndex = 0, rightIndex = 0;

    smoothedPath.push_back(portalApex);

    for (size_t i = 1; i < portals.size(); i++)
    {
        const fluczak::VectorMath::SimpleVector2D<float> left = portals[i].leftPoint;
        const fluczak::VectorMath::SimpleVector2D<float> right = portals[i].rightPoint;

        if (SignedArea(portalApex, portalRight, right) <= 0.0f)
        {
            if (portalApex == portalRight || SignedArea(portalApex, portalLeft, right) > 0.0f)
            {
                portalRight = right;
                rightIndex = static_cast<int>(i);
            }
            else
            {
                smoothedPath.push_back(portalLeft);
                portalApex = portalLeft;
                apexIndex = leftIndex;
                portalLeft = portalApex;
                portalRight = portalApex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
                i = apexIndex;
                continue;
            }
        }

        if (SignedArea(portalApex, portalLeft, left) >= 0.0f)
        {
            if (portalApex == portalLeft || SignedArea(portalApex, portalRight, left) < 0.0f)
            {
                portalLeft = left;
                leftIndex = static_cast<int>(i);
            }
            else
            {
                smoothedPath.push_back(portalRight);
                portalApex = portalRight;
                apexIndex = rightIndex;
                portalLeft = portalApex;
                portalRight = portalApex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
                i = apexIndex;
            }
        }
    }

    if (smoothedPath.empty() || smoothedPath.back() != portals.back().leftPoint)
    {
        smoothedPath.push_back(portals.back().leftPoint);
    }

    return smoothedPath;
}

/**
 * \brief Given an arbitrary point in the world, return a point that is closest to the given point
 * on the nav mesh
 * \param point - an arbitrary point in the game world
 * \return - A point closest to 'point' on the nav mesh
 */
fluczak::VectorMath::SimpleVector2D<float> fluczak::Ai::NavMesh::FindClosestPointOnNavMesh(fluczak::VectorMath::SimpleVector2D<float> point) const
{
    fluczak::VectorMath::SimpleVector2D<float> closestPoint = {};
    float closestDistance = std::numeric_limits<float>::max();
    for (const auto& vertex : graph.GetNodes())
    {
        const float distance = VectorMath::SimpleVector2D<float>::Length(point - vertex.position);

        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestPoint = vertex.position;
        }
    }

    return closestPoint;
}

/**
 * \brief - Draw a debug form of the nav mesh
 * \param debugRenderer - Engine's debug renderer
 */
void fluczak::Ai::NavMesh::DebugDraw() const
{

}

/**
 * \brief A method that takes in a position and returns a NavMeshSample
 * \param position - A position being sampled on the navmesh
 * \return a NavMeshSample - An object with two getters- DidHit()- a boolean value if the sample
 * successfully found a node on the navmesh and GetHitNode()- which returns the closest graph node
 * to the given position
 */
fluczak::Ai::NavMeshSample fluczak::Ai::NavMesh::SamplePosition(const fluczak::VectorMath::SimpleVector2D<float>& position) const
{
    for (const Triangle& triangle : triangles)
    {
        const std::array<fluczak::VectorMath::SimpleVector2D<float>, 3>& vertices = triangle.vertices;
        if (!PointInTriangle(position, vertices[0], vertices[1], vertices[2])) continue;

        return NavMeshSample{ true, &graph.GetNodes()[triangle.nodeIndex] };
    }

    return NavMeshSample{ false, nullptr };
}

/**
 * \brief a function that calculates a signed area of the given triangle- used for checking
 * if a given point lies within a triangle
 * \param p1 - position of vertex one of the triangle
 * \param p2 - position of vertex two of the triangle
 * \param p3 -position of vertex three of the triangle
 * \return - A signed area of a triangle
 */
float fluczak::Ai::NavMesh::SignedArea(const fluczak::VectorMath::SimpleVector2D<float>& p1, const fluczak::VectorMath::SimpleVector2D<float>& p2, const fluczak::VectorMath::SimpleVector2D<float>& p3)
{
    const float ax = p2.x - p1.x;
    const float ay = p2.y - p1.y;
    const float bx = p3.x - p1.x;
    const float by = p3.y - p1.y;
    return bx * ay - ax * by;
}

/**
 * \brief - a function that checks if a point lies within a given triangle
 * \param pt - a point being checked
 * \param v1 - vertex one of the triangle
 * \param v2 - vertex two of the triangle
 * \param v3 -vertex three of the triangle
 * \return - a result of the check if the point p lies in the given triangle
 */
bool fluczak::Ai::NavMesh::PointInTriangle(const fluczak::VectorMath::SimpleVector2D<float>& pt, const fluczak::VectorMath::SimpleVector2D<float>& v1, const fluczak::VectorMath::SimpleVector2D<float>& v2, const fluczak::VectorMath::SimpleVector2D<float>& v3)
{
    return VectorMath::Utils::IsPointInsidePolygon(pt, { v1, v2, v3 });
}

/**
 * \brief - Add vertex information into the triangulation object. It is the first step of triangulation from the cdt
 * official documentation.
 * \param cdt - Triangulation object reference
 * \param levelPath -A temporary path of summarized level floor with obstacles
 */
void fluczak::Ai::NavMesh::AddTriangulationVertices(CDT::Triangulation<double>& cdt, const Clipper2Lib::PathsD& levelPath)
{
    for (const std::vector<Clipper2Lib::Point<double>>& mesh : levelPath)
    {
        const size_t meshSize = mesh.size();
        std::vector<CDT::V2d<double>> vertices;
        for (size_t i = 0; i < meshSize; i++)
        {
            vertices.push_back({ mesh[i].x, mesh[i].y });
        }

        cdt.insertVertices(vertices);
    }
}

/**
 * \brief - Add edge information for obstacles and floors into the triangulation object.
 * It is the second required step of triangulation from the cdt official documentation
 * \param cdt - Triangulation object reference
 * \param levelPath - A temporary path of summarized level floor with obstacles
 */
void fluczak::Ai::NavMesh::AddTriangulationEdges(CDT::Triangulation<double>& cdt, const Clipper2Lib::PathsD& levelPath)
{
    size_t currentObstacleIndex = 0;
    // provide edges of the walkable areas for hole detection from triangulation
    std::vector<CDT::Edge> edges{};
    const size_t pathLength = levelPath[0].size();

    for (size_t floorVertexIndex = 0; floorVertexIndex < pathLength; floorVertexIndex++)
    {
        edges.emplace_back(floorVertexIndex % pathLength + currentObstacleIndex, (floorVertexIndex + 1) % pathLength + currentObstacleIndex);
    }

    currentObstacleIndex += levelPath[0].size();
    cdt.insertEdges(edges);

    // provide edges of obstacles for hole detection from triangulation
    for (size_t meshIndex = 1; meshIndex < levelPath.size(); meshIndex++)
    {
        std::vector<CDT::Edge> edges{};
        size_t pathLength = levelPath[meshIndex].size();
        for (size_t obstacleVertexIndex = 0; obstacleVertexIndex < pathLength; obstacleVertexIndex++)
        {
            edges.emplace_back(obstacleVertexIndex % pathLength + currentObstacleIndex, (obstacleVertexIndex + 1) % pathLength + currentObstacleIndex);
        }
        currentObstacleIndex += levelPath[meshIndex].size();
        cdt.insertEdges(edges);
    }
}

/**
 * \brief -Initialize the navigation mesh Ai::graph using data stored in the cdt object
 * it will be later used for the navigation. Fill the triangle vector with newly created
 * navmesh triangles.
 * \param cdt - Triangulation object provided
 */
void fluczak::Ai::NavMesh::InitializeTriangles(const CDT::Triangulation<double>& cdt)
{
    const int offset = static_cast<int>(triangles.size());
    for (CDT::Triangle triangle : cdt.triangles)
    {
        Triangle toAdd(triangle);

        for (int i = 0; i < 3; i++)
        {
            const CDT::V2d<double> point = cdt.vertices[triangle.vertices[i]];
            toAdd.vertices[i] = VectorFromCDT(point);

            if (triangle.neighbors[i] >= static_cast<unsigned>(cdt.triangles.size()) || triangle.neighbors[i] < 0) continue;
            toAdd.cdtTriangle.neighbors[i] += offset;
        }

        triangles.push_back(toAdd);
    }
}

/**
 * \brief Initialize underlying navmesh graph from previously created triangles
 */
void fluczak::Ai::NavMesh::InitializeGraph()
{
    std::vector<EdgeData> edgeData;
    std::vector<Ai::Node> nodes;

    int index = 0;
    for (Triangle& triangle : triangles)
    {
        for (int i = 0; i < 3; i++)
        {
            if (triangle.cdtTriangle.neighbors[i] >= static_cast<int>(triangles.size())) continue;
            edgeData.emplace_back(index, triangle.cdtTriangle.neighbors[i], 1.0f);
        }

        triangle.nodeIndex = index;
        nodes.emplace_back(GetTriangleCentroid(triangle));
        index++;
    }

    graph.AddNodesAndEdges(nodes, edgeData);
    graph.InitializeEuclideanWeights();
}

/**
 * \brief - A method that initializes the trianglesPath used for debugging from the triangles
 * vector member
 */
void fluczak::Ai::NavMesh::CreateTrianglesMesh()
{
    int index = 0;
    for (const Triangle& triangle : triangles)
    {
        trianglesPath.push_back({});

        for (int i = 0; i < 3; i++)
        {
            trianglesPath[index].emplace_back(triangle.vertices[i].x, triangle.vertices[i].y);
        }

        trianglesPath[index].emplace_back(triangle.vertices[0].x, triangle.vertices[0].y);
        index++;
    }
}

/**
 * \brief A collective triangulation method - it takes a vector of level areas with baked floors and
 * obstacles and generates a triangulated meshes that are later used for creating a graph
 * \param paths - All separate level paths
 */
void fluczak::Ai::NavMesh::TriangulateMesh(std::vector<Clipper2Lib::PathsD>& paths)
{
    for (Clipper2Lib::PathsD& path : paths)
    {
        CDT::Triangulation<double> cdt(CDT::VertexInsertionOrder::AsProvided, CDT::IntersectingConstraintEdges::Ignore, 0.0);

        AddTriangulationVertices(cdt, path);
        AddTriangulationEdges(cdt, path);

        cdt.eraseOuterTrianglesAndHoles();
        CDT::extractEdgesFromTriangles(cdt.triangles);

        InitializeTriangles(cdt);
        CreateTrianglesMesh();
    }
}

/**
 * \brief - Given a triangle class return a centroid of the triangle
 * \param triangle - a reference to a triangle instance
 * \return - a fluczak::VectorMath::SimpleVector2D<float> of a triangle's centroid position
 */
fluczak::VectorMath::SimpleVector2D<float> fluczak::Ai::NavMesh::GetTriangleCentroid(const Triangle& triangle)
{
    float oX = (triangle.vertices[0].x + triangle.vertices[1].x + triangle.vertices[2].x) / 3.0f;
    float oY = (triangle.vertices[0].y + triangle.vertices[1].y + triangle.vertices[2].y) / 3.0f;
    return { oX, oY };
}

/**
 * \brief A helper function for converting V2d objects to fluczak::VectorMath::SimpleVector2D<float>s - it is used
 * for creating nodes for the graph from triangles generated by CDT
 * \param cdtVec - a V2d object
 * \return - a fluczak::VectorMath::SimpleVector2D<float> converted from cdtVec
 */
fluczak::VectorMath::SimpleVector2D<float> fluczak::Ai::NavMesh::VectorFromCDT(const CDT::V2d<double>& cdtVec) { return { static_cast<float>(cdtVec.x),static_cast<float>(cdtVec.y) }; }