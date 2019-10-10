/*
* Name: Jack Nichols
* ---------------------------------------
* File: Trailblazer.cpp
* ---------------------------------------
* This file implements several graph algorithms,
* including including breadth-first search,
* Dijkstra’s algorithm, and A* search, in order
* to build a navigation program.
*/

#include "Trailblazer.h"
#include "queue.h"
#include "priorityqueue.h"
using namespace std;

// CONSTANTS
static const double SUFFICIENT_DIFFERENCE = 0.2;

//METHODS
void considerNeighbors(Path &currentPath, Queue<Path> &paths, Set<RoadNode*> &visited, Set<RoadNode*> &neighbors);
Path findDijkstrasPath(const RoadGraph& graph, RoadNode* start, RoadNode* end, RoadEdge *excludedEdge, string algorithm);
void considerPQNeighbors(RoadEdge *excludedEdge, RoadNode* vertex, const RoadGraph& graph, Path &currentPath, PriorityQueue<Path> &paths,
                        Set<RoadNode*> &visited, Set<RoadNode*> &neighbors, double &priority, string type, RoadNode* end);
double getDijkstraPriority(RoadNode* vertex, RoadNode* neighbor, double &prevPriority, const RoadGraph& graph);
double getTravelHeuristic(RoadNode* vertex, RoadNode* neighbor, const RoadGraph& graph);
Path getShortestAlternate(const RoadGraph& graph, Vector<Path> &alternativePaths, Path shortest);
double getDifferenceScore(Path path, Path shortest);
double getCost(Path path, const RoadGraph& graph);
double getAStarPriority(Path &path, RoadNode* neighbor, const RoadGraph& graph, RoadNode* end);

/*
* METHOD: breadthFirstSearch
* ----------------------------------------
* This method implements a breadth first search
* in order to find the shortest path between two vertices.
* This method finds a path by starting at a specified vertex
* and evaluating all the neighbors of that point untill a path
* from the starting point to the ending point is successfully
* created.  The completed shortest path between the two points
* is then returned.
*/
Path breadthFirstSearch(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
   Set<RoadNode*> visited; Queue<Path> paths; Path currentPath;
   currentPath.add(start);
   paths.enqueue(currentPath);
   // loop through all potential paths...
   while(!paths.isEmpty() && !visited.contains(end)) {
       currentPath = paths.dequeue();
       RoadNode* vertex = currentPath.get(currentPath.size() - 1);
       vertex->setColor(Color::GREEN);
       if (!visited.contains(vertex)) {
           visited.add(vertex);
           if (vertex == end) {
               return currentPath;
           }
           Set<RoadNode*> neighbors = graph.neighborsOf(vertex);
           // now... we need to add all potential neighbors to a new Path and enqueue that path...
           considerNeighbors(currentPath, paths, visited, neighbors);
       }
   }
   return {};  // Returns an empty path if no path can be found
}

/*
* METHOD: considerNeighbors
* ----------------------------------------
* This method iterates through all of the neighboring
* verticies of a given vertex and creates a new path
* ending with each neighbor.  Each new path is then
* added to a queue containing all possible incomplete
* and complete paths.
*/
void considerNeighbors(Path &currentPath, Queue<Path> &paths, Set<RoadNode*> &visited, Set<RoadNode*> &neighbors) {
   for (RoadNode* neighbor : neighbors) {
       Path newPath = currentPath;
       newPath.add(neighbor);
       paths.enqueue(newPath);
       if (!visited.contains(neighbor)) {
           neighbor->setColor(Color::YELLOW);
       }
   }
}

/*
* METHOD: dijkstrasAlgorithm
* ----------------------------------------
* This method calls upon Dijkstras' Algorithm to find the lowest-cost
* route between two nodes, without excluding any edges.
*/
Path dijkstrasAlgorithm(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
   // pass in excludedEdge as a nullptr... since we are not running AltRoute...
   return (findDijkstrasPath(graph, start, end, nullptr, "D"));
}

/*
* METHOD: aStar
* ----------------------------------------
* This method implements a search by the A* algorithm
* in order to find the most cost efficient path between two vertices.
* This method finds a path by starting at a specified vertex
* and evaluating all the neighbors of that point.  The most cost efficient
* path becomes the path with the most urgent priority in a priority
* queue of paths, and that path is returned.  The priority of a given
* path is determined by considering a heuristic value.
*/
Path aStar(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
   return (findDijkstrasPath(graph, start, end, nullptr, "A"));
}

/*
* METHOD: findDijkstrasPath
* ----------------------------------------
* This method is called by AltRoute and DijkstrasAlgorithm. Essentially,
* it implements a search by dijkstra's algorithm in order to find
* the most cost efficient path between two vertices. This method
* finds a path by starting at a specified vertex and evaluating all the
* neighbors of that point.  The most cost efficient path becomes the
* path with the most urgent priority in a priority queue of paths, and
* that path is returned.
*/
Path findDijkstrasPath(const RoadGraph& graph, RoadNode* start, RoadNode* end, RoadEdge *excludedEdge, string algorithm) {
   PriorityQueue<Path> paths;
   Set<RoadNode*> visited;
   Path currentPath;
   currentPath.add(start);
   // if Dijkstra's... first priority is always 0...
   if (algorithm == "D") paths.enqueue(currentPath, 0);
   // if Astar... first priority should be the travel heuristic...
   if (algorithm == "A") paths.enqueue(currentPath, getTravelHeuristic(start, end, graph));
   while(!paths.isEmpty() && !visited.contains(end)){
       double priority = paths.peekPriority();
       currentPath = paths.dequeue();
       RoadNode* vertex = currentPath.get(currentPath.size() - 1);
       vertex->setColor(Color::GREEN);
       if (!visited.contains(vertex)) {
           visited.add(vertex);
           if (vertex == end) {
               return currentPath;
           }
           Set<RoadNode*> neighbors = graph.neighborsOf(vertex);
           // again... we must differentiate between Dijkstra's and A*...
           if (algorithm == "D") {
               considerPQNeighbors(excludedEdge, vertex, graph, currentPath, paths, visited, neighbors, priority, "D", end);
           } else {
               considerPQNeighbors(excludedEdge, vertex, graph, currentPath, paths, visited, neighbors, priority, "A", end);
           }
       }
   }
   return {};
}

/*
* METHOD: considerPQNeighbors
* ----------------------------------------
* This method iterates through all of the neighboring
* verticies of a given vertex and creates a new path
* ending with each neighbor.  Each new path is then
* added to a priority queue containing all possible incomplete
* and complete paths.
*/
void considerPQNeighbors(RoadEdge *excludedEdge, RoadNode* vertex, const RoadGraph& graph, Path &currentPath, PriorityQueue<Path> &paths,
                        Set<RoadNode*> &visited, Set<RoadNode*> &neighbors, double &priority, string type, RoadNode* end) {
   // if we are not using AltRoute... then excludedEdge will be a nullptr..
   if (excludedEdge != nullptr) {
       if (vertex == excludedEdge->from()) {
           // if AltRoute, then we want to remove an edge by removing a neighbor that we can visit...
           neighbors.remove(excludedEdge->to());
       }
   }
   for (RoadNode* neighbor : neighbors) {
       Path newPath = currentPath;
       newPath.add(neighbor);
       double newPriority;
       // priorities for A* and Dijkstra are different (especially since we do not want the travel heuristic to compound)...
       if (type == "A") {
           newPriority = getAStarPriority(newPath, neighbor, graph, end);
       } else {
           newPriority = getDijkstraPriority(vertex, neighbor, priority, graph);
       }
       // enqueue with newPriority and continue algorithm...
       paths.enqueue(newPath, newPriority);
       if (!visited.contains(neighbor)) {
           neighbor->setColor(Color::YELLOW);
       }
   }
}

/*
* METHOD: getAStarPriority
* ----------------------------------------
* This method returns the priority for the AStar algorithm which includes the cost of the path
* and the the travelHeuristic between the neighbor and the final node.
*/
double getAStarPriority(Path &path, RoadNode* neighbor, const RoadGraph& graph, RoadNode* end) {
   return getCost(path, graph) + getTravelHeuristic(neighbor, end, graph);
}

/*
* METHOD: getDijkstraPriority
* ----------------------------------------
* This method returns the priority for the Dijkstra algorithm which is simply the cost of
* the path. This value can be found by adding the cost of the new edge to the previous priority.
*/
double getDijkstraPriority(RoadNode* vertex, RoadNode* neighbor, double &prevPriority, const RoadGraph& graph) {
   RoadEdge* edgeBetween = graph.edgeBetween(vertex, neighbor);
   double cost = edgeBetween->cost();
   return prevPriority + cost;
}

/*
* METHOD: getTravelHeuristic
* ----------------------------------------
* This method calculates the heuristic value
* of travel between two points by dividing the
* distance between the points by the time it
* takes to travel along the given path.  The
* heuristic value is then returned.
*/
double getTravelHeuristic(RoadNode* neighbor, RoadNode* end, const RoadGraph& graph) {
   double distance = graph.crowFlyDistanceBetween(neighbor, end);
   double speed = graph.maxRoadSpeed();
   return distance / speed;
}

/*
* METHOD: alternativeRoute
* ----------------------------------------
* This method uses an exludedEdge and the AStar algorithm to an alternative
* path to that which Dijkstra or AStar would return. It first creates a vector
* of excludedEdges and then finds an alternative Path for each excluded edge.
*/
Path alternativeRoute(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
   // 1. find the shortest path...
   Path shortest = aStar(graph, start, end);
   Vector<RoadEdge*> excludedEdges;
   // 2. find all edges of the shortest path...
   for (int i = 0; i <= shortest.size() - 2; i++) {
       RoadEdge* edge = graph.edgeBetween(shortest[i], shortest[i + 1]);
       excludedEdges.add(edge);
   }
   Vector<Path> alternativePaths;
   // 3. get a path for every excluded Edge...
   for (RoadEdge *edge : excludedEdges) {
       alternativePaths.add(findDijkstrasPath(graph, start, end, edge, "A"));
   }
   // 4. return the lowest cost path that meets the difference threshold...
   return (getShortestAlternate(graph, alternativePaths, shortest));
}

/*
* METHOD: getShortestAlternate
* ----------------------------------------
* This method takes a vector of alternative paths and returns
* the one with the smallest cost that meets the difference requirement
*/
Path getShortestAlternate(const RoadGraph& graph, Vector<Path> &alternativePaths, Path shortest) {
   // if there are no alternative paths (i.e. the path is just to itself)...
   if (alternativePaths.isEmpty()) return {};
   // 1. start with bestPath as first path in the vector (this path may not actually meet the difference threshold)
   Path bestPath = alternativePaths[0];
   int index = 0;
   // 2. loop through all paths until one that meets the difference threshold is found (could just be the first one)...
   while (index < alternativePaths.size()) {
       bestPath = alternativePaths[index];
       if (getDifferenceScore(bestPath, shortest) > SUFFICIENT_DIFFERENCE) {
           break;
       }
       index++;

   }

   // if no path has a difference score of greater than 0.2 ... return an empty path
   if (index == alternativePaths.size()) return {};
   // 3. loop through the rest of the paths and see if any has a smaller cost...
   for (int i = index; i < alternativePaths.size(); i++) {
       Path path = alternativePaths[i];
       if (getDifferenceScore(path, shortest) > SUFFICIENT_DIFFERENCE && getCost(path, graph) < getCost(bestPath, graph)) {
           bestPath = path;
       }
   }
   return bestPath;
}

/*
* METHOD: getDifferenceScore
* ----------------------------------------
* The difference score is just the number of dissimilar nodes (in path but
* not in shortest path) divided by the total number of nodes in the shortest.
*/
double getDifferenceScore(Path path, Path shortest) {
   double totalNodes = shortest.size();
   double diffNodes = 0;
   for (RoadNode* node : path) {
       if (!shortest.contains(node)) {
           diffNodes++;
       }
   }
   return (diffNodes / totalNodes);
}

/*
* METHOD: getCost
* ----------------------------------------
* This method returns the cost of a path. First,
* we create a vector of edges and then sum
* the cost of each edge in the path.
*/
double getCost(Path path, const RoadGraph& graph) {
   double cost = 0;
   Vector<RoadEdge*> edges;
   // create a vector of all the edges in the path
   for (int i = 0; i <= path.size() - 2; i++) {
       RoadEdge* edge = graph.edgeBetween(path[i], path[i + 1]);
       edges.add(edge);
   }
   // adds the cost of that edge to the total cost
   for (RoadEdge * edge : edges) {
       cost += edge->cost();
   }
   return cost;
}