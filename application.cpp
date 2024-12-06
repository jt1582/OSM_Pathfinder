#include "application.h"$

#include <iostream>
#include <limits>
#include <map>
#include <queue>  // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h" 
using namespace std;
#include "json.hpp"
using json = nlohmann::json;


double INF = numeric_limits<double>::max();

void buildGraph(istream& input, graph<long long, double>& g,
                vector<BuildingInfo>& buildings) {
    json data;
    input >> data;
    for (const auto& building : data["buildings"]) {
        long long id = building["id"];
        double lat = building["lat"];
        double lon = building["lon"];
        string name = building["name"];
        string abbr = building["abbr"];

        BuildingInfo info(id, {lat, lon}, name, abbr);
        buildings.push_back(info);
        g.addVertex(id);
    }
    unordered_map<long long, Coordinates> waypointCoords;
    for (const auto& waypoint : data["waypoints"]) {
        long long id = waypoint["id"];
        double lat = waypoint["lat"];
        double lon = waypoint["lon"];
        waypointCoords[id] = {lat, lon};
        g.addVertex(id);
    }

    for (const auto& footway : data["footways"]) {
        for (size_t i = 1; i < footway.size(); ++i) {
            long long from = footway[i - 1];
            long long to = footway[i];
            double distance = distBetween2Points(waypointCoords[from], waypointCoords[to]);
            g.addEdge(from, to, distance);
            g.addEdge(to, from, distance);
        }
    }

    for (const auto& building : buildings) {
        for (const auto& [id, coords] : waypointCoords) {
            double distance = distBetween2Points(building.location, coords);
            if (distance <= 0.036) {
                g.addEdge(building.id, id, distance);
                g.addEdge(id, building.id, distance);
            }
        }
    }
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo>& buildings,
                             const string& query) {
  for (const BuildingInfo& building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo>& buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo& building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

vector<long long> dijkstra(const graph<long long, double>& G, long long start,
                           long long target,
                           const set<long long>& ignoreNodes) {
    class prioritize {
    public:
        bool operator()(const pair<long long, double>& p1,
                        const pair<long long, double>& p2) const {
            return p1.second > p2.second;
        }
    };
    if (start == target) {
        return {start};
    }

    unordered_map<long long, double> distances;
    unordered_map<long long, long long> parent;
    set<long long> visited;
    for (long long v : G.getVertices()) {
        distances[v] = numeric_limits<double>::max();
    }
    distances[start] = 0;

    priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> pq;
    pq.push({start, 0});

    while (!pq.empty()) {
        long long current = pq.top().first;
        double currentDistance = pq.top().second;
        pq.pop();

        if (visited.count(current)) {
            continue;
        }
        visited.insert(current);

        if (current == target) {
            break;
        }

        for (long long neighbor : G.neighbors(current)) {
            if (ignoreNodes.count(neighbor) && neighbor != start && neighbor != target) {
                continue;
            }
            double weight;
            if (G.getWeight(current, neighbor, weight)) {
                double newDistance = currentDistance + weight;
                if (newDistance < distances[neighbor]) {
                    distances[neighbor] = newDistance;
                    parent[neighbor] = current;
                    pq.push({neighbor, newDistance});
                }
            }
        }
    }

    vector<long long> path;
    if (distances[target] == numeric_limits<double>::max()) {
        return {};
    }

    for (long long at = target; at != start; at = parent[at]) {
        path.push_back(at);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return path;
}

double pathLength(const graph<long long, double>& G,
                  const vector<long long>& path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long>& path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

void application(const vector<BuildingInfo>& buildings,
                 const graph<long long, double>& G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto& building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
