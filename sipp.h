#ifndef SIPP_H
#define SIPP_H
#include "structs.h"
#include "map.h"
#include "heuristic.h"
#include <unordered_map>
#include <map>
#include <set>
class SIPP
{
public:

    SIPP()  {}
    ~SIPP() {}
    Path find_path(Agent const& agent, const Map &map, std::list<Constraint> const& cons, Heuristic &h_values);

private:
    Agent agent;
    std::vector<Path> find_partial_path(std::vector<Node> const& starts, std::vector<Node> const& goals, const Map &map, Heuristic &h_values, double max_f = CN_INFINITY);
    void append(Path& result, Path const& part);
    void find_successors(Node const& curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values, Node const& goal);
    void add_open(Node const& newNode);
    Node find_min();
    double dist(const Node &a, const Node &b);
    std::vector<Node> const& reconstruct_path(Node curNode);
    void make_constraints(std::list<Constraint> const& cons);
    void clear();
    void add_collision_interval(int id, std::pair<double, double> const& interval);
    void add_move_constraint(Move const& move);
    //std::vector<Node> get_endpoints(int node_id, double node_i, double node_j, double t1, double t2);
    void get_endpoints(int node_id, double node_i, double node_j, double t1, double t2, std::vector<Node>& nodes, bool first=false, bool last=false);
    double check_endpoint(Node start, Node goal);

    std::unordered_map<int, Node> close;
    std::list<Node> open;
    std::unordered_map<int, std::pair<double, bool>> visited;
    std::map<std::pair<int, int>, std::vector<Move>> constraints;//stores sets of constraints associated with moves
    std::unordered_map<int, std::vector<std::pair<double, double>>> collision_intervals;//stores sets of collision intervals associated with cells
    std::vector<Move> landmarks;
    Path path;
};

#endif // SIPP_H
