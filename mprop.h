#ifndef MProp_H
#define MProp_H
#include "structs.h"
#include "map.h"
#include "heuristic.h"
#include <unordered_map>
#include <map>
#include <set>
#include <array>

class MProp
{
public:

    MProp()  {}
    ~MProp() {}
    void mutex_propagation(Agent const& agent1, Agent const& agent2, Map const& map, std::list<Constraint> const& cons, Heuristic &h_values);
    void mutex_propagation(Agent const& agent1, Agent const& agent2, Map const& map, std::list<Constraint> const& cons1, std::list<Constraint> const& cons2, Heuristic &h_values);

private:
    void joint_product(const std::array<std::vector<Node>, 2> &v, NodePair const* parent, Map const& map, std::vector<NodePair> &s);
    void joint_successors(NodePair const& v, Map const& map, std::vector<NodePair> &s, Heuristic const& h_values, NodePair const& goal);
    void find_successors(Node const& curNode, const Map &map, std::vector<Node> &succs, Heuristic const& h_values, Node const& goal, int agent_index) const;
    std::vector<Path> find_partial_path(std::vector<NodePair> const& starts, std::vector<NodePair> const& goals, const Map &map, Heuristic &h_values, double max_f = CN_INFINITY);
    void append(Path& result, Path const& part);
    void add_open(NodePair const& newNode);
    NodePair find_min();
    double dist(const Node &a, const Node &b) const;
    std::vector<Node> reconstruct_path(Node curNode);
    void make_constraints(std::list<Constraint> &cons, int agent_index);
    void clear();
    void add_collision_interval(int id, std::pair<double, double> interval, int agent_index);
    void add_move_constraint(Move const& move, int agent_index);
    std::vector<Node> get_endpoints(int node_id, double node_i, double node_j, double t1, double t2);
    double check_endpoint(Node start, Node goal);

    std::array<Agent,2> agent;
    std::unordered_map<uint64_t, NodePair> close;
    std::list<NodePair> open;
    std::unordered_map<uint64_t, std::pair<double, bool>> visited;
    std::array<std::map<std::pair<int, int>, std::vector<Move>>,2> constraints;//stores sets of constraints associated with moves
    std::array<std::unordered_map<int, std::vector<std::pair<double, double>>>,2> collision_intervals;//stores sets of collision intervals associated with cells
    std::array<std::vector<Move>,2> landmarks;
    Path path;
};

#endif // MProp_H
