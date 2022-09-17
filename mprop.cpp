#include "mprop.h"
#include "FPUtil.h"

void MProp::joint_product(const std::array<std::vector<Node>, 2> &v, NodePair const* parent, Map const& map, std::vector<NodePair> &s)
{
    s.reserve(v[0].size() * v[1].size());
    for (auto const &a : v[0])
    {
        for (auto const &b : v[1])
        {
            NodePair newNode(a, b, parent);
            auto it = visited.find(newNode.hash(map.get_size()));
            if(it != visited.end())
            {
                if(it->second.second)
                {
                    continue;
                }
            }
            if(it != visited.end())
            {
                if(it->second.first - CN_EPSILON < newNode.g)
                {
                    continue;
                }
                else
                {
                    it->second.first = newNode.g;
                }
            }
            else
            {
                visited.emplace(newNode.hash(map.get_size()), std::make_pair(newNode.g, false));
            }

            s.emplace_back(a, b, parent);
        }
    }
}

void MProp::joint_successors(NodePair const& node, Map const& map, std::vector<NodePair> &s, Heuristic const& h_values, NodePair const& goal)
{
    static std::array<std::vector<Node>,2> succs;
    succs[0].clear(); succs[1].clear();
    auto sd(std::min(node.nodes[0].j, node.nodes[1].j));
    for (int i(0); i < 2; ++i)
    {
        if (fequal(sd, node.nodes[i].j))
        {
            find_successors(node.nodes[i], map, succs[i], h_values, goal.nodes[i], i);
        }
        else
        {
            succs[i].push_back(node.nodes[i]);
        }
    }
    joint_product(succs, &node, map, s);
}

void MProp::clear()
{
    open.clear();
    close.clear();
    for (int i(0); i < 2; ++i)
    {
        collision_intervals[i].clear();
        landmarks[i].clear();
        constraints[i].clear();
    }
    visited.clear();
    path.cost = -1;
}

double MProp::dist(const Node& a, const Node& b) const
{
    return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}

void MProp::find_successors(Node const &curNode, const Map &map, std::vector<Node> &succs, Heuristic const& h_values, Node const &goal, int a) const
{
    Node newNode;
    std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
    for(auto move : valid_moves)
    {
        newNode.i = move.i;
        newNode.j = move.j;
        newNode.id = move.id;
        double cost = dist(curNode, newNode);
        newNode.g = curNode.g + cost;
        std::vector<std::pair<double, double>> intervals;
        auto colls_it = collision_intervals[a].find(newNode.id);
        if(colls_it != collision_intervals[a].end())
        {
            std::pair<double, double> interval = {0, CN_INFINITY};
            for(unsigned int i = 0; i < colls_it->second.size(); i++)
            {
                interval.second = colls_it->second[i].first;
                intervals.push_back(interval);
                interval.first = colls_it->second[i].second;
            }
            interval.second = CN_INFINITY;
            intervals.push_back(interval);
        }
        else
        {
            intervals.emplace_back(0, CN_INFINITY);
        }
        auto cons_it = constraints[a].find({curNode.id, newNode.id});
        int id(0);
        for(auto interval: intervals)
        {
            newNode.interval_id = id; // Needed for computing hash
            id++;
            if(interval.second < newNode.g)
            {
                continue;
            }
            if(interval.first > newNode.g)
            {
                newNode.g = interval.first;
            }
            if(cons_it != constraints[a].end())
            {
                for(unsigned int i = 0; i < cons_it->second.size(); i++)
                {
                    if(newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 && newNode.g - cost < cons_it->second[i].t2)
                    {
                        newNode.g = cons_it->second[i].t2 + cost;
                    }
                }
            }
            newNode.interval = interval;
            if(newNode.g - cost > curNode.interval.second || newNode.g > newNode.interval.second)
            {
                continue;
            }
            if(goal.id == agent[a].goal_id) //perfect heuristic is known
            {
                newNode.f = newNode.g + h_values.get_value(newNode.id, agent[a].id);
            }
            else
            {
                double h = sqrt(pow(goal.i - newNode.i, 2) + pow(goal.j - newNode.j, 2));
                for(unsigned int i = 0; i < h_values.get_size(); i++) //differential heuristic with pivots placed to agents goals
                {
                    h = std::max(h, fabs(h_values.get_value(newNode.id, i) - h_values.get_value(goal.id, i)));
                }
                newNode.f = newNode.g + h;
            }
            succs.push_back(newNode);
        }
    }
}

NodePair MProp::find_min()
{
    NodePair min = *open.begin();
    open.pop_front();
    return min;
}

void MProp::add_open(NodePair const& newNode)
{
    if (open.empty() || open.back().f - CN_EPSILON < newNode.f)
    {
        open.push_back(newNode);
        return;
    }
    for(auto iter = open.begin(); iter != open.end(); ++iter)
    {
        if(iter->f > newNode.f + CN_EPSILON) // if newNode.f has lower f-value
        {
            open.emplace(iter, newNode);
            return;
        }
        else if(fabs(iter->f - newNode.f) < CN_EPSILON && newNode.g + CN_EPSILON > iter->g) // if f-values are equal, compare g-values
        {
            open.emplace(iter, newNode);
            return;
        }
    }
    open.push_back(newNode);
    return;
}

std::vector<Node> MProp::reconstruct_path(Node curNode)
{
    path.nodes.clear();
    if(curNode.parent != nullptr)
    do
    {
        path.nodes.insert(path.nodes.begin(), curNode);
        curNode = *curNode.parent;
    }
    while(curNode.parent != nullptr);
    path.nodes.insert(path.nodes.begin(), curNode);
    for(unsigned int i = 0; i < path.nodes.size(); i++)
    {
        unsigned int j = i + 1;
        if(j == path.nodes.size())
            break;
        if(fabs(path.nodes[j].g - path.nodes[i].g - dist(path.nodes[j], path.nodes[i])) > CN_EPSILON)
        {
            Node add = path.nodes[i];
            add.g = path.nodes[j].g - dist(path.nodes[j], path.nodes[i]);
            path.nodes.emplace(path.nodes.begin() + j, add);
        }
    }
    return path.nodes;
}

void MProp::add_collision_interval(int id, std::pair<double, double> interval, int a)
{
    if(collision_intervals[a].count(id) == 0)
        collision_intervals[a].insert({id, {interval}});
    else
        collision_intervals[a][id].push_back(interval);
    std::sort(collision_intervals[a][id].begin(), collision_intervals[a][id].end());
    for(unsigned int i = 0; i + 1 < collision_intervals[a][id].size(); i++)
        if(collision_intervals[a][id][i].second + CN_EPSILON > collision_intervals[a][id][i+1].first)
        {
            collision_intervals[a][id][i].second = collision_intervals[a][id][i+1].second;
            collision_intervals[a][id].erase(collision_intervals[a][id].begin() + i + 1);
            i--;
        }
}

void MProp::add_move_constraint(Move const& move, int a)
{
    std::vector<Move> m_cons(0);
    if(constraints[a].count({move.id1, move.id2}) == 0)
        constraints[a].insert({{move.id1, move.id2}, {move}});
    else
    {
        m_cons = constraints[a].at({move.id1, move.id2});
        bool inserted(false);
        for(unsigned int i = 0; i < m_cons.size(); i++)
        {
            if(inserted)
                break;
            if(m_cons[i].t1 > move.t1)
            {
                if(m_cons[i].t1 < move.t2 + CN_EPSILON)
                {
                    m_cons[i].t1 = move.t1;
                    if(move.t2 + CN_EPSILON > m_cons[i].t2)
                        m_cons[i].t2 = move.t2;
                    inserted = true;
                    if(i != 0)
                        if(m_cons[i-1].t2 + CN_EPSILON > move.t1 && m_cons[i-1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i-1].t2 = move.t2;
                            if(m_cons[i-1].t2 + CN_EPSILON > m_cons[i].t1 && m_cons[i-1].t2 < m_cons[i].t2 + CN_EPSILON)
                            {
                                m_cons[i-1].t2 = m_cons[i].t2;
                                m_cons.erase(m_cons.begin() + i);
                            }
                            inserted = true;
                        }
                }
                else
                {
                    if(i != 0)
                        if(m_cons[i-1].t2 + CN_EPSILON > move.t1 && m_cons[i-1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i-1].t2 = move.t2;
                            inserted = true;
                            break;
                        }
                    m_cons.insert(m_cons.begin() + i, move);
                    inserted = true;
                }
            }
        }
        if(m_cons.back().t2 + CN_EPSILON > move.t1 && m_cons.back().t2 < move.t2 + CN_EPSILON)
            m_cons.back().t2 = move.t2;
        else if(!inserted)
            m_cons.push_back(move);
        constraints[a].at({move.id1, move.id2}) = m_cons;
    }
}

void MProp::make_constraints(std::list<Constraint> &cons, int a)
{
    for(auto con : cons)
    {
        if(con.positive == false)
        {
            if(con.id1 == con.id2) // wait constraint
                add_collision_interval(con.id1, std::make_pair(con.t1, con.t2), a);
            else
                add_move_constraint(Move(con), a);
        }
        else
        {
            bool inserted = false;
            for(unsigned int i = 0; i < landmarks[a].size(); i++)
                if(landmarks[a][i].t1 > con.t1)
                {
                    landmarks[a].insert(landmarks[a].begin() + i, Move(con.t1, con.t2, con.id1, con.id2));
                    inserted = true;
                    break;
                }
            if(!inserted)
                landmarks[a].emplace_back(con.t1, con.t2, con.id1, con.id2);
        }
    }
}

void MProp::append(Path& result, Path const& part)
{
    result.nodes.insert(result.nodes.end(),part.nodes.begin()+1, part.nodes.end());
    //part.nodes.erase(part.nodes.begin());
    //for(auto const& n(part.nodes.begin()+1); n!=part.nodes)
        //result.nodes.push_back(n);
    //return result;
}

std::vector<Path> MProp::find_partial_path(std::vector<NodePair> const& starts, std::vector<NodePair> const& goals, const Map &map, Heuristic &h_values, std::array<double,2> const& max_f)
{
    open.clear();
    close.clear();
    path.cost = -1;
    visited.clear();
    std::vector<Path> paths(goals.size());
    int pathFound(0);
    for(auto s:starts)
    {
        s.parent = nullptr;
        open.push_back(s);
        visited.emplace(s.hash(map.get_size()), std::make_pair(s.g, false));
    }
    while(!open.empty())
    {
        auto curNode = find_min();
        auto v = visited.find(curNode.hash(map.get_size()));
        if(v->second.second)
            continue;
        v->second.second = true;
        auto parent = &close.emplace(curNode.hash(map.get_size()), curNode).first->second;
        if(curNode.id == goals[0].id)
        {
            for (unsigned int i = 0; i < goals.size(); i++)
            {
                if (curNode.nodes[0].g - CN_EPSILON < goals[i].nodes[0].interval.second && goals[i].nodes[0].interval.first - CN_EPSILON < curNode.nodes[0].interval.second) ||
                (curNode.nodes[1].g - CN_EPSILON < goals[i].nodes[1].interval.second && goals[i].nodes[1].interval.first - CN_EPSILON < curNode.nodes[1].interval.second))
                {
                    paths[i].nodes = reconstruct_path(curNode);
                    if (paths[i].nodes.back().g < goals[i].interval.first)
                    {
                        curNode.g = goals[i].interval.first;
                        paths[i].nodes.push_back(curNode);
                    }
                    paths[i].cost = curNode.g;
                    paths[i].expanded = int(close.size());
                    pathFound++;
                }
            }
            if(pathFound == int(goals.size()))
                return paths;
        }
        static std::vector<Node> succs;
        succs.clear();
        find_successors(curNode, map, succs, h_values, Node(goals[0].id, 0, 0, goals[0].i, goals[0].j));
        std::list<NodePair>::iterator it = succs.begin();
        while(it != succs.end())
        {
            if((*it)[0].f > max_f[0] || (*it)[1].f > max_f[1])
            {
                it++;
                continue;
            }
            it->parent = parent;
            add_open(*it);
            it++;
        }
    }
    return paths;
}

std::vector<Node> MProp::get_endpoints(int node_id, double node_i, double node_j, double t1, double t2, int a)
{
    std::vector<Node> nodes;
    nodes = {Node(node_id, 0, 0, node_i, node_j, nullptr, t1, t2)};
    if(collision_intervals[a][node_id].empty())
        return nodes;
    else
        for(unsigned int k = 0; k < collision_intervals[a][node_id].size(); k++)
        {    
            unsigned int i(0);
            while(i < nodes.size())
            {
                Node n = nodes[i];
                auto c = collision_intervals[a][node_id][k];
                bool changed = false;
                if(c.first - CN_EPSILON < n.interval.first && c.second + CN_EPSILON > n.interval.second)
                {
                    nodes.erase(nodes.begin() + i);
                    changed = true;
                }
                else if(c.first - CN_EPSILON < n.interval.first && c.second > n.interval.first)
                {
                    nodes[i].interval.first = c.second;
                    changed = true;
                }
                else if(c.first - CN_EPSILON > n.interval.first && c.second + CN_EPSILON < n.interval.second)
                {
                    nodes[i].interval.second = c.first;
                    nodes.insert(nodes.begin() + i + 1, Node(node_id, 0, 0, node_i, node_j, nullptr, c.second, n.interval.second));
                    changed = true;
                }
                else if(c.first < n.interval.second && c.second + CN_EPSILON > n.interval.second)
                {
                    nodes[i].interval.second = c.first;
                    changed = true;
                }
                if(changed)
                {
                    i = -1;
                    k = 0;
                }
                i++;
            }
        }
    return nodes;
}

double MProp::check_endpoint(Node start, Node goal)
{
    double cost = sqrt(pow(start.i - goal.i, 2) + pow(start.j - goal.j, 2));
    if(start.g + cost < goal.interval.first)
        start.g = goal.interval.first - cost;
    if(constraints.count({start.id, goal.id}) != 0)
    {
        auto it = constraints.find({start.id, goal.id});
        for(unsigned int i = 0; i < it->second.size(); i++)
            if(start.g + CN_EPSILON > it->second[i].t1 && start.g < it->second[i].t2)
                start.g = it->second[i].t2;
    }
    if(start.g > start.interval.second || start.g + cost > goal.interval.second)
        return CN_INFINITY;
    else
        return start.g + cost;
}

Path MProp::mutex_propagation(Agent const& agent1, Agent const& agent2, Map const& map, std::list<Constraint> const& cons1, std::list<Constraint> const& cons2, Heuristic &h_values)
{
    this->clear();
    this->agent = {{agent1,agent2}};
    make_constraints(cons1, 0);
    make_constraints(cons2, 1);

    std::vector<NodePair> starts, goals;
    std::vector<Path> parts, results, new_results;
    Path part, result;
    int expanded(0);
    int i0, i1 = 0;
    if(landmarks[0].size() || landmarks[1].size())
    {
        for(unsigned int i = 0; i <= landmarks.size(); i++)
        {
            if(i == 0)
            {
                starts = {{get_endpoints(agent[0].start_id, agent[0].start_i, agent[0].start_j, 0, CN_INFINITY,0).at(0)},
                          {get_endpoints(agent[1].start_id, agent[1].start_i, agent[1].start_j, 0, CN_INFINITY,1).at(0)}};
                goals = {get_endpoints(landmarks[0][i0].id1, map.get_i(landmarks[0][i0].id1), map.get_j(landmarks[0][i0].id1), landmarks[0][i0].t1, landmarks[0][i0].t2,0),
                         get_endpoints(landmarks[0][i0].id1, map.get_i(landmarks[0][i0].id1), map.get_j(landmarks[0][i0].id1), landmarks[0][i0].t1, landmarks[0][i0].t2,1)};
            }
            else
            {
                starts.clear();
                for(auto p:results)
                    starts.push_back(p.nodes.back());
                if(i == landmarks.size())
                    goals = {get_endpoints(agent[0].goal_id, agent[0].goal_i, agent[0].goal_j, 0, CN_INFINITY,0).back(),
                             get_endpoints(agent[1].goal_id, agent[1].goal_i, agent[1].goal_j, 0, CN_INFINITY,1).back()};
                else
                    goals = {get_endpoints(landmarks[0][i0].id1, map.get_i(landmarks[0][i0].id1), map.get_j(landmarks[0][i0].id1), landmarks[0][i0].t1, landmarks[0][i0].t2),
                             get_endpoints(landmarks[1][i1].id1, map.get_i(landmarks[1][i1].id1), map.get_j(landmarks[1][i1].id1), landmarks[1][i1].t1, landmarks[1][i1].t2)};
            }
            if(goals.empty())
                return Path();
            parts = find_partial_path(starts, goals, map, h_values, goals.back().interval.second);
            expanded += int(close.size());
            new_results.clear();
            if(i == 0)
                for(unsigned int k = 0; k < parts.size(); k++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    new_results.push_back(parts[k]);
                }
            for(unsigned int k = 0; k < parts.size(); k++)
                for(unsigned int j = 0; j < results.size(); j++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    if(fabs(parts[k].nodes[0].interval.first - results[j].nodes.back().interval.first) < CN_EPSILON && fabs(parts[k].nodes[0].interval.second - results[j].nodes.back().interval.second) < CN_EPSILON)
                    {
                        new_results.push_back(results[j]);
                        append(new_results.back(), parts[k]);
                    }
                }
            results = new_results;
            if(results.empty())
                return Path();
            if(i < landmarks.size())
            {
                starts.clear();
                for(auto p:results)
                    starts.push_back(p.nodes.back());
                double offset = sqrt(pow(map.get_i(landmarks[i].id1) - map.get_i(landmarks[i].id2), 2) + pow(map.get_j(landmarks[i].id1) - map.get_j(landmarks[i].id2), 2));
                goals = get_endpoints(landmarks[i].id2, map.get_i(landmarks[i].id2), map.get_j(landmarks[i].id2), landmarks[i].t1 + offset, landmarks[i].t2 + offset);
                if(goals.empty())
                    return Path();
                new_results.clear();
                for(unsigned int k = 0; k < goals.size(); k++)
                {
                    double best_g(CN_INFINITY);
                    int best_start_id = -1;
                    for(unsigned int j = 0; j < starts.size(); j++)
                    {
                        double g = check_endpoint(starts[j], goals[k]);
                        if(g < best_g)
                        {
                            best_start_id = j;
                            best_g = g;
                        }
                    }
                    if(best_start_id >= 0)
                    {
                        goals[k].g = best_g;
                        if(collision_intervals[a][goals[k].id].empty())
                            goals[k].interval.second = CN_INFINITY;
                        else
                        {
                            for(auto c:collision_intervals[a][goals[k].id])
                                if(goals[k].g < c.first)
                                {
                                    goals[k].interval.second = c.first;
                                    break;
                                }
                        }
                        new_results.push_back(results[best_start_id]);
                        if(goals[k].g - starts[best_start_id].g > offset + CN_EPSILON)
                        {
                            new_results.back().nodes.push_back(new_results.back().nodes.back());
                            new_results.back().nodes.back().g = goals[k].g - offset;
                        }
                        new_results.back().nodes.push_back(goals[k]);
                    }
                }

                results = new_results;
                if(results.empty())
                    return Path();
            }
        }
        result = results[0];
    }
    else
    {
        starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
        goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY).back()};
        parts = find_partial_path(starts, goals, map, h_values);
        expanded = int(close.size());
        if(parts[0].cost < 0)
            return Path();
        result = parts[0];
    }
    result.nodes.shrink_to_fit();
    result.cost = result.nodes.back().g;
    result.agentID = agent.id;
    result.expanded = expanded;
    return result;
}
