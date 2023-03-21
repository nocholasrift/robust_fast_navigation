#ifndef JPS_H
#define JPS_H

#include <map>
#include <queue>
#include <vector>
#include <Eigen/Core>

struct JPSNode{
    bool isValid = true;
    int x, y;

    int dirx, diry;

    double cost, manhattan;

}; typedef JPSNode JPSNode_t;

class Compare{
    public:
    // returns True if first argument is higher priority than n2 (lower cost)
    bool operator()(const JPSNode_t& n1, const JPSNode_t& n2){
        return n1.cost+n1.manhattan > n2.cost+n2.manhattan;
    }
};

class JPSPlan{

public:
    JPSPlan();
    
    void set_start(int x, int y);
    void set_destination(int x, int y);
    void set_map(unsigned char* map, int sizeX, int sizeY);
    void JPS();
    std::vector<Eigen::Vector2d> getPath(bool simplify = true);


private:

    double chebyshev_dist(int x, int y);
    double manhattan_distance(int x, int y);
    double octile_dist(int x, int y);
    double euclidean_dist(int x, int y);

    void add_to_queue(int x, int y, int dirx, int diry, double cost);
    bool explore_straight(const JPSNode_t& start);
    bool explore_diagonal(const JPSNode_t& start);

    std::map<int, bool> closedSet;
    std::map<int, int> parents;

    unsigned char* _map;

    int sizeX, sizeY, startX, startY, destX, destY, goalInd;

    std::priority_queue<JPSNode_t, std::deque<JPSNode_t>, Compare> q;

};

#endif
