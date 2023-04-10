#include <math.h>
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <robust_fast_navigation/JPS.h>

JPSPlan::JPSPlan(){
    occupied_val = 100;
}

void JPSPlan::set_start(int x, int y){
    startX = x;
    startY = y;
}

void JPSPlan::set_destination(int x, int y){
    destX = x;
    destY = y;
}

double JPSPlan::chebyshev_dist(int x, int y){
    return std::max(abs(destX-x), abs(destY-y));
}

// im going to assume we never go over max_int number of cells in either direction...
double JPSPlan::manhattan_distance(int x, int y){
    return abs(destX-x) + abs(destY-y);
}

double JPSPlan::octile_dist(int x, int y){
    int dx = abs(destX-x);
    int dy = abs(destY-y);
    return std::max(dx,dy)*(sqrt(2)) + std::min(dx,dy);
}

double JPSPlan::euclidean_dist(int x, int y){
    return sqrt((x-destX)*(x-destX) + (y-destY)*(y-destY));
}

void JPSPlan::set_occ_value(double x){
    occupied_val = x;
}

void JPSPlan::add_to_queue(int x, int y, int dirx, int diry, double cost){
    
    static int count = 0;

    if (closedSet.find(y*sizeX + x) != closedSet.end())
        return;
    
    //std::cout << "adding to queue (" << x << ", " << y << ", " << dirx << ", " << diry << ")" << std::endl;

    JPSNode_t node;
    node.x = x;
    node.y = y;
    node.dirx = dirx;
    node.diry = diry;
    node.cost = cost;
    node.manhattan = manhattan_distance(x,y); // octile_dist(x,y);
    q.push(node);
    count += 1;
}

// JPSNode_t explore_straight(const JPSNode_t& start){
bool JPSPlan::explore_straight(const JPSNode_t& start){

    int dirx = start.dirx;
    int diry = start.diry;

    double cost = start.cost;
    JPSNode_t curr = start;

    closedSet[start.y*sizeX + start.x] = true;
    //std::cout << "explore straight (" << start.x << ", " << start.y << ", " << dirx << ", " << diry << ")" << std::endl;

    while (true){
        JPSNode_t n = curr;
        n.x += dirx;
        n.y += diry;
        cost += 1;
        // if(start.x == 21 && start.y == 26)
        //     //std::cout << "[straight](" << curr.x << ", " << curr.y << ")" << std::endl;
        // is node on border?
        if ( (n.x > sizeX-1 && dirx == 1) || (n.y > sizeY-1 && diry == 1)
            || (n.x < 0 && dirx == -1) || (n.y < 0 && diry == -1)){

            // if (start.x == 21 && start.y == 26)
            //     //std::cout << "poop" << std::endl;
            return false;
        }

        if(_map[n.y*sizeX + n.x] == occupied_val){
            return false;
        }

        if ( n.x == destX && n.y == destY){
            //std::cout << "GOAL FOUND" << std::endl;
            parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
            add_to_queue(n.x, n.y, n.dirx, n.diry, cost);
            return true;
        }

        // only one of dirx or diry can be non-zero in this function call
        bool added = false;
        if (dirx != 0){
            if (n.y != sizeY-1 &&_map[(n.y+1)*sizeX + n.x] == occupied_val && _map[(n.y+1)*sizeX + n.x+dirx] != occupied_val){
                //std::cout << "found forced at (" << n.x << "," << n.y << ")\t" << dirx << "\t1" << std::endl;
                add_to_queue(n.x, n.y, dirx, 1, cost);
                added = true;
            }

            if (n.y != 0 && _map[(n.y-1)*sizeX + n.x] == occupied_val && _map[(n.y-1)*sizeX + n.x+dirx] != occupied_val){
                //std::cout << "found forced at (" << n.x << "," << n.y << ")\t" << dirx << "\t-1" << std::endl;
                add_to_queue(n.x, n.y, dirx, -1, cost);
                added = true;
            }
        }
        
        else{
            if (n.x != sizeX-1 && _map[n.y*sizeX + n.x+1] == occupied_val && _map[(n.y+diry)*sizeX + n.x+1] != occupied_val){
                //std::cout << "found forced at (" << n.x << "," << n.y << ")\t1\t" << diry << std::endl;
                add_to_queue(n.x, n.y, 1, diry, cost);
                added = true;
            }

            if (n.x != 0 && _map[n.y*sizeX + n.x-1] == occupied_val && _map[(n.y+diry)*sizeX + n.x-1] != occupied_val){
                //std::cout << "found forced at (" << n.x << "," << n.y << ")\t-1\t" << diry << std::endl;
                add_to_queue(n.x, n.y, -1, diry, cost);
                added = true;
            }

        }

        // if node has any forced neighbords, it is a jump point, so add to queue as well
        if (added){
            parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
            //std::cout << "parent of (" <<n.x<< "," <<n.y<< ") is (" <<start.x<< "," <<start.y<< ")" << std::endl;
            add_to_queue(n.x, n.y, n.dirx, n.diry, cost);
            return true;
        }

        // might need this, not sure at the moment...
        // closedSet[n.y*sizeX + n.x] = true;
        curr.x = n.x;
        curr.y = n.y;
        curr.cost = cost;

    }
}

bool JPSPlan::explore_diagonal(const JPSNode_t& start){

    int dirx = start.dirx;
    int diry = start.diry;

    double cost = start.cost;
    JPSNode_t curr = start;
    closedSet[start.y*sizeX + start.x] = true;

    //std::cout << "explore diagonal (" << start.x << ", " << start.y << ", " << dirx << ", " << diry << ")" << std::endl;
    
    while(true){
        JPSNode_t n = curr;
        n.x += dirx;
        n.y += diry;
        cost += 1; //sqrt(2);

        // //std::cout << "[diagonal] curr is: " << "(" << curr.x << ", " << curr.y << ")" << std::endl;

        if ( (n.x > sizeX-1 && dirx == 1) || (n.y > sizeY-1 && diry == 1)
            || (n.x < 0 && dirx == -1) || (n.y < 0 && diry == -1)){
            return false;
        }

        if(_map[n.y*sizeX + n.x] == occupied_val){
            return false;
        }

        if ( n.x == destX && n.y == destY){
            //std::cout << "GOAL FOUND" << std::endl;
            parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
            add_to_queue(n.x, n.y, n.dirx, n.diry, cost);
            return true;
        }

        bool added = false;
        if ( _map[n.y*sizeX + curr.x] == occupied_val && _map[(n.y+diry)*sizeX + curr.x] != occupied_val){
            //std::cout << "JP @ (" << n.x << "," << n.y << ")" << std::endl;
            parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
            add_to_queue(n.x, n.y, -n.dirx, n.diry, cost);
            added = true;
        }

        if ( _map[curr.y*sizeX + n.x] == occupied_val && _map[curr.y*sizeX + n.x+dirx] != occupied_val){
            //std::cout << "JP @ (" << n.x << "," << n.y << ")" << std::endl;
            parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
            add_to_queue(n.x, n.y, n.dirx, -n.diry, cost);
            added = true;
        }

        // if(added){
        //     add_to_queue(n.x, n.y, n.dirx, 0, cost);
        //     add_to_queue(n.x, n.y, 0, n.diry, cost);
        // }

        JPSNode_t horN;
        horN.x = n.x;
        horN.y = n.y;
        horN.dirx = dirx;
        horN.diry = 0;
        horN.cost = cost;
        bool found_hor = explore_straight(horN);
        
        JPSNode_t verN;
        verN.x = n.x;
        verN.y = n.y;
        verN.dirx = 0;
        verN.diry = diry;
        verN.cost = cost;
        bool found_ver = explore_straight(verN);

        if (found_ver || found_hor){
            parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
            //std::cout << "JP @ found vert/horiz to (" << n.x << "," << n.y << ")" << std::endl;
            // add_to_queue(n.x, n.y, n.dirx, n.diry, cost);
            // return true;
        }

        if (added){
            // add_to_queue(n.x, n.y, n.dirx, n.diry, cost);
            // return true;
        }

        // if (added || found_ver || found_hor){
        //     parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
        //     add_to_queue(n.x, n.y, n.dirx, n.diry, cost);
        // }

        // moving diagonally
        curr.x = n.x;
        curr.y = n.y;
        curr.cost = cost;
    }

    //std::cout << "**************************" << std::endl;

}

void JPSPlan::JPS(){

    closedSet.clear();
    parents.clear();

    goalInd = -1;

    if ( _map[startY*sizeX + startX] == occupied_val || _map[destY*sizeX + destX] == occupied_val){
        std::cerr << "start or destination is in occupied space" << std::endl;
        return;
    }

    std::cout << "start " << (int) _map[startY*sizeX + startX] << " and destination " << (int)_map[destY*sizeX + destX] << std::endl;

    // cardinal
    add_to_queue(startX, startY, 1, 0, 0);
    add_to_queue(startX, startY, -1, 0, 0);
    add_to_queue(startX, startY, 0, 1, 0);
    add_to_queue(startX, startY, 0, -1, 0);
    
    // diagonal
    add_to_queue(startX, startY, 1, 1, 0);
    add_to_queue(startX, startY, 1, -1, 0);
    add_to_queue(startX, startY, -1, 1, 0);
    add_to_queue(startX, startY, -1, -1, 0);

    parents[startY*sizeY + startX] = startY*sizeY + startX;

    while (q.size() > 0){
        JPSNode_t node = q.top();
        q.pop();

        //std::cout << "LOOKING AT NODE (" << node.x << ", " << node.y <<  
        //     ", " << node.dirx << ", " << node.diry << ", " << node.cost+node.manhattan << ")" << std::endl;

        if (node.x == destX && node.y == destY){
            goalInd = destY * sizeX + destX;
            //std::cout << "found goal (" << destX << ", " << destY <<  ") :)" << std::endl;
            break;
        }

        if (node.dirx == 0 || node.diry == 0){
            explore_straight(node);
        }
        else{
            explore_diagonal(node);
        }

    }
}

std::vector<Eigen::Vector2d> JPSPlan::getPath(bool simplify){

    std::vector<Eigen::Vector2d> ret;

    if (goalInd == -1)
        return ret;

    int x, y;
    y = goalInd/sizeX;
    x = goalInd - y*sizeX;
    ret.push_back(Eigen::Vector2d(x,y));

    int i = parents[goalInd];
    int j = 0;

    while(i != startY*sizeX + startX && j++ < 1000){
        //std::cout << "(" << i-((int)(i/sizeX))*sizeX << "," << i/sizeX << ") -->";
        y = i/sizeX;
        x = i - y*sizeX;
        ret.push_back(Eigen::Vector2d(x,y));
        i = parents[i];
    }

    //std::cout << "(" << i-((int)(i/sizeX))*sizeX << "," << i/sizeX << ")\n";
    y = i/sizeX;
    x = i - y*sizeX;
    ret.push_back(Eigen::Vector2d(x,y));
    std::reverse(ret.begin(), ret.end());

    if (simplify){

        for(int i = 1; i < ret.size()-1; ){
            
            if ((ret[i+1][0] == ret[i][0] && ret[i-1][0]== ret[i][0]) ||
                (ret[i+1][1] == ret[i][1] && ret[i-1][1] == ret[i][1]) )
                ret.erase(ret.begin()+i);
            else
                i++;
        
        }

    }
    
    ret = simplifyPath(ret);
    std::reverse(std::begin(ret), std::end(ret));
    ret = simplifyPath(ret);
    std::reverse(std::begin(ret), std::end(ret));
    return ret;
}

void JPSPlan::set_map(unsigned char* map, int sizeX, int sizeY){
    this->_map = map;
    this->sizeX = sizeX;
    this->sizeY = sizeY;
}

std::vector<Eigen::Vector2d> JPSPlan::simplifyPath(const std::vector<Eigen::Vector2d>& path){
    if (path.size() < 2)
        return path;

    // cut zigzag segment
    std::vector<Eigen::Vector2d> optimized_path;

    Eigen::Vector2d pose1 = path[0];
    Eigen::Vector2d pose2 = path[1];
    Eigen::Vector2d prev_pose = pose1;

    optimized_path.push_back(pose1);

    double cost1, cost2, cost3;

    if (!isBlocked(pose1, pose2))
        cost1 = (pose1 - pose2).norm();
    else
        cost1 = std::numeric_limits<double>::infinity();

    for (unsigned int i = 1; i < path.size() - 1; i++)
    {
        pose1 = path[i];
        pose2 = path[i + 1];
        if (!isBlocked(pose1, pose2))
            cost2 = (pose1 - pose2).norm();
        else
            cost2 = std::numeric_limits<double>::infinity();

        if (!isBlocked(prev_pose, pose2))
            cost3 = (prev_pose - pose2).norm();
        else
            cost3 = std::numeric_limits<double>::infinity();

        if (cost3 < cost1 + cost2)
            cost1 = cost3;
        else
        {
            optimized_path.push_back(path[i]);
            cost1 = (pose1 - pose2).norm();
            prev_pose = pose1;
        }
    }

    optimized_path.push_back(path.back());
    return optimized_path;
}

bool JPSPlan::isBlocked(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double max_range){
    
    // raycast and bresenham
    unsigned int size_x = this->sizeX;

    double sx = p1[0];
    double sy = p1[1];
    double ex = p2[0];
    double ey = p2[1];

    int dx = ex - sx;
    int dy = ey - sy;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    
    int offset_dx = dx > 0 ? 1 : -1;
    int offset_dy = (dy > 0 ? 1 : -1) * this->sizeX;

    unsigned int offset = sy * size_x + sx;

    double dist = hypot(dx, dy);
    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_range / dist);

    unsigned int term; 
    if (abs_dx >= abs_dy){
        int error_y = abs_dx / 2;
        return bresenham( abs_dx, abs_dy, error_y, offset_dx, offset_dy, 
                offset, (unsigned int)(scale * abs_dx), term);
    } else{
        int error_x = abs_dy / 2;
        return bresenham(abs_dy, abs_dx, error_x, offset_dy, offset_dx,
                offset, (unsigned int)(scale * abs_dy), term);
    }

}

bool JPSPlan::bresenham(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
    int offset_b, unsigned int offset, unsigned int max_range, unsigned int& term){
    
    unsigned int end = std::min(max_range, abs_da);
    unsigned int mx, my;

    bool is_hit = false;
    for (unsigned int i = 0; i < end; ++i) {
        offset += offset_a;
        error_b += abs_db;
        
        if (_map[offset] == occupied_val){
            is_hit = true;
            break;
        }

        if ((unsigned int)error_b >= abs_da) {
            offset += offset_b;
            error_b -= abs_da;
        }

        if (_map[offset] == occupied_val){
            is_hit = true;
            break;
        }
    }
    
    term = offset;
    return is_hit;
}
