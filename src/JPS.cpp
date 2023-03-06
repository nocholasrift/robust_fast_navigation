#include <math.h>
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <robust_fast_navigation/JPS.h>

JPSPlanner::JPSPlanner(){

}

void JPSPlanner::set_start(int x, int y){
    startX = x;
    startY = y;
}

void JPSPlanner::set_destination(int x, int y){
    destX = x;
    destY = y;
}

double JPSPlanner::chebyshev_dist(int x, int y){
    return std::max(abs(destX-x), abs(destY-y));
}

// im going to assume we never go over max_int number of cells in either direction...
double JPSPlanner::manhattan_distance(int x, int y){
    return abs(destX-x) + abs(destY-y);
}

double JPSPlanner::octile_dist(int x, int y){
    int dx = abs(destX-x);
    int dy = abs(destY-y);
    return std::max(dx,dy)*(sqrt(2)) + std::min(dx,dy);
}

double JPSPlanner::euclidean_dist(int x, int y){
    return sqrt((x-destX)*(x-destX) + (y-destY)*(y-destY));
}

void JPSPlanner::add_to_queue(int x, int y, int dirx, int diry, double cost){
    
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
bool JPSPlanner::explore_straight(const JPSNode_t& start){

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

        if(_map[n.y*sizeX + n.x] != 0){
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
            if (n.y != sizeY-1 &&_map[(n.y+1)*sizeX + n.x] != 0 && _map[(n.y+1)*sizeX + n.x+dirx] == 0){
                //std::cout << "found forced at (" << n.x << "," << n.y << ")\t" << dirx << "\t1" << std::endl;
                add_to_queue(n.x, n.y, dirx, 1, cost);
                added = true;
            }

            if (n.y != 0 && _map[(n.y-1)*sizeX + n.x] != 0 && _map[(n.y-1)*sizeX + n.x+dirx] == 0){
                //std::cout << "found forced at (" << n.x << "," << n.y << ")\t" << dirx << "\t-1" << std::endl;
                add_to_queue(n.x, n.y, dirx, -1, cost);
                added = true;
            }
        }
        
        else{
            if (n.x != sizeX-1 && _map[n.y*sizeX + n.x+1] != 0 && _map[(n.y+diry)*sizeX + n.x+1] == 0){
                //std::cout << "found forced at (" << n.x << "," << n.y << ")\t1\t" << diry << std::endl;
                add_to_queue(n.x, n.y, 1, diry, cost);
                added = true;
            }

            if (n.x != 0 && _map[n.y*sizeX + n.x-1] != 0 && _map[(n.y+diry)*sizeX + n.x-1] == 0){
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

bool JPSPlanner::explore_diagonal(const JPSNode_t& start){

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

        if(_map[n.y*sizeX + n.x] != 0){
            return false;
        }

        if ( n.x == destX && n.y == destY){
            //std::cout << "GOAL FOUND" << std::endl;
            parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
            add_to_queue(n.x, n.y, n.dirx, n.diry, cost);
            return true;
        }

        bool added = false;
        if ( _map[n.y*sizeX + curr.x] != 0 && _map[(n.y+diry)*sizeX + curr.x] == 0){
            //std::cout << "JP @ (" << n.x << "," << n.y << ")" << std::endl;
            parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
            add_to_queue(n.x, n.y, -n.dirx, n.diry, cost);
            added = true;
        }

        if ( _map[curr.y*sizeX + n.x] != 0 && _map[curr.y*sizeX + n.x+dirx] == 0){
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

void JPSPlanner::JPS(){

    closedSet.clear();
    parents.clear();

    goalInd = -1;

    if ( _map[startY*sizeX + startX] != 0 || _map[destY*sizeX + destX] != 0){
        std::cerr << "start or destination in is occupied space" << std::endl;
        return;
    }

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

std::vector<Eigen::Vector2d> JPSPlanner::getPath(bool simplify){

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
    
    return ret;
}

void JPSPlanner::set_map(unsigned char* map, int sizeX, int sizeY){
    this->_map = map;
    this->sizeX = sizeX;
    this->sizeY = sizeY;
}

// void set_straight_line(int start, int end){
//     // //std::cout << start << " " << end << std::endl;
//     int sy = start / sizeX;
//     int sx = start - sy*sizeX;

//     int ey = end / sizeX;
//     int ex = end - ey*sizeX;

//     // //std::cout << "(" << sx << ", " << sy << ")" << std::endl;
//     // //std::cout << "(" << ex << ", " << ey << ")" << std::endl;

//     int dx = ex - sx > 0 ? 1 : -1;
//     int dy = ey - sy > 0 ? 1 : -1;

//     if (ex - sx == 0)
//         dx = 0;
//     if (ey - sy == 0)
//         dy = 0;

//     int x = sx, y = sy;
//     while (x != ex || y != ey){
//         _map[y*sizeX + x] = 7;
//         x += dx;
//         y += dy;
//         // //std::cout << "***********" << std::endl;
//         // //std::cout << "(" << sx << ", " << sy << ")" << std::endl;
//         // //std::cout << "(" << ex << ", " << ey << ")" << std::endl;
//         // //std::cout << dx << " " << dy << std::endl;
//         // //std::cout << x << " " << y << std::endl;
//     }

// }

// void setPath(int goalInd){
//     int i = parents[goalInd];
//     int j = 0;
//     set_straight_line(goalInd, i);

//     while(i != startY * sizeX + startX && j++ < 20){
//         set_straight_line(i, parents[i]);
//         i = parents[i];
//     }

//     int startInd = startY*sizeX + startX;
//     _map[startInd] = 6;
//     _map[goalInd] = 8;
// }

// void getPath(int goalInd){

//     int i = parents[goalInd];
//     int j = 0;
//     //std::cout << "path is: [" << "(" << goalInd-((int)(goalInd/sizeX))*sizeX << "," << goalInd/sizeX << ")" << "-->";
//     while(i != startY * sizeX + startX && j++ < 20){
//         //std::cout << "(" << i-((int)(i/sizeX))*sizeX << "," << i/sizeX << ") -->";
//         // //std::cout << i << ", ";
//         i = parents[i];
//     }

//     //std::cout << "(" << startX << ", " << startY << ")" <<std::endl;
// }

// void print_board(){
//     for(int j = sizeY-1; j >= 0; j--){
//         for(int i = 0; i < sizeX; i++){
//             if (_map[j*sizeX + i] == 7)
//                 //std::cout << "\033[1;31m"<< (int)_map[j*sizeX + i] << "\033[0m ";
//             else if (_map[j*sizeX + i] != 0 && _map[j*sizeX + i] != 1)
//                 //std::cout << "\033[1;32m"<< (int)_map[j*sizeX + i] << "\033[0m ";
//             else if (_map[j*sizeX + i] == 1)
//                 //std::cout << "\033[1;44m \033[0m ";
//             else
//                 //std::cout << (int)_map[j*sizeX + i] << " ";
//         }
//         //std::cout << "\n";
//     }
// }

// void load_from_file(int& sX, int& sY, int& eX, int& eY){

//     std::ifstream in;
//     in.open("map.txt");

//     int element;
//     if (in.is_open()){
//         int i = 0;
//         while(in >> element){
//             if (element == 6){
//                 sY = i/sizeX;
//                 sX = i-sY*sizeX;
//                 _map[i++] = 0;
//             } 
//             else if(element == 8){
//                 eY = i/sizeX;
//                 eX = i-eY*sizeX;
//                 _map[i++] = 0;
//             } else
//                 _map[i++] = element;
//         }
//     }

//     in.close();
// }

// int main(){

//     sizeX = 50;
//     sizeY = 50;
//     _map = new unsigned char[sizeX*sizeY];

//     for(int i = 0; i < sizeX*sizeY; i++)
//         _map[i] = 0;

//     srand((unsigned) time(NULL));
//     // for(int i = 0; i < sizeX; i++){
//     //     for(int j = 0; j < sizeY; j++){
//     //         if (i == 20 || i == 29 || j  20 && j < 30)
//     //             _map[i] = 1;
//     //     }
//     // }

//     // _map[25*sizeX+29] = 0;

//     _map[23*sizeX + 23] = 1;
//     _map[24*sizeX + 23] = 1;
//     _map[25*sizeX + 23] = 1;
//     _map[26*sizeX + 23] = 1;
//     _map[27*sizeX + 23] = 1;

//     _map[23*sizeX + 27] = 1;
//     _map[24*sizeX + 27] = 1;
//     _map[25*sizeX + 27] = 0;
//     _map[26*sizeX + 27] = 1;
//     _map[27*sizeX + 27] = 1;

//     _map[23*sizeX + 23] = 1;
//     _map[23*sizeX + 24] = 1;
//     _map[23*sizeX + 25] = 1;
//     _map[23*sizeX + 26] = 1;
//     _map[23*sizeX + 27] = 1;

//     _map[27*sizeX + 23] = 1;
//     _map[27*sizeX + 24] = 1;
//     _map[27*sizeX + 25] = 1;
//     _map[27*sizeX + 26] = 1;
//     _map[27*sizeX + 27] = 1;

//     _map[30*sizeX + 6] = 1;
//     _map[27*sizeX + 7] = 1;
//     _map[24*sizeX + 8] = 1;
//     _map[21*sizeX + 16] = 1;
//     // _map[22*sizeX + 12] = 1;

//     for(int i = 0; i < 30; i++){
//         int x = rand() % sizeX;
//         int y = rand() % sizeY;
//         _map[y*sizeX + x] = 1;
//     }

//     int sX = rand() % sizeX;
//     int sY = rand() % sizeY;

//     while(_map[sY*sizeX + sX] != 0){
//         sX = rand() % sizeX;
//         sY = rand() % sizeY;
//     }

//     int eX = rand() % sizeX;
//     int eY = rand() % sizeY;

//     while(_map[eY*sizeX + eX] != 0){
//         eX = rand() % sizeX;
//         eY = rand() % sizeY;
//     }

//     // load_from_file(sX, sY, eX, eY);

//     set_start(sX, sY);
//     set_destination(eX,eY);

//     // std::ofstream myfile("map.txt");
//     // if(myfile.is_open()){
//     //     for(int i = 0; i < sizeX*sizeY; i++){
//     //         if (i == sY*sizeX + sX)
//     //             myfile << "6 ";
//     //         else if(i == eY*sizeX + eX)
//     //             myfile << "8 ";
//     //         else
//     //             myfile << (int)_map[i] << " ";
//     //     }
//     //     myfile.close();
//     // }

//     int ind = JPS();
//     // ind = -1;
//     if (ind < 0){
//         //std::cout << "goal not found!" << std::endl;
//         print_board();
//         return -1;
//     }

//     getPath(ind);
    
//     setPath(ind);
//     print_board();

//     return 0;
// }

