#include <robust_fast_navigation/JPS3D.h>

JPS3DNeib::JPS3DNeib()
{
    int id = 0;
    for (int dz = -1; dz <= 1; ++dz)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
                for (int dev = 0; dev < nsz[norm1][0]; ++dev)
                    Neib(dx, dy, dz, norm1, dev,
                         ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
                for (int dev = 0; dev < nsz[norm1][1]; ++dev)
                {
                    FNeib(dx, dy, dz, norm1, dev,
                          f1[id][0][dev], f1[id][1][dev], f1[id][2][dev],
                          f2[id][0][dev], f2[id][1][dev], f2[id][2][dev]);
                }
                id++;
            }
        }
    }
}

void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
                     int &tx, int &ty, int &tz)
{
    switch (norm1)
    {
    case 0:
        switch (dev)
        {
        case 0:
            tx = 1;
            ty = 0;
            tz = 0;
            return;
        case 1:
            tx = -1;
            ty = 0;
            tz = 0;
            return;
        case 2:
            tx = 0;
            ty = 1;
            tz = 0;
            return;
        case 3:
            tx = 1;
            ty = 1;
            tz = 0;
            return;
        case 4:
            tx = -1;
            ty = 1;
            tz = 0;
            return;
        case 5:
            tx = 0;
            ty = -1;
            tz = 0;
            return;
        case 6:
            tx = 1;
            ty = -1;
            tz = 0;
            return;
        case 7:
            tx = -1;
            ty = -1;
            tz = 0;
            return;
        case 8:
            tx = 0;
            ty = 0;
            tz = 1;
            return;
        case 9:
            tx = 1;
            ty = 0;
            tz = 1;
            return;
        case 10:
            tx = -1;
            ty = 0;
            tz = 1;
            return;
        case 11:
            tx = 0;
            ty = 1;
            tz = 1;
            return;
        case 12:
            tx = 1;
            ty = 1;
            tz = 1;
            return;
        case 13:
            tx = -1;
            ty = 1;
            tz = 1;
            return;
        case 14:
            tx = 0;
            ty = -1;
            tz = 1;
            return;
        case 15:
            tx = 1;
            ty = -1;
            tz = 1;
            return;
        case 16:
            tx = -1;
            ty = -1;
            tz = 1;
            return;
        case 17:
            tx = 0;
            ty = 0;
            tz = -1;
            return;
        case 18:
            tx = 1;
            ty = 0;
            tz = -1;
            return;
        case 19:
            tx = -1;
            ty = 0;
            tz = -1;
            return;
        case 20:
            tx = 0;
            ty = 1;
            tz = -1;
            return;
        case 21:
            tx = 1;
            ty = 1;
            tz = -1;
            return;
        case 22:
            tx = -1;
            ty = 1;
            tz = -1;
            return;
        case 23:
            tx = 0;
            ty = -1;
            tz = -1;
            return;
        case 24:
            tx = 1;
            ty = -1;
            tz = -1;
            return;
        case 25:
            tx = -1;
            ty = -1;
            tz = -1;
            return;
        }
    case 1:
        tx = dx;
        ty = dy;
        tz = dz;
        return;
    case 2:
        switch (dev)
        {
        case 0:
            if (dz == 0)
            {
                tx = 0;
                ty = dy;
                tz = 0;
                return;
            }
            else
            {
                tx = 0;
                ty = 0;
                tz = dz;
                return;
            }
        case 1:
            if (dx == 0)
            {
                tx = 0;
                ty = dy;
                tz = 0;
                return;
            }
            else
            {
                tx = dx;
                ty = 0;
                tz = 0;
                return;
            }
        case 2:
            tx = dx;
            ty = dy;
            tz = dz;
            return;
        }
    case 3:
        switch (dev)
        {
        case 0:
            tx = dx;
            ty = 0;
            tz = 0;
            return;
        case 1:
            tx = 0;
            ty = dy;
            tz = 0;
            return;
        case 2:
            tx = 0;
            ty = 0;
            tz = dz;
            return;
        case 3:
            tx = dx;
            ty = dy;
            tz = 0;
            return;
        case 4:
            tx = dx;
            ty = 0;
            tz = dz;
            return;
        case 5:
            tx = 0;
            ty = dy;
            tz = dz;
            return;
        case 6:
            tx = dx;
            ty = dy;
            tz = dz;
            return;
        }
    }
}

void JPS3DNeib::FNeib(int dx, int dy, int dz, int norm1, int dev,
                      int &fx, int &fy, int &fz,
                      int &nx, int &ny, int &nz)
{
    switch (norm1)
    {
    case 1:
        switch (dev)
        {
        case 0:
            fx = 0;
            fy = 1;
            fz = 0;
            break;
        case 1:
            fx = 0;
            fy = -1;
            fz = 0;
            break;
        case 2:
            fx = 1;
            fy = 0;
            fz = 0;
            break;
        case 3:
            fx = 1;
            fy = 1;
            fz = 0;
            break;
        case 4:
            fx = 1;
            fy = -1;
            fz = 0;
            break;
        case 5:
            fx = -1;
            fy = 0;
            fz = 0;
            break;
        case 6:
            fx = -1;
            fy = 1;
            fz = 0;
            break;
        case 7:
            fx = -1;
            fy = -1;
            fz = 0;
            break;
        }
        nx = fx;
        ny = fy;
        nz = dz;
        // switch order if different direction
        if (dx != 0)
        {
            fz = fx;
            fx = 0;
            nz = fz;
            nx = dx;
        }
        if (dy != 0)
        {
            fz = fy;
            fy = 0;
            nz = fz;
            ny = dy;
        }
        return;
    case 2:
        if (dx == 0)
        {
            switch (dev)
            {
            case 0:
                fx = 0;
                fy = 0;
                fz = -dz;
                nx = 0;
                ny = dy;
                nz = -dz;
                return;
            case 1:
                fx = 0;
                fy = -dy;
                fz = 0;
                nx = 0;
                ny = -dy;
                nz = dz;
                return;
            case 2:
                fx = 1;
                fy = 0;
                fz = 0;
                nx = 1;
                ny = dy;
                nz = dz;
                return;
            case 3:
                fx = -1;
                fy = 0;
                fz = 0;
                nx = -1;
                ny = dy;
                nz = dz;
                return;
            case 4:
                fx = 1;
                fy = 0;
                fz = -dz;
                nx = 1;
                ny = dy;
                nz = -dz;
                return;
            case 5:
                fx = 1;
                fy = -dy;
                fz = 0;
                nx = 1;
                ny = -dy;
                nz = dz;
                return;
            case 6:
                fx = -1;
                fy = 0;
                fz = -dz;
                nx = -1;
                ny = dy;
                nz = -dz;
                return;
            case 7:
                fx = -1;
                fy = -dy;
                fz = 0;
                nx = -1;
                ny = -dy;
                nz = dz;
                return;
            // Extras
            case 8:
                fx = 1;
                fy = 0;
                fz = 0;
                nx = 1;
                ny = dy;
                nz = 0;
                return;
            case 9:
                fx = 1;
                fy = 0;
                fz = 0;
                nx = 1;
                ny = 0;
                nz = dz;
                return;
            case 10:
                fx = -1;
                fy = 0;
                fz = 0;
                nx = -1;
                ny = dy;
                nz = 0;
                return;
            case 11:
                fx = -1;
                fy = 0;
                fz = 0;
                nx = -1;
                ny = 0;
                nz = dz;
                return;
            }
        }
        else if (dy == 0)
        {
            switch (dev)
            {
            case 0:
                fx = 0;
                fy = 0;
                fz = -dz;
                nx = dx;
                ny = 0;
                nz = -dz;
                return;
            case 1:
                fx = -dx;
                fy = 0;
                fz = 0;
                nx = -dx;
                ny = 0;
                nz = dz;
                return;
            case 2:
                fx = 0;
                fy = 1;
                fz = 0;
                nx = dx;
                ny = 1;
                nz = dz;
                return;
            case 3:
                fx = 0;
                fy = -1;
                fz = 0;
                nx = dx;
                ny = -1;
                nz = dz;
                return;
            case 4:
                fx = 0;
                fy = 1;
                fz = -dz;
                nx = dx;
                ny = 1;
                nz = -dz;
                return;
            case 5:
                fx = -dx;
                fy = 1;
                fz = 0;
                nx = -dx;
                ny = 1;
                nz = dz;
                return;
            case 6:
                fx = 0;
                fy = -1;
                fz = -dz;
                nx = dx;
                ny = -1;
                nz = -dz;
                return;
            case 7:
                fx = -dx;
                fy = -1;
                fz = 0;
                nx = -dx;
                ny = -1;
                nz = dz;
                return;
            // Extras
            case 8:
                fx = 0;
                fy = 1;
                fz = 0;
                nx = dx;
                ny = 1;
                nz = 0;
                return;
            case 9:
                fx = 0;
                fy = 1;
                fz = 0;
                nx = 0;
                ny = 1;
                nz = dz;
                return;
            case 10:
                fx = 0;
                fy = -1;
                fz = 0;
                nx = dx;
                ny = -1;
                nz = 0;
                return;
            case 11:
                fx = 0;
                fy = -1;
                fz = 0;
                nx = 0;
                ny = -1;
                nz = dz;
                return;
            }
        }
        else
        { // dz==0
            switch (dev)
            {
            case 0:
                fx = 0;
                fy = -dy;
                fz = 0;
                nx = dx;
                ny = -dy;
                nz = 0;
                return;
            case 1:
                fx = -dx;
                fy = 0;
                fz = 0;
                nx = -dx;
                ny = dy;
                nz = 0;
                return;
            case 2:
                fx = 0;
                fy = 0;
                fz = 1;
                nx = dx;
                ny = dy;
                nz = 1;
                return;
            case 3:
                fx = 0;
                fy = 0;
                fz = -1;
                nx = dx;
                ny = dy;
                nz = -1;
                return;
            case 4:
                fx = 0;
                fy = -dy;
                fz = 1;
                nx = dx;
                ny = -dy;
                nz = 1;
                return;
            case 5:
                fx = -dx;
                fy = 0;
                fz = 1;
                nx = -dx;
                ny = dy;
                nz = 1;
                return;
            case 6:
                fx = 0;
                fy = -dy;
                fz = -1;
                nx = dx;
                ny = -dy;
                nz = -1;
                return;
            case 7:
                fx = -dx;
                fy = 0;
                fz = -1;
                nx = -dx;
                ny = dy;
                nz = -1;
                return;
            // Extras
            case 8:
                fx = 0;
                fy = 0;
                fz = 1;
                nx = dx;
                ny = 0;
                nz = 1;
                return;
            case 9:
                fx = 0;
                fy = 0;
                fz = 1;
                nx = 0;
                ny = dy;
                nz = 1;
                return;
            case 10:
                fx = 0;
                fy = 0;
                fz = -1;
                nx = dx;
                ny = 0;
                nz = -1;
                return;
            case 11:
                fx = 0;
                fy = 0;
                fz = -1;
                nx = 0;
                ny = dy;
                nz = -1;
                return;
            }
        }
    case 3:
        switch (dev)
        {
        case 0:
            fx = -dx;
            fy = 0;
            fz = 0;
            nx = -dx;
            ny = dy;
            nz = dz;
            return;
        case 1:
            fx = 0;
            fy = -dy;
            fz = 0;
            nx = dx;
            ny = -dy;
            nz = dz;
            return;
        case 2:
            fx = 0;
            fy = 0;
            fz = -dz;
            nx = dx;
            ny = dy;
            nz = -dz;
            return;
        // Need to check up to here for forced!
        case 3:
            fx = 0;
            fy = -dy;
            fz = -dz;
            nx = dx;
            ny = -dy;
            nz = -dz;
            return;
        case 4:
            fx = -dx;
            fy = 0;
            fz = -dz;
            nx = -dx;
            ny = dy;
            nz = -dz;
            return;
        case 5:
            fx = -dx;
            fy = -dy;
            fz = 0;
            nx = -dx;
            ny = -dy;
            nz = dz;
            return;
        // Extras
        case 6:
            fx = -dx;
            fy = 0;
            fz = 0;
            nx = -dx;
            ny = 0;
            nz = dz;
            return;
        case 7:
            fx = -dx;
            fy = 0;
            fz = 0;
            nx = -dx;
            ny = dy;
            nz = 0;
            return;
        case 8:
            fx = 0;
            fy = -dy;
            fz = 0;
            nx = 0;
            ny = -dy;
            nz = dz;
            return;
        case 9:
            fx = 0;
            fy = -dy;
            fz = 0;
            nx = dx;
            ny = -dy;
            nz = 0;
            return;
        case 10:
            fx = 0;
            fy = 0;
            fz = -dz;
            nx = 0;
            ny = dy;
            nz = -dz;
            return;
        case 11:
            fx = 0;
            fy = 0;
            fz = -dz;
            nx = dx;
            ny = 0;
            nz = -dz;
            return;
        }
    }
}

/**********************************************************************
  Simple constructor which sets occupied_val field to default of 100.
***********************************************************************/
JPS3DPlan::JPS3DPlan()
{
    occupied_val = 100;
    originX = 0;
    originY = 0;
    originZ = 0;
}

/**********************************************************************
  Function to set start of JPS. The (x,y) coordinate should be in grid
  cell coordinates.

  Inputs:
    - x: starting x position
    - y: starting y position
***********************************************************************/
void JPS3DPlan::set_start(int x, int y, int z)
{
    startX = x;
    startY = y;
    startZ = z;
}

/**********************************************************************
  Function to set destination of JPS. The (x,y) coordinate should be in
  grid cell coordinates.

  Inputs:
    - x: destination x position
    - y: destination y position
***********************************************************************/
void JPS3DPlan::set_destination(int x, int y, int z)
{
    destX = x;
    destY = y;
    destZ = z;
}

double JPS3DPlan::euclidean_dist(int x, int y, int z)
{
    return sqrt((x - destX) * (x - destX) + (y - destY) * (y - destY) + (z - destZ) * (z - destZ));
}

/**********************************************************************
  Set the value which the JPS should use as occupied in the grid. For
  traditional maps/costmaps this value is usually 100 or 253/254 (in
  ROS at least).

  Inputs:
    - x: value which indicates occupancy in the grid.
***********************************************************************/
void JPS3DPlan::set_occ_value(double x)
{
    occupied_val = x;
}

/**********************************************************************
  This function pushes a node onto the priority queue, where cost is
  based on the cost-to-go heuristic function of the node in question.
  To see the comparator for and instantiation of the priority queue,
  see JPS.h.

  Inputs:
    - x: x grid cell coordinate of the node
    - y: y grid cell coordinate of the node
    - dirx: which direction in x is the node searching in. This
      determines whether the node will use explore_straight or
      explore_diagonal when popped off queue.
    - diry: which direction in y is the node searching in.
    - cost: heuristic cost-to-go associated with the node.
***********************************************************************/
void JPS3DPlan::add_to_queue(int x, int y, int z, int dirx, int diry, int dirz, double cost)
{

    static int count = 0;

    if (closedSet.find(y * sizeX + x) != closedSet.end())
        return;

    // std::cout << "adding to queue (" << x << ", " << y << ", " << dirx << ", " << diry << ")" << std::endl;

    JPS3DNode_t node;
    node.x = x;
    node.y = y;
    node.dirx = dirx;
    node.diry = diry;
    node.dirz = dirz;
    node.cost = cost;
    node.heuristic = euclidean_dist(x, y, z);
    q.push(node);
    count += 1;
}

bool JPS3DPlan::add_to_parents(const JPS3DNode_t &node, const JPS3DNode_t &parent, double cost)
{

    // check if node is not in parents
    if (parents.find(node.y * sizeX + node.x) == parents.end())
    {
        parents[node.z * sizeX * sizeY + node.y * sizeX + node.x] = std::make_pair(parent.y * sizeX + parent.x, cost);
        return true;
    }

    // node is in parents, but check if cost is lower
    if (cost < parents[node.y * sizeX + node.x].second)
    {
        parents[node.z * sizeX * sizeY + node.y * sizeX + node.x] = std::make_pair(parent.y * sizeX + parent.x, cost);
    }
    else
        return false;

    return true;
}

bool JPS3DPlan::jump(const JPS3DNode_t &start)
{
    int dirx = start.dirx;
    int diry = start.diry;
    int dirz = start.dirz;

    // start cost is parent cost + # cells between parent and start node
    int parentInd = parents[start.z * sizeX * sizeY + start.y * sizeX + start.x].first;
    int pz = parentInd / (sizeX * sizeY);
    int py = (parentInd - pz * sizeX * sizeY) / sizeX;
    int px = parentInd - pz * sizeX * sizeY - py * sizeX;

    double cost = parents[start.z * sizeX * sizeY + start.y * sizeX + start.x].second + euclidean_dist(px, py, pz);

    JPS3DNode_t curr = start;
    closedSet[curr.z * sizeX * sizeY + curr.y * sizeX + curr.x] = true;

    while (true)
    {
        JPS3DNode_t n = curr;
        n.x += dirx;
        n.y += diry;
        n.z += dirz;
        cost += 1;

        if ((n.x > sizeX - 1 && dirx == 1) || (n.x < 0 && dirx == -1) ||
            (n.y > sizeY - 1 && diry == 1) || (n.y < 0 && diry == -1) ||
            (n.z > sizeZ - 1 && dirz == 1) || (n.z < 0 && dirz == -1))
        {
            return false;
        }

        if (_map[n.z * sizeX * sizeY + n.y * sizeX + n.x] == occupied_val)
        {
            return false;
        }

        if (n.x == destX && n.y == destY && n.z == destZ)
        {
            add_to_parents(n, curr, cost);
            add_to_queue(n.x, n.y, n.dirx, n.diry, cost);
            return true;
        }
    }
}
