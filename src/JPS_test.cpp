// File to test the JPS algorithm in 3D using the octomap library and JPS3D
#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <robust_fast_navigation/JPS3D.h>

std::shared_ptr<octomap::OcTree> octree;
JPS3DPlan jps;
bool ran = false;

void mapcb(const octomap_msgs::Octomap::ConstPtr &msg)
{
    octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
    jps.set_map(octree);
    ran = true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "jps_test");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/octomap_binary", 1, mapcb);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        if (!ran)
            continue;

        jps.set_start(10,-10,25);

        double resolution = octree->getResolution();
        double minx, miny, minz, maxx, maxy, maxz;
        octree->getMetricMin(minx,miny,minz);
        octree->getMetricMax(maxx,maxy,maxz);

        minx = minx - resolution/2;
        miny = miny - resolution/2;
        minz = minz - resolution/2;
        maxx = maxx + resolution/2;
        maxy = maxy + resolution/2;
        maxz = maxz + resolution/2;

        // // print out size of octree in each dimension
        size_t sizeX = static_cast<size_t>((maxx-minx)/resolution);
        size_t sizeY = static_cast<size_t>((maxy-miny)/resolution);
        size_t sizeZ = static_cast<size_t>((maxz-minz)/resolution);

        // std::cout << "Num Nodes in each dim: " << numLeafNodesX << ", " << numLeafNodesY << ", " << numLeafNodesZ << std::endl;
        
        // // print the bounding box coordinates of the octree

        // // average the min and max to get the center of the octree
        // double cx = (minx + maxx) / 2;
        // double cy = (miny + maxy) / 2;
        // double cz = (minz + maxz) / 2;
        // std::cout << "Center: " << cx << ", " << cy << ", " << cz << std::endl;

        // Create an OcTreeKey from 3D coordinates
        octomap::OcTreeKey key;


        double nx=minx+resolution/2, ny=miny+resolution/2, nz=minz+resolution/2;
        // double nx=0, ny=0, nz=0;
        int i = 0;

        key = octree->coordToKey(octomap::point3d(nx, ny, nz));
        std::cout << i++ << " Adjacent Voxel Cell Coordinate (Key): "
                    << key[0] << ", " << key[1] << ", " << key[2] << std::endl;

         octomap::point3d adjacent_coordinates = octree->keyToCoord(key);
        nx = adjacent_coordinates.x();
        ny = adjacent_coordinates.y();
        nz = adjacent_coordinates.z();

        std::cout << "World Coordinates of Adjacent Voxel Cell: "
                    << nx << ", " << ny << ", " << nz << std::endl;

        while(true){

            if (nx < minx || nx > maxx || ny < miny || ny > maxy || nz < minz || nz > maxz){
                std::cout << "out of bounds" << std::endl;
                std::cout << "bounds of map are: " << minx << ", " << maxx << ", " << miny << ", " << maxy << ", " << minz << ", " << maxz << std::endl;
                std::cout << sizeX << " " << sizeY << " " << sizeZ << std::endl;
                std::cout << "resolution: " << resolution << std::endl;

                std::cout << i++ << " Adjacent Voxel Cell Coordinate (Key): "
                    << key[0] << ", " << key[1] << ", " << key[2] << std::endl;

                std::cout << "World Coordinates of Adjacent Voxel Cell: "
                    << nx << ", " << ny << ", " << nz << std::endl;

                break;
            }

            key = octree->coordToKey(octomap::point3d(nx, ny, nz));
            // Increment the X dimension (move to the adjacent voxel cell in the X direction)
            key[0]++;
            // Print the new key (adjacent voxel cell in the X direction)
            // std::cout << i++ << " Adjacent Voxel Cell Coordinate (Key): "
            //         << key[0] << ", " << key[1] << ", " << key[2] << std::endl;

            // Convert the modified key back to world coordinates
            octomap::point3d adjacent_coordinates = octree->keyToCoord(key);
            nx = adjacent_coordinates.x();
            ny = adjacent_coordinates.y();
            nz = adjacent_coordinates.z();

            // Print the world coordinates corresponding to the adjacent key
            // std::cout << "World Coordinates of Adjacent Voxel Cell: "
            //         << nx << ", " << ny << ", " << nz << std::endl;

        }
            
        

        break;
    }

    return 0;
}
