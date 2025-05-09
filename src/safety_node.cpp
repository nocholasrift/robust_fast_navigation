#include <costmap_2d/costmap_2d_ros.h>
#include <robust_fast_navigation/map_util.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>

class SafetyMonitor
{
   public:
    SafetyMonitor(ros::NodeHandle &nh)
    {
        _is_costmap_started = false;
        _is_new_traj        = false;

        _safetyTimer = nh.createTimer(ros::Duration(0.1), &SafetyMonitor::safetyLoop, this);

        _trajSub = nh.subscribe("raw_trajectory", 1, &SafetyMonitor::trajCallback, this);

        _safeTrajPub =
            nh.advertise<trajectory_msgs::JointTrajectory>("reference_trajectory", 1);
        _trajVizPub = nh.advertise<visualization_msgs::Marker>("/MINCO_path", 1);

        _occ_grid = std::make_unique<map_util::occupancy_grid_t>();
    }

    void spin()
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        _costmap = std::make_unique<costmap_2d::Costmap2DROS>("global_costmap", tfBuffer);
        _costmap->start();

        _is_costmap_started = true;

        _occ_grid = std::make_unique<map_util::occupancy_grid_t>(*_costmap->getCostmap());

        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::waitForShutdown();
    }

    void trajCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
    {
        _is_new_traj = true;
        _traj        = *msg;
    }

    void safetyLoop(const ros::TimerEvent &)
    {
        if (!_is_costmap_started) return;

        _costmap->updateMap();
        _occ_grid->update(*_costmap->getCostmap());

        if (_traj.points.size() == 0) return;

        trajectory_msgs::JointTrajectory safe_traj = _traj;
        safe_traj.points.clear();

        // check if trajectory is safe
        bool is_safe = true;
        for (trajectory_msgs::JointTrajectoryPoint &point : _traj.points)
        {
            if (_occ_grid->is_occupied(point.positions[0], point.positions[1], "inflated") &&
                _occ_grid->get_signed_dist(point.positions[0], point.positions[1]) <
                    -_occ_grid->get_resolution() / 4)
            {
                ROS_WARN("Trajectory is in occupied space, trimming!");
                is_safe = false;
                break;
            }
            else
                safe_traj.points.push_back(point);
        }

        if (is_safe && !_is_new_traj)
        {
            ROS_INFO("Trajectory is safe, no need to trim!");
            return;
        }

        _is_new_traj = false;

        visualizeTraj(safe_traj);
        _safeTrajPub.publish(safe_traj);
    }

    void visualizeTraj(const trajectory_msgs::JointTrajectory &sentTraj)
    {
        if (sentTraj.points.size() > 0)
        {
            visualization_msgs::Marker arclenmsg;
            arclenmsg.header.frame_id    = "map";
            arclenmsg.header.stamp       = ros::Time::now();
            arclenmsg.ns                 = "planTraj";
            arclenmsg.id                 = 80;
            arclenmsg.action             = visualization_msgs::Marker::ADD;
            arclenmsg.type               = visualization_msgs::Marker::LINE_STRIP;
            arclenmsg.scale.x            = .1;
            arclenmsg.pose.orientation.w = 1;

            for (const trajectory_msgs::JointTrajectoryPoint &p : sentTraj.points)
            {
                std_msgs::ColorRGBA color_msg;
                color_msg.r = 0.0;
                color_msg.g = 1.0;
                color_msg.b = 0.0;
                color_msg.a = 1.0;
                arclenmsg.colors.push_back(color_msg);

                geometry_msgs::Point point_msg;
                point_msg.x = p.positions[0];
                point_msg.y = p.positions[1];
                point_msg.z = p.positions[2];
                arclenmsg.points.push_back(point_msg);
            }

            _trajVizPub.publish(arclenmsg);
        }
    }

   private:
    std::unique_ptr<costmap_2d::Costmap2DROS> _costmap;
    std::unique_ptr<map_util::occupancy_grid_t> _occ_grid;

    ros::Timer _safetyTimer;

    ros::Subscriber _trajSub;
    ros::Publisher _safeTrajPub;
    ros::Publisher _trajVizPub;

    trajectory_msgs::JointTrajectory _traj;

    bool _is_costmap_started;
    bool _is_new_traj;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robust_planner");
    ros::NodeHandle nh;

    SafetyMonitor safety_monitor(nh);
    safety_monitor.spin();
    return 0;
}
