#include <ros/ros.h>
#include <Eigen/Core>
#include <boost/bind.hpp>

#include <trajectory_msgs/JointTrajectory.h>

#include <faster/solver.hpp>

#include <robust_fast_navigation/utils.h>
#include <robust_fast_navigation/corridor.h>
#include <robust_fast_navigation/SolveFromState.h>

class solver_service{
public:
    solver_service(ros::NodeHandle& nh){

        double limits[3] = {1.2, 1.2,4};

        solver.setN(6);
        solver.createVars();
        solver.setDC(.05);
        solver.setBounds(limits);
        solver.setForceFinalConstraint(true);
        solver.setFactorInitialAndFinalAndIncrement(1,10,1.0);
        solver.setThreads(0);
        solver.setWMax(4.0);
        solver.setVerbose(0);

        service = nh.advertiseService("solve_from_state", &solver_service::solveFromSolverStateService, this);
    }
    
    ~solver_service(){

    }

    bool solveFromSolverStateService(
        robust_fast_navigation::SolveFromState::Request& req,  
        robust_fast_navigation::SolveFromState::Response& res){

        ROS_INFO("solving from solver state");

        robust_fast_navigation::SolverState solver_state = req.state;
        
        std::vector<Eigen::MatrixX4d> polys;
        corridor::msgToCorridor(polys, solver_state.polys);

        // ROS_INFO_STREAM(polys[0]);
        state initialState;
        state finalState;

        initialState.setPos(solver_state.initialPVA.positions[0], solver_state.initialPVA.positions[1], solver_state.initialPVA.positions[2]);
        initialState.setVel(solver_state.initialPVA.velocities[0], solver_state.initialPVA.velocities[1], solver_state.initialPVA.velocities[2]);
        initialState.setAccel(solver_state.initialPVA.accelerations[0], solver_state.initialPVA.accelerations[1], solver_state.initialPVA.accelerations[2]);
        initialState.setJerk(0,0,0);

        finalState.setPos(solver_state.finalPVA.positions[0], solver_state.finalPVA.positions[1], solver_state.finalPVA.positions[2]);
        finalState.setVel(solver_state.finalPVA.velocities[0], solver_state.finalPVA.velocities[1], solver_state.finalPVA.velocities[2]);
        finalState.setAccel(solver_state.finalPVA.accelerations[0], solver_state.finalPVA.accelerations[1], solver_state.initialPVA.accelerations[2]);
        finalState.setJerk(0,0,0);

        solver.setX0(initialState);
        solver.setXf(finalState);
        solver.setPolytopes(polys);

        res.success = solver.genNewTraj();

        if (res.success == 1){
            solver.fillX();

            trajectory_msgs::JointTrajectory traj = convertTrajToMsg(solver.X_temp_, .05, "");
            res.trajectory = traj;
        }

        if (res.success == 0){
            bool is_in = isInPoly(polys[0], 
                                    Eigen::Vector2d(
                                        solver_state.initialPVA.positions[0], 
                                        solver_state.initialPVA.positions[1]));

            ROS_INFO("init state is in poly: %d", is_in);
        }

        return true;
    }

private:
    SolverGurobi solver;
    ros::ServiceServer service;
};


int main(int argc, char** argv){
    ros::init(argc, argv, "solve_from_state_server");
    ros::NodeHandle nh;
    
    solver_service solver_service(nh);

    ros::spin();
    return 0;
}
