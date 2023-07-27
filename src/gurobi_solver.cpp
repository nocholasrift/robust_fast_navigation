#include <chrono>

#include <robust_fast_navigation/solver_utils.h>
#include <robust_fast_navigation/gurobi_solver.h>

#include <drake/solvers/mathematical_program.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/geometry/optimization/hpolyhedron.h>

TrajectorySolver::TrajectorySolver(){

    _N = 2;
    _dt = 5;
    _interp_time = .01;
    forceFinalVA = false;

    _v_max = 1.8;
    _a_max = 3.0;
    _j_max = 5.0;

    _use_minvo = true;

    _m.set(GRB_StringAttr_ModelName, "Trajectory_Solver");

      M_pos_bs2mv_seg0 <<

         1.1023313949144333268037598827505,   0.34205724556666972091534262290224, -0.092730934245582874453361910127569, -0.032032766697130621302846975595457,
      -0.049683556253749178166501110354147,   0.65780347324677179710050722860615,   0.53053863760186903419935333658941,   0.21181027098212013015654520131648,
      -0.047309044211162346038612724896666,  0.015594436894155586093013710069499,    0.5051827557159349613158383363043,   0.63650059656260427054519368539331,
     -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,   0.18372189915240558222286892942066;

    M_pos_bs2mv_seg1 <<

    0.27558284872860833170093997068761,  0.085514311391667430228835655725561, -0.023182733561395718613340477531892, -0.0080081916742826553257117438988644,
        0.6099042761975865811763242163579,   0.63806904207840509091198555324809,   0.29959938009132258684985572472215,    0.12252106674808682651445224109921,
    0.11985166952332682033244282138185,   0.29187180223752445806795208227413,   0.66657381254229419731416328431806,    0.70176522577378930289881964199594,
    -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,    0.18372189915240558222286892942066;

    M_pos_bs2mv_rest <<

    0.18372189915240555446729331379174,  0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988,
    0.70176522577378919187651717948029,   0.66657381254229419731416328431806,   0.29187180223752384744528853843804,    0.11985166952332582113172065874096,
    0.11985166952332682033244282138185,   0.29187180223752445806795208227413,   0.66657381254229419731416328431806,    0.70176522577378930289881964199594,
    -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,    0.18372189915240558222286892942066;

    M_pos_bs2mv_seg_last2 <<

    0.18372189915240569324517139193631,  0.057009540927778309948870116841135, -0.015455155707597145742226985021261, -0.0053387944495218164764338553140988,
    0.70176522577378952494342456702725,   0.66657381254229453038107067186502,   0.29187180223752412500104469472717,    0.11985166952332593215402312125661,
        0.1225210667480875342816304396365,   0.29959938009132280889446064975346,   0.63806904207840497988968309073243,    0.60990427619758624810941682881094,
    -0.0080081916742826154270717964323012, -0.023182733561395621468825822830695,  0.085514311391667444106623463540018,    0.27558284872860833170093997068761;

    M_pos_bs2mv_seg_last <<

    0.18372189915240555446729331379174, 0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988,
    0.63650059656260415952289122287766,   0.5051827557159349613158383363043,  0.015594436894155294659469745965907,  -0.047309044211162887272337229660479,
    0.21181027098212068526805751389475,  0.53053863760186914522165579910506,   0.65780347324677146403359984105919,  -0.049683556253749622255710960416764,
    -0.032032766697130461708287185729205, -0.09273093424558248587530329132278,   0.34205724556666977642649385416007,     1.1023313949144333268037598827505;

    M_pos_bs2be_seg0 <<

            1.0000,    0.0000,   -0.0000,         0,
                 0,    1.0000,    0.5000,    0.2500,
                 0,   -0.0000,    0.5000,    0.5833,
                 0,         0,         0,    0.1667;

    M_pos_bs2be_seg1 <<

        0.2500,    0.0000,   -0.0000,         0,
        0.5833,    0.6667,    0.3333,    0.1667,
        0.1667,    0.3333,    0.6667,    0.6667,
                0,         0,         0,    0.1667;

    M_pos_bs2be_rest <<

        0.1667,    0.0000,         0,         0,
        0.6667,    0.6667,    0.3333,    0.1667,
        0.1667,    0.3333,    0.6667,    0.6667,
                0,         0,         0,    0.1667;

    M_pos_bs2be_seg_last2 <<

        0.1667,         0,   -0.0000,         0,
        0.6667,    0.6667,    0.3333,    0.1667,
        0.1667,    0.3333,    0.6667,    0.5833,
                0,         0,         0,    0.2500;

    M_pos_bs2be_seg_last <<

        0.1667,    0.0000,         0,         0,
        0.5833,    0.5000,         0,         0,
        0.2500,    0.5000,    1.0000,         0,
                0,         0,         0,    1.0000;


    setBasisConverter();
}

TrajectorySolver::~TrajectorySolver(){
    delete _env;
}

void TrajectorySolver::setBasisConverter(){

    bs2mv_matrices.clear();
    bs2be_matrices.clear();

    bs2mv_matrices.push_back(M_pos_bs2mv_seg0);
    bs2mv_matrices.push_back(M_pos_bs2mv_seg1);

    bs2be_matrices.push_back(M_pos_bs2be_seg0);
    bs2be_matrices.push_back(M_pos_bs2be_seg1);
    for(int segment = 0; segment < _N-4; segment++){
        bs2mv_matrices.push_back(M_pos_bs2mv_rest);
        bs2be_matrices.push_back(M_pos_bs2be_rest);
    }
    bs2be_matrices.push_back(M_pos_bs2be_seg_last2);
    bs2be_matrices.push_back(M_pos_bs2be_seg_last);

    bs2mv_matrices.push_back(M_pos_bs2mv_seg_last2);
    bs2mv_matrices.push_back(M_pos_bs2mv_seg_last);

    A_be << -0.008,  0.12, -0.6, 1.0,
             0.024, -0.24,  0.6,   0,
             -0.024,  0.12,    0,   0,
             0.008,     0,    0,   0;

    A_mv << -0.027533047834852528268356763874181,  0.27958193077329627662663824594347, -0.89245775759341949839864582827431,                  0.91437149799125659734933338484986,
             0.053434070143108883144122955854982, -0.47383959808521893819488468579948,   1.0504719372501213126014363297145, -0.000000000000000055511151231257827021181583404541,
            -0.053434070143108883144122955854982,  0.32767145406141430896695965202525, -0.31963121713109816646181116084335,                 0.085628502008743445639282754200394,
             0.027533047834852528268356763874181, -0.13341378674949164739871321216924,  0.16161703747439635225902065940318, -0.000000000000000012522535092207576212786079850048;
 

    return;
}

void TrajectorySolver::setVariables(){

    // polynomial coefficients for x,y,z axis
    std::vector<std::string> coeffs = {"ax", "ay", "az", "bx", "by", "bz", "cx", "cy", "cz", "dx", "dy", "dz"};

    _xs.clear();
    for(int segment = 0; segment < _N; segment++){

        std::vector<GRBVar> segment_vars;
        for(int i = 0; i < coeffs.size(); i++)
            segment_vars.push_back(_m.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, coeffs[i]+"_"+std::to_string(segment)));

        _xs.push_back(segment_vars);
    }

    return;
}

void TrajectorySolver::setObjective(){

    std::cout << "setting objective" << std::endl;
    GRBQuadExpr cost = 0;

    for(int segment = 0; segment < _N; segment++){
        std::vector<GRBLinExpr> J = {getJerk(segment, 0, 0), getJerk(segment, 0, 1), getJerk(segment, 0, 2)};
        cost += gurobiNorm(J);
    }

    _m.setObjective(cost, GRB_MINIMIZE);

    return;
}

void TrajectorySolver::setInterpTime(double interp_time){
    _interp_time = interp_time;
}

// set the polytope constraints for the control points (Ax <= b)
void TrajectorySolver::setPolys(const std::vector<Eigen::MatrixX4d>& hPolys){
    std::cout << "setting polytope constraints" << std::endl;

    _hPolys = hPolys;
    _N = _hPolys.size();

    // remove poly constraints from last iteration
    for(GRBConstr& poly_const : _poly_constraints)
        _m.remove(poly_const);

    _poly_constraints.clear();

    for(int segment = 0; segment < _N; segment++){

        std::vector<GRBLinExpr> cp0 = getCP0(segment);
        std::vector<GRBLinExpr> cp1 = getCP1(segment);
        std::vector<GRBLinExpr> cp2 = getCP2(segment);
        std::vector<GRBLinExpr> cp3 = getCP3(segment);

        int poly = segment;
        // for(int poly = 0; poly < _hPolys.size(); poly++){
            Eigen::MatrixXd A = _hPolys[poly].leftCols(_hPolys[poly].cols()-1);
            Eigen::VectorXd b = -_hPolys[poly].rightCols(1);

            std::vector<std::vector<GRBLinExpr> > Q_be {
                {cp0[0], cp1[0], cp2[0], cp3[0]},
                {cp0[1], cp1[1], cp2[1], cp3[1]},
                {cp0[2], cp1[2], cp2[2], cp3[2]}
            };
            

            // std::vector<std::vector<GRBLinExpr> > Q_bs = gurobiMatMul(Q_be, bs2be_matrices[segment].inverse());
            // std::vector<std::vector<GRBLinExpr> > Q_mv = gurobiMatMul(Q_bs, bs2mv_matrices[segment]);
            std::cout << "trying to invert" << std::endl;
            std::cout << A_mv << std::endl;
            Eigen::MatrixXd A_be2mv = A_be * A_mv.inverse();
            std::cout << "invert done" << std::endl;
            std::vector<std::vector<GRBLinExpr> > Q_mv = gurobiMatMul(Q_be, A_be2mv);
            
            std::vector<GRBLinExpr> cp0_mv = {Q_mv[0][0], Q_mv[1][0], Q_mv[2][0]};
            std::vector<GRBLinExpr> cp1_mv = {Q_mv[0][1], Q_mv[1][1], Q_mv[2][1]};
            std::vector<GRBLinExpr> cp2_mv = {Q_mv[0][2], Q_mv[1][2], Q_mv[2][2]};
            std::vector<GRBLinExpr> cp3_mv = {Q_mv[0][3], Q_mv[1][3], Q_mv[2][3]};

            std::vector<GRBLinExpr> Acp0, Acp1, Acp2, Acp3;
            if (_use_minvo){
                Acp0 = gurobiMatMul(A, cp0_mv);
                Acp1 = gurobiMatMul(A, cp1_mv);
                Acp2 = gurobiMatMul(A, cp2_mv);
                Acp3 = gurobiMatMul(A, cp3_mv);
            }else{
                Acp0 = gurobiMatMul(A, cp0);
                Acp1 = gurobiMatMul(A, cp1);
                Acp2 = gurobiMatMul(A, cp2);
                Acp3 = gurobiMatMul(A, cp3);
            }


            for(int i = 0; i < b.rows(); i++){
                std::string name0 = "Poly" + std::to_string(poly) + "_" + std::to_string(i)+"_segment" + std::to_string(segment) + "_cp0";
                std::string name1 = "Poly" + std::to_string(poly) + "_" + std::to_string(i)+"_segment" + std::to_string(segment) + "_cp1";
                std::string name2 = "Poly" + std::to_string(poly) + "_" + std::to_string(i)+"_segment" + std::to_string(segment) + "_cp2";
                std::string name3 = "Poly" + std::to_string(poly) + "_" + std::to_string(i)+"_segment" + std::to_string(segment) + "_cp3";

                _poly_constraints.push_back(_m.addConstr(Acp0[i] <= b[i], name0));
                _poly_constraints.push_back(_m.addConstr(Acp1[i] <= b[i], name1));
                _poly_constraints.push_back(_m.addConstr(Acp2[i] <= b[i], name2));
                _poly_constraints.push_back(_m.addConstr(Acp3[i] <= b[i], name3));
            }
        // }
    }

    return;
}

void TrajectorySolver::setInitialState(const Eigen::Matrix3d& PVA){
    std::cout << "setting up initial state" << std::endl;

    for(const GRBConstr& constraint : _initial_conditions)
        _m.remove(constraint);


    _initial_conditions.clear();
    for(int axis = 0; axis < 3; axis++){
        _initial_conditions.push_back(
            _m.addConstr(getPos(0,0,axis)==PVA(0,axis), "InitialPosAxis_"+std::to_string(axis))
        );
        _initial_conditions.push_back(
            _m.addConstr(getVel(0,0,axis)==PVA(1,axis), "InitialVelAxis_"+std::to_string(axis))
        );
        _initial_conditions.push_back(
            _m.addConstr(getAcc(0,0,axis)==PVA(2,axis), "InitialAccAxis_"+std::to_string(axis))
        );
    }
}

void TrajectorySolver::setFinalState(const Eigen::Matrix3d& PVA){
    std::cout << "setting up final state" << std::endl;

    for(const GRBConstr& constraint : _final_conditions)
        _m.remove(constraint);

    _final_conditions.clear();
    for(int axis = 0; axis < 3; axis++){
        _final_conditions.push_back(
            _m.addConstr(getPos(_N-1,_dt,axis)==PVA(0,axis), "FinalPosAxis_"+std::to_string(axis))
        );
        if (forceFinalVA){
            _final_conditions.push_back(
                _m.addConstr(getVel(_N-1,_dt,axis)==PVA(1,axis), "FinalVelAxis_"+std::to_string(axis))
            );
            _final_conditions.push_back(
                _m.addConstr(getAcc(_N-1,_dt,axis)==PVA(2,axis), "FinalAccAxis_"+std::to_string(axis))
            );
        }
    }
}

void TrajectorySolver::setDynamicConstraints(){

    std::cout << "setting up dynamic constraints" << std::endl;
    // set constraints for maximum state values [v,a,j]
    // also set continuity constraints for each
    for(int i = 0; i < _max_constraints.size(); i++)
        _m.remove(_max_constraints[i]);

    for(int i = 0; i < _dynamic_constraints.size(); i++)
        _m.remove(_dynamic_constraints[i]);

    _max_constraints.clear();
    _dynamic_constraints.clear();
    for(int segment = 0; segment < _N; segment++){
        for(int axis = 0; axis < 3; axis++){

            // maximum value constraints
            _max_constraints.push_back(
                _m.addConstr(getVel(segment, 0, axis) <= _v_max, "v_max_segment"+std::to_string(segment)+"_axis"+std::to_string(axis))
            );
            _max_constraints.push_back(
                _m.addConstr(getVel(segment, 0, axis) >= -_v_max, "v_min_segment"+std::to_string(segment)+"_axis"+std::to_string(axis))
            );

            _max_constraints.push_back(
                _m.addConstr(getAcc(segment, 0, axis) <= _a_max, "a_max_segment"+std::to_string(segment)+"_axis"+std::to_string(axis))
            );
            _max_constraints.push_back(
                _m.addConstr(getAcc(segment, 0, axis) >= -_a_max, "a_min_segment"+std::to_string(segment)+"_axis"+std::to_string(axis))
            );

            _max_constraints.push_back(
                _m.addConstr(getJerk(segment, 0, axis) <= _j_max, "j_max_segment"+std::to_string(segment)+"_axis"+std::to_string(axis))
            );
            _max_constraints.push_back(
                _m.addConstr(getJerk(segment, 0, axis) >= -_j_max, "j_min_segment"+std::to_string(segment)+"_axis"+std::to_string(axis))
            );

            // continuity constraints
            if (segment >= _N-1)
                continue;

            _dynamic_constraints.push_back(
                _m.addConstr(getPos(segment, _dt, axis)==getPos(segment+1,0,axis), 
                            "ContPos_t" + std::to_string(segment)+"_axis"+std::to_string(axis))
            );
            _dynamic_constraints.push_back(
                _m.addConstr(getVel(segment, _dt, axis)==getVel(segment+1,0,axis), 
                            "ContVel_t" + std::to_string(segment)+"_axis"+std::to_string(axis))
            );
            _dynamic_constraints.push_back(
                _m.addConstr(getAcc(segment, _dt, axis)==getAcc(segment+1,0,axis), 
                            "ContAcc_t" + std::to_string(segment)+"_axis"+std::to_string(axis))
            );
        }
    }
}

void TrajectorySolver::generateTrajectory(){
    int size = (int)(_N)*_dt / _interp_time;
    std::vector<Eigen::Vector3d> traj(size);

    double t = 0;
    int segment = 0;

    for(int segment = 0; segment < _N; segment++){
        std::vector<GRBLinExpr> cp0 = getCP0(segment);
        std::vector<GRBLinExpr> cp1 = getCP1(segment);
        std::vector<GRBLinExpr> cp2 = getCP2(segment);
        std::vector<GRBLinExpr> cp3 = getCP3(segment);

        Eigen::Vector3d eig_cp0(cp0[0].getValue(), cp0[1].getValue(), cp0[2].getValue());
        Eigen::Vector3d eig_cp1(cp1[0].getValue(), cp1[1].getValue(), cp1[2].getValue());
        Eigen::Vector3d eig_cp2(cp2[0].getValue(), cp2[1].getValue(), cp2[2].getValue());
        Eigen::Vector3d eig_cp3(cp3[0].getValue(), cp3[1].getValue(), cp3[2].getValue());

        std::cout << eig_cp0 << std::endl;
        std::cout << eig_cp1 << std::endl;
        std::cout << eig_cp2 << std::endl;
        std::cout << eig_cp3 << std::endl;
    }

    for(int i = 0; i < size; i++){
        t = t + _interp_time;
        if (t > _dt * (segment+1))
            segment = std::min(segment+1, _N-1);

        double posx = getPos(segment, t-segment*_dt, 0).getValue();
        double posy = getPos(segment, t-segment*_dt, 1).getValue();
        double posz = getPos(segment, t-segment*_dt, 2).getValue();

        traj[i] = Eigen::Vector3d(posx, posy, posz);
        std::cout << traj[i] << std::endl;
    }

    return;
}

bool TrajectorySolver::solve(){

    bool res = true;

    std::cout << "updating model" << std::endl;
    _m.update();
    std::cout << "optimizing" << std::endl;
    auto start = std::chrono::steady_clock::now();
    _m.optimize();
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "solved, elapsed time is: " << elapsed/1000.0 << std::endl;

    int status = _m.get(GRB_IntAttr_Status);
    if (status == GRB_OPTIMAL){
        std::cout << "optimal solution found" << std::endl;
        generateTrajectory();
    }
    else {
        res = false;
        if (status == GRB_INF_OR_UNBD){
            std::cout << "gurobi solution was unbounded or infeasible" << std::endl;
            _m.computeIIS();
            _m.write("/home/nick/catkin_ws/src/robust_fast_navigation/gurobi_dump/out.ilp");
        }
        else if(status == GRB_NUMERIC)
            std::cout << "gurobi encountered numerical issues, potentially infeasible / unbounded" << std::endl;
        else if(status == GRB_INTERRUPTED)
            std::cout << "gurobi interrupted by user" << std::endl;
    }

    return res;
}

GRBLinExpr TrajectorySolver::getPos(int segment, double t, int axis){
    GRBLinExpr pos = _xs[segment][axis]*t*t*t + _xs[segment][3+axis]*t*t + _xs[segment][6+axis]*t + _xs[segment][9+axis];
    return pos;
}

GRBLinExpr TrajectorySolver::getVel(int segment, double t, int axis){
    GRBLinExpr vel = 3*_xs[segment][axis]*t*t + 2*_xs[segment][3+axis]*t + _xs[segment][6+axis];
    return vel;
}

GRBLinExpr TrajectorySolver::getAcc(int segment, double t, int axis){
    GRBLinExpr acc = 6*_xs[segment][axis]*t + 2*_xs[segment][3+axis];
    return acc;
}

GRBLinExpr TrajectorySolver::getJerk(int segment, double t, int axis){
    GRBLinExpr jerk = 6*_xs[segment][axis];
    return jerk; 
}

// polynomial is At^3 + Bt^2 + Ct + D for t in [0,_dt]
GRBLinExpr TrajectorySolver::getA(int segment, int axis){
    return _xs[segment][axis];
}

// polynomial is At^3 + Bt^2 + Ct + D for t in [0,_dt]
GRBLinExpr TrajectorySolver::getB(int segment, int axis){
    return _xs[segment][3+axis];
}

// polynomial is At^3 + Bt^2 + Ct + D for t in [0,_dt]
GRBLinExpr TrajectorySolver::getC(int segment, int axis){
    return _xs[segment][6+axis];
}

// polynomial is At^3 + Bt^2 + Ct + D for t in [0,_dt]
GRBLinExpr TrajectorySolver::getD(int segment, int axis){
    return _xs[segment][9+axis];
}

// normalized polynomial is At^3 + Bt^2 + Ct + D for t in [0,1]
GRBLinExpr TrajectorySolver::getAn(int segment, int axis){
    return _xs[segment][axis] *_dt * _dt * _dt;
}

// normalized polynomial is At^3 + Bt^2 + Ct + D for t in [0,1]
GRBLinExpr TrajectorySolver::getBn(int segment, int axis){
    return _xs[segment][3+axis] *_dt * _dt;
}

// normalized polynomial is At^3 + Bt^2 + Ct + D for t in [0,1]
GRBLinExpr TrajectorySolver::getCn(int segment, int axis){
    return _xs[segment][6+axis] *_dt;
}

// normalized polynomial is At^3 + Bt^2 + Ct + D for t in [0,1]
GRBLinExpr TrajectorySolver::getDn(int segment, int axis){
    return _xs[segment][9+axis];
}

// Control points, code taken with <3 from FASTER: MIT-ACL GROUP

// cp0 is the initial position
std::vector<GRBLinExpr> TrajectorySolver::getCP0(int segment){
    std::vector<GRBLinExpr> cp = { getPos(segment, 0, 0), getPos(segment, 0, 1), getPos(segment, 0, 2) };
    return cp;
}

// cp1
std::vector<GRBLinExpr> TrajectorySolver::getCP1(int segment){ 
    GRBLinExpr cpx = (getCn(segment, 0) + 3 * getDn(segment, 0)) / 3;
    GRBLinExpr cpy = (getCn(segment, 1) + 3 * getDn(segment, 1)) / 3;
    GRBLinExpr cpz = (getCn(segment, 2) + 3 * getDn(segment, 2)) / 3;
    std::vector<GRBLinExpr> cp = { cpx, cpy, cpz };
    return cp;
}

// cp2 
std::vector<GRBLinExpr> TrajectorySolver::getCP2(int segment){
    GRBLinExpr cpx = (getBn(segment, 0) + 2 * getCn(segment, 0) + 3 * getDn(segment, 0)) / 3;
    GRBLinExpr cpy = (getBn(segment, 1) + 2 * getCn(segment, 1) + 3 * getDn(segment, 1)) / 3;
    GRBLinExpr cpz = (getBn(segment, 2) + 2 * getCn(segment, 2) + 3 * getDn(segment, 2)) / 3;
    std::vector<GRBLinExpr> cp = { cpx, cpy, cpz };
    return cp;
}

// cp3 is the final position
std::vector<GRBLinExpr> TrajectorySolver::getCP3(int segment){
    std::vector<GRBLinExpr> cp = { getPos(segment, _dt, 0), getPos(segment, _dt, 1), getPos(segment, _dt, 2) };
    return cp;
}

int main(){

    std::cout << "poly" << std::endl;
    Eigen::MatrixX4d poly(13,4);
	poly << -1,            0,            0,       -4.975,
            -0.000126426,     0.454825,            0,      -1.2389,
            0.00136563,     0.453043,            0,     -1.24004,
            0.000278955,    -0.455005,            0,    -0.761792,
            -0.00121247,    -0.453227,            0,    -0.760702,
                0.157664, -0.000971388,            0,     -1.21657,
                0.155856,    0.0114496,            0,     -1.22088,
                -0.14007,       0.4321,            0,     -1.04421,
                    1,            0,            0,      -10.175,
                    0,            1,            0,       -5.025,
                    0,           -1,            0,       -6.625,
                    0,            0,            -1,        -.1,
                    0,            0,            1,        -.1;

    std::cout << "poly 1" << std::endl;

    Eigen::MatrixX4d poly1(10,4);
	poly1 << -0.751462,    -0.563597, -4.15515e-05,      2.97297,
            -2.22045e-16,           -1,  5.29396e-23,       -8.375,
            -0.0050348,     0.176628, -0.000136769,    -0.465327,
            0.0231564,     0.197991, -0.000135347,    -0.614205,
                0.731436,     0.598495,  4.06844e-06,     -4.73764,
                    1,  2.77556e-16,  5.29396e-23,      -11.925,
            2.22045e-16,            1, -5.29396e-23,       -3.375,
                    -1, -2.77556e-16, -5.29396e-23,        0.175,
                    0,            0,            -1,        -.1,
                    0,            0,            1,        -.1;

    Eigen::MatrixX4d noZ1 = poly.topRows(poly.rows()-2);
    Eigen::MatrixX2d A1 = noZ1.leftCols(noZ1.cols()-2);
    std::cout << A1 << std::endl;
    Eigen::VectorXd b1 = -1*noZ1.rightCols(1);
    std::cout << b1 << std::endl;
    
    drake::geometry::optimization::HPolyhedron Hpoly1(A1, b1);
    drake::geometry::optimization::VPolytope Vpoly1(Hpoly1);
    Eigen::MatrixXd verts = Vpoly1.vertices();

    // std::cout << "verts of pol1 are\n" << verts.transpose() << std::endl;
    for(int i = 0; i < verts.cols(); i++){
        std::cout << "(" << verts.col(i).transpose()[0] << ", " << verts.col(i).transpose()[0]<< ")" << std::endl;
    }

    std::cout << "poly 2" << std::endl;

    Eigen::MatrixX4d poly2(8,4);
    poly2 <<        1,            0,            0,      -11.925,
                    -0.105388,     0.229673,  2.70733e-06,      1.06789,
                    0.000885633,     0.240744,  2.98777e-06,     0.396271,
                    -5.55112e-17,           -1,  2.06795e-24,       -9.975,
                            -1,            0,            0,        1.925,
                    5.55112e-17,            1, -2.06795e-24,       -1.625,
                    0,            0,            -1,        -.1,
                    0,            0,            1,        -.1;

    Eigen::Matrix3d initialPVA;
    initialPVA << 0,0,0,
                  0,0,0,
                  0,0,0;

    Eigen::Matrix3d finalPVA;
    finalPVA << 3,2,0,
                0,0,0,
                0,0,0;

    std::vector<Eigen::MatrixX4d> hPolys = {poly, poly1};//, poly2};

    std::cout << "initializing solver" << std::endl;
    // steps to run solver
    TrajectorySolver solver;
    std::cout << "done!" << std::endl;
    solver.setVariables();
    std::cout << "set variables" << std::endl;
    solver.setInitialState(initialPVA);
    std::cout << "set initial state" << std::endl;
    solver.setFinalState(finalPVA);
    std::cout << "set final state" << std::endl;
    solver.setDynamicConstraints();
    std::cout << "set constraints" << std::endl;
    solver.setPolys(hPolys);
    std::cout << "set polys" << std::endl;
    solver.setObjective();
    std::cout << "set objective" << std::endl;

    solver.solve();
    return 0;
}
