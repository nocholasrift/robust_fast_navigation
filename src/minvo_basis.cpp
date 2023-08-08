#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ginac/ginac.h>

using namespace GiNaC;

class BasisConverter{
public:
    BasisConverter(){
        // Digits = 20;
        // initialize the bezier basis matrix on the interval [0,1]
        A_bezier = Eigen::MatrixXd(4,4);
        A_bezier << -1, 3, -3, 1,
                    3, -6, 3, 0,
                    -3, 3, 0, 0,
                    1, 0, 0, 0;

        // initialize the minvo basis matrix on the interval [-1,1]
        A_minvo = Eigen::MatrixXd(4,4);
        A_minvo <<  -0.43020000000000,    0.45680000000000,   -0.02700000000000,    0.00040000000000,
                    0.83490000000000,   -0.45680000000000,   -0.79210000000000,    0.49960000000000,
                    -0.83490000000000,   -0.45680000000000,    0.79210000000000,    0.49960000000000,
                    0.43020000000000,    0.45680000000000,    0.02700000000000,    0.00040000000000;
        
        // roots of lambda polynomial
        roots_lambda = {
            Eigen::Vector2d(0.0309000000000, 1.000000000000),
            Eigen::Vector2d(-1.000000000000, 0.7735000000000),
            Eigen::Vector2d(-0.7735000000000, 1.000000000000),
            Eigen::Vector2d(-1.000000000000, -0.0309000000000)
        };

    }

    Eigen::MatrixXd get_bernstein_on_interval(const std::vector<double>& int2){

        std::vector<double> int1 = {0,1};
        
        // Initialize GiNaC symbols
        symbol t("t");

        // Define transformation from interval 1 to interval 2
        ex u = (int1[1]-int1[0])/(int2[1]-int2[0])*(t-int2[0])+int1[0];

        // Define Bernstein basis polynomials
        ex B1 = -(u - 1) * (u - 1) * (u - 1);
        ex B2 = 3 * u * (u - 1) * (u - 1);
        ex B3 = -3 * u * u * (u - 1);
        ex B4 = u * u * u;

        ex B1_poly = B1.expand();
        ex B2_poly = B2.expand();
        ex B3_poly = B3.expand();
        ex B4_poly = B4.expand();


        std::vector<ex> B_coeffs = {B1_poly, B2_poly, B3_poly, B4_poly};
        Eigen::MatrixXd A_conv(4,4);

        for(int i = 0; i < B_coeffs.size(); ++i){
            ex B = B_coeffs[i];
            for (int deg = 3; deg >= 0; --deg)
                A_conv(i,3-deg) = ex_to<numeric>(B.coeff(t,deg)).to_double();
        }

        return A_conv;
    }

    Eigen::MatrixXd get_minvo_on_interval(const std::vector<double>& int2){
        std::vector<double> int1 = {-1,1};

        symbol t("t");
        ex u = (int1[1]-int1[0])/(int2[1]-int2[0])*(t-int2[0])+int1[0];

        std::vector<ex> T = {
            u*u*u,
            u*u,
            u,
            1
        };

        Eigen::MatrixXd A_conv(4,4);
        std::vector<ex> polynomials;
        for (int row = 0; row < A_minvo.rows(); ++row){
            polynomials.push_back(0);
            for(int i = 0; i < T.size(); ++i)
                polynomials[row] += A_minvo(row,i)*T[i];

            for(int deg = 3; deg >= 0; --deg)
                A_conv(row,3-deg) = ex_to<numeric>(polynomials[row].expand().coeff(t,deg)).to_double();
        }

        return A_conv;
    }


protected:
    Eigen::MatrixXd A_minvo, A_bezier;
    std::vector<Eigen::Vector2d> roots_lambda;
};



int main() {
    BasisConverter bc;

    Eigen::MatrixXd poly(3,4);
    poly << 0.7160,   -0.1370,   -0.4618,    0.3999,
            0.3365,    0.8679,   -0.2322,    0.2508,
            0.4907,   -0.0914,   -0.8137,    0.4999;

    Eigen::MatrixXd bezier_matrix = bc.get_bernstein_on_interval({0,.695378});
    Eigen::MatrixXd minvo_matrix = bc.get_minvo_on_interval({0,.695378});
    Eigen::MatrixXd converted = poly*minvo_matrix.inverse();

    std::cout << bezier_matrix << std::endl;
    std::cout << "********************" << std::endl;
    std::cout << minvo_matrix << std::endl;
    // std::cout << converted << std::endl;
    return 0;
}
