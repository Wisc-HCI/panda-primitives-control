#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <array>
#include <algorithm>
#include <math.h>
#include <nlopt.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;

class BSplineSurface{
    private:
        int k; // order of the B-Spline functions
        vector<vector<array<double,3>>> control_pts; // 2D (x,y,z) grid of control points
        int m; // # number of control points in u-direction 0-> m
        int n; // # number of control points in v-direction 0-> n
        vector<double> knots_u;
        vector<double> knots_v;

        array<double,3> u_dir_plane;
        array<double,3> v_dir_plane;

        // Recursive function for getting B-spline basis function values
        double getN(int i, int p, double x, vector<double> t);

        static double obj_bestplane(const std::vector<double> &x, std::vector<double> &grad, void *data);
        void calculate_plane_dirs();
        static Eigen::MatrixXd exp_map(Eigen::VectorXd w);
        
        
    public:
        BSplineSurface();
        
        // This creates a B-spline surface based on control pts
        // It is also called by loadSurface
        void initialize(int k, vector<vector<array<double,3>>> &control_pts);
        
        // Evaluates a parameterized point (U,V) and returns 3D location and normal vector
        void calculateSurfacePoint(double u, double v, array<double,3> &r, array<double,3> &n_hat, array<double,3> &r_u, array<double,3> &r_v);
        
        // Read and write surfaces as CSV files
        void loadSurface(string filename);
        
        // TODO: Actually write this!
        void writeSurface(string filename);
};