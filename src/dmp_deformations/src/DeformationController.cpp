#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <ctime>
#include <ratio>
#include <chrono>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <relaxed_ik/EEPoseGoals.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <panda_ros_msgs/HybridPose.h>

#include "BSplineSurface.h"
#include "nlopt.hpp"
#include <iomanip>
#include <thread>

// Force Dimension
#include "dhdc.h"

#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace std::chrono;

class DeformationController{
    private:
        array<double, 3> actual_pos;
        array<double, 4> actual_orientation;
        tf2_ros::Buffer tfBuffer;
      
        // For recording
        std::ofstream outputfile;
        double x, y, z, fx, fy, fz;

        bool replay_active = false;

        // For the NL optimization
        typedef struct {
            double x, y, z;
            BSplineSurface surface;
        } opt_data;

        // General robotics mathematical operations
        array<double,3> crossProduct(array<double,3> x, array<double,3> y);
        array<double,4> slerp(array<double,4> q_start, array<double,4> q_end,double x);
        array<double,3> vectorIntoConstraintFrame(double x, double y, double z, double qx, double qy, double qz, double qw);
        array<double,3> vectorOutOfConstraintFrame(double x, double y, double z, double qx, double qy, double qz, double qw);
        void rotationToQuaternion(array<double,3> x_hat, array<double,3> y_hat, array<double,3>z_hat, array<double,4> &q_out);

        void check_for_haptic_cue(double var_x, double var_y, double var_z);
        static double obj_closest_surface(const std::vector<double> &x, std::vector<double> &grad, void *data);
        void getClosestParams(double x, double y, double z, double &u, double&v, BSplineSurface surface);
        void bidirectional_checker(array<double,3> &vec, array<double,3> &prev_vec);
        void calculateDMPTransition(double ii, double &transition_x, double &transition_y, double &transition_z, double &transition_def_x, double &transition_def_y, double &transition_def_z, double x_imp, double y_imp, double z_imp, double x_def, double y_def, double z_def, vector<array<double,3>> selections,vector<array<double,7>> starting_points, vector<string> surfaces);
        void calculateNearestContact(int ii,vector<array<double,3>> selections,vector<array<double,7>> starting_points,vector<array<double,7>> attractor_points,bool &dmp_x_limiting,bool &dmp_y_limiting,bool &dmp_z_limiting,double &dmp_x_collision, double &dmp_y_collision, double &dmp_z_collision);
        array<double,3> deformationScaling(array<double,3> &rotated_deformation, double var_x, double var_y, double var_z, geometry_msgs::Vector3 selection, double x, double y, double collision_scaling);
        array<double,3> getFirstSurfaceVelocity(array<double,7> attractor_point, array<double,7> starting_point,double dmp_u,double dmp_v, array<double,3> r_u, array<double,3> r_v,double &ddu, double& ddv);
        array<double,3> map_deformation_input(int method, double dmp_fx,double dmp_fy,double dmp_fz,double dx,double dy, double dz, array<double,4> q_out);
        void forceOnloading(int ii, geometry_msgs::Vector3 selection, vector<array<double,7>> starting_points, vector<array<double,7>> attractor_points, vector<vector<array<double,7>>> dmps, BSplineSurface curr_surface, string surface_name, ros::Publisher &selection_vector_pub, ros::Publisher &constraint_frame_pub, ros::Publisher wrench_goal_pub, ros::Publisher hybrid_pub,double &ddu, double& ddv);
        void readDemo(vector<vector<array<double,7>>> &dmps,vector<vector<array<double,7>>> &dmps_reverse,vector<array<double,3>> &selections,vector<array<double,7>> &starting_points,vector<array<double,7>> &attractor_points, vector<string> &surfaces, vector<vector<array<double,3>>> &variance_dmp,vector<string> &preactions);
        void replay_demo(ros::NodeHandle n);
        void actualPose(geometry_msgs::Pose pose);
        void readPandaForces(geometry_msgs::Wrench wrench);
        bool surfaceBidirectional(string surface);
    
    public:
        array<double, 3> inputDevice_velocity;
        bool haptic_cue_increased = false;
        bool haptic_cue_decreased = false;
        int haptic_cue_count = 0;
        double dmp_fx, dmp_fy, dmp_fz;
        //haptic cue related
        double prev_var_x, prev_var_y, prev_var_z;
        bool var_x_changing,var_y_changing,var_z_changing;
        string trajectoryFile;

        ros::Publisher fd_publisher;

        DeformationController();
        DeformationController(string file);

        // Run the deformation controller and replay
        int run_deformation_controller(int argc, char **argv);

        // Functions specific to the particular input device
        virtual int init_inputDevice() = 0;
        virtual array<double, 3> getInputDeviceVelocity() = 0;
        virtual void run_zero_displacement_controller() = 0;
};

// Default Constructor
// TODO: override with filename for learned behavior

DeformationController::DeformationController(){
    inputDevice_velocity = {0.0, 0.0, 0.0};
    prev_var_x = 0.0; prev_var_y = 0.0; prev_var_z = 0.0;
    var_x_changing = false; var_y_changing = false; var_z_changing = false;
    trajectoryFile = "learneddmp.csv";
}

DeformationController::DeformationController(string file){
    inputDevice_velocity = {0.0, 0.0, 0.0};
    prev_var_x = 0.0; prev_var_y = 0.0; prev_var_z = 0.0;
    var_x_changing = false; var_y_changing = false; var_z_changing = false;
    if(file==""){
        trajectoryFile = "learneddmp.csv";
    }
    else{
        trajectoryFile = file;
    }
}


////////////////////////////////
// General robot utility fxns //
////////////////////////////////

/**
* Calculates the cross product between 2 3-D vectors
*/
array<double,3> DeformationController::crossProduct(array<double,3> x, array<double,3> y){
    array<double,3> return_vec;
    return_vec[0]=x[1]*y[2]-x[2]*y[1];
    return_vec[1]=x[2]*y[0]-x[0]*y[2];
    return_vec[2]=x[0]*y[1]-x[1]*y[0];
    return return_vec;
}

/**
* Orientation interpolation
* based on https://en.wikipedia.org/wiki/Slerp
* interpolation, x, is 0 to 1 bounded
*/
array<double,4> DeformationController::slerp(array<double,4> q_start, array<double,4> q_end,double x){
    array<double,4> q_interp;
    
    // Make sure quaterions are properly normalized
    double mag_q_start = sqrt(q_start[0]*q_start[0]+q_start[1]*q_start[1]+q_start[2]*q_start[2]+q_start[3]*q_start[3]);
    q_start[0] = q_start[0]/mag_q_start; q_start[1] = q_start[1]/mag_q_start;
    q_start[2] = q_start[2]/mag_q_start; q_start[3] = q_start[3]/mag_q_start;

    double mag_q_end = sqrt(q_end[0]*q_end[0]+q_end[1]*q_end[1]+q_end[2]*q_end[2]+q_end[3]*q_end[3]);
    q_end[0] = q_end[0]/mag_q_end; q_end[1] = q_end[1]/mag_q_end;
    q_end[2] = q_end[2]/mag_q_end; q_end[3] = q_end[3]/mag_q_end;
    
    // Compute the cosine of the angle between the two vectors.
    // dot product for quaternions is normal vector inner product
    double dot = q_start[0]*q_end[0]+q_start[1]*q_end[1]+q_start[2]*q_end[2]+q_start[3]*q_end[3];

    // make sure the shorter path is chosen
    if (dot < 0.0f) {
        q_end[0] = -q_end[0]; q_end[1] = -q_end[1]; q_end[2] = -q_end[2]; q_end[3] = -q_end[3];
        dot = -dot;
    }

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
        // for close vectors, linear interpolation
        q_interp[0] = q_start[0]+x*(q_end[0]-q_start[0]);
        q_interp[1] = q_start[1]+x*(q_end[1]-q_start[1]);
        q_interp[2] = q_start[2]+x*(q_end[2]-q_start[2]);
        q_interp[3] = q_start[3]+x*(q_end[3]-q_start[3]);
        
        double mag_q_interp = sqrt(q_interp[0]*q_interp[0]+q_interp[1]*q_interp[1]+q_interp[2]*q_interp[2]+q_interp[3]*q_interp[3]);
        q_interp[0] = q_interp[0]/mag_q_interp; q_interp[1] = q_interp[1]/mag_q_interp;
        q_interp[2] = q_interp[2]/mag_q_interp; q_interp[3] = q_interp[3]/mag_q_interp;
        return q_interp;
    }

    double theta_0 = acos(dot);
    double theta = theta_0*x;
    double sin_theta = sin(theta);
    double sin_theta_0 = sin(theta_0);

    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    double s1 = sin_theta / sin_theta_0;

    q_interp[0] = s0*q_start[0]+s1*q_end[0];
    q_interp[1] = s0*q_start[1]+s1*q_end[1];
    q_interp[2] = s0*q_start[2]+s1*q_end[2];
    q_interp[3] = s0*q_start[3]+s1*q_end[3];

    return q_interp;
}

/**
* Rotates a vector into a frame (represented as a quaternion)
*/
array<double,3> DeformationController::vectorIntoConstraintFrame(double x, double y, double z, double qx, double qy, double qz, double qw){
    // Make sure the quaternion is normalized (otherwise, there will be skew)
    double mag = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qx = qx/mag; qy = qy/mag; qz = qz/mag; qw = qw/mag;

    // Calculate the rotation matrix
    double r11=1.0-2*qy*qy-2*qz*qz;
    double r12=2*qx*qy-2*qz*qw;
    double r13=2*qx*qz+2*qy*qw;
    double r21=2*qx*qy+2*qz*qw;
    double r22=1.0-2*qx*qx-2*qz*qz;
    double r23=2*qy*qz-2*qx*qw;
    double r31=2*qx*qz-2*qy*qw;
    double r32=2*qy*qz+2*qx*qw;
    double r33=1.0-2*qx*qx-2*qy*qy;

    // Rotate the vector out of the frame
    // by multiplying by the transpose of
    // the rotation matrix
    array<double,3> rotated_vector;
    rotated_vector[0] = r11*x+r21*y+r31*z;
    rotated_vector[1] = r12*x+r22*y+r32*z;
    rotated_vector[2] = r13*x+r23*y+r33*z;
    return rotated_vector;
}

/**
* Rotates a vector OUT OF a frame (represented as a quaternion)
*/
array<double,3> DeformationController::vectorOutOfConstraintFrame(double x, double y, double z, double qx, double qy, double qz, double qw){
    // Make sure the quaternion is normalized (otherwise, there will be skew)
    double mag = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qx = qx/mag; qy = qy/mag; qz = qz/mag; qw = qw/mag;

    // Calculate the rotation matrix
    double r11=1.0-2*qy*qy-2*qz*qz;
    double r12=2*qx*qy-2*qz*qw;
    double r13=2*qx*qz+2*qy*qw;
    double r21=2*qx*qy+2*qz*qw;
    double r22=1.0-2*qx*qx-2*qz*qz;
    double r23=2*qy*qz-2*qx*qw;
    double r31=2*qx*qz-2*qy*qw;
    double r32=2*qy*qz+2*qx*qw;
    double r33=1.0-2*qx*qx-2*qy*qy;

    // Rotate the vector via the rotation matrix
    array<double,3> rotated_vector;
    rotated_vector[0] = r11*x+r12*y+r13*z;
    rotated_vector[1] = r21*x+r22*y+r23*z;
    rotated_vector[2] = r31*x+r32*y+r33*z;
    return rotated_vector;
}

/**
* This function takes a rotation matrix expressed as 3 column vectors and creates
* a quaternion of the form x,y,z,w
*/
void DeformationController::rotationToQuaternion(array<double,3> x_hat, array<double,3> y_hat, array<double,3>z_hat, array<double,4> &q_out){
    // Using simple (non-optimal) formulation found here: https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
    // page 2

    double trace = x_hat[0] + y_hat[1] + z_hat[2];
    double k;

    // Positive trace - W is largest
    if(trace > 0.0){
        k = 0.5/(sqrt(1.0+trace));
        q_out = {k*(z_hat[1]-y_hat[2]), k*(x_hat[2]-z_hat[0]), k*(y_hat[0]-x_hat[1]), 0.25/k};
    }

    // X is largest
    else if ((x_hat[0]>y_hat[1]) && (x_hat[0]>z_hat[2])){
        k = 0.5/(sqrt(1.0+x_hat[0]-y_hat[1]-z_hat[2]));
        q_out = {0.25/k, k*(x_hat[1]+y_hat[0]), k*(x_hat[2]+z_hat[0]), k*(z_hat[1]-y_hat[2])};
    }

    // Y is largest
    else if (y_hat[1]>z_hat[2]){
        k = 0.5/(sqrt(1.0+y_hat[1]-x_hat[0]-z_hat[2]));
        q_out = {k*(x_hat[1]+y_hat[0]), 0.25/k, k*(y_hat[2]+z_hat[1]), k*(x_hat[2]-z_hat[0])};
    }

    // Z is largest
    else{
        k = 0.5/(sqrt(1.0+z_hat[2]-x_hat[0]-y_hat[1]));
        q_out = {k*(x_hat[2]+z_hat[0]), k*(y_hat[2]+z_hat[1]), 0.25/k, k*(y_hat[0]-x_hat[1])};
    }

    // Make sure it is normalized
    double mag = sqrt(q_out[0]*q_out[0]+q_out[1]*q_out[1]+q_out[2]*q_out[2]+q_out[3]*q_out[3]);
    q_out[0] = -q_out[0]/mag; q_out[1] = -q_out[1]/mag; q_out[2] = -q_out[2]/mag; q_out[3] = q_out[3]/mag;
}

////////////////////////////
// DEFORMATION FUNCTIONS  //
////////////////////////////

bool DeformationController::surfaceBidirectional(string surface){
    if(surface=="cowling4"){
        return false;
    }

    else if(surface=="layup2"){
        return true;
    }

    else if(surface=="fastener1"){
        return false;
    }

    else{
        return false;
    }
}

/**
* Checks whether a haptic cue should be issues based on a changing variance
*/
void DeformationController::check_for_haptic_cue(double var_x, double var_y, double var_z){
    if(var_x != prev_var_x && !var_x_changing)
    {
        var_x_changing = true;
        if(var_x-prev_var_x>0.0){
            // increasing range -> push user in
            haptic_cue_increased = true;
        }

        else{
            // decreasing range -> push user out
            haptic_cue_decreased = true;
        }
    }

    else if(var_x==prev_var_x){
        var_x_changing=false;
    }

    prev_var_x = var_x; prev_var_y = var_y; prev_var_z = var_z;

}

// Next 3 pieces are for surface optimization
double DeformationController::obj_closest_surface(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    opt_data *d = (opt_data *) data;
    array<double,3> r = {0.0, 0.0, 0.0};
    d->surface.calculateSurfacePointOnly(x[0],x[1],r);
    
    return (r[0]-d->x)*(r[0]-d->x)+(r[1]-d->y)*(r[1]-d->y)+(r[2]-d->z)*(r[2]-d->z);
}

void DeformationController::getClosestParams(double x, double y, double z, double &u, double&v, BSplineSurface surface){
    // Find the closest point on the surface using NL-opt
    nlopt::opt opt(nlopt::LN_COBYLA, 2);
    
    // Bounds of surface are 0 to 1 in U,V directions
    std::vector<double> lb(2);
    lb[0] = 0.0; lb[1] = 0.0;
    std::vector<double> ub(2);
    ub[0] = 1.0; ub[1] = 1.0;
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    // Initial guess for parameters U,V
    std::vector<double> params(2);
    params[0] = u; params[1] = v;
    double minf;

    opt_data data;
    data.x=x; data.y=y; data.z=z;
    data.surface = surface;

    opt.set_min_objective(obj_closest_surface,&data);
    opt.set_xtol_rel(1e-2);
    opt.set_maxeval(20);

    try{
        nlopt::result result = opt.optimize(params, minf);
        
        // Output new values if found
        u = params[0]; v=params[1];
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }
}

/**
* TODO: fill this out
*/
void DeformationController::bidirectional_checker(array<double,3> &vec, array<double,3> &prev_vec){
    // Check if velocity should be flipped (tool goes both ways)
    // TODO: change to a function
    double sign = prev_vec[0]*vec[0] +  prev_vec[1]*vec[1] +  prev_vec[2]*vec[2];
    cout << "SIGN:" << sign << endl;
    if(sign<0.0){
        vec[0] = -vec[0]; vec[1] = -vec[1]; vec[2] = -vec[2]; 
    }
}

/**
* Determines whether a transition should occur to the next DMP based on residual "Error"
from the deformations

transition_x,transition_y, transition_z are outputs representing the change to be applied
to the next starting point
*/
void DeformationController::calculateDMPTransition(double ii, double &transition_x, double &transition_y, double &transition_z, double &transition_def_x, double &transition_def_y, double &transition_def_z, double x_imp, double y_imp, double z_imp, double x_def, double y_def, double z_def, vector<array<double,3>> selections,vector<array<double,7>> starting_points, vector<string> surfaces){
    if((ii+1)<selections.size()){
        // TWO TRANSITIONS -> need 6 transition variables, need the stiffness, need the deformation scaling

        // Situations - same requires direct addition
        // position to force -> nothing
        // force to position -> nothing
        // position to manifold
        // manifold to position

        std::string rospath = ros::package::getPath("dmp_deformations");

        if (selections[ii][2]==0.0 && selections[ii+1][2]==1.0){
            // On the manifold currently and coming off it!
            BSplineSurface surface;
            surface.loadSurface(rospath+"/../../devel/lib/dmp_deformations/"+surfaces[ii]+".csv");
            array<double,3> r; array<double,3> n_hat; array<double,3> r_u; array<double,3> r_v;

            // transition point based on current surface value (state + deformation)
            surface.calculateSurfacePoint(x_imp+x_def,y_imp+y_def,r,n_hat,r_u,r_v);

            transition_x = (r[0]-starting_points[ii+1][0]);
            transition_y = (r[1]-starting_points[ii+1][1]);
            transition_z = (r[2]-starting_points[ii+1][2]);

            transition_def_x = 0.0; transition_def_y = 0.0; transition_def_z = 0.0; 
        }

        else if (selections[ii][2]==1.0 && selections[ii+1][2]==0.0){
            
            // Need the surface that is being approached
            BSplineSurface surface;
            surface.loadSurface(rospath+"/../../devel/lib/dmp_deformations/"+surfaces[ii+1]+".csv");

            // Approaching the manifold, find the nearest manifold point to transition
            double u = starting_points[ii+1][0];
            double v = starting_points[ii+1][1];

            // Use TF2 to the get the current point
            ros::spinOnce();
            try{
                geometry_msgs::TransformStamped transformStamped;
                transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
                actual_pos[0] = transformStamped.transform.translation.x;
                actual_pos[1] = transformStamped.transform.translation.y;
                actual_pos[2] = transformStamped.transform.translation.z;
            }

            catch(tf2::TransformException &ex){
                cout << "Can't get TF2" << endl << ex.what() << endl;
            }
            getClosestParams(actual_pos[0], actual_pos[1], actual_pos[2],u,v,surface);

            transition_x = u-starting_points[ii+1][0];
            transition_y = v-starting_points[ii+1][1];
            transition_z = 0.0;

            cout << "TRANSITIONYYYYY MCGEEEEEE"<< endl << "TRANSITIONYYYYY MCGEEEEEE" << endl << "TRANSITIONYYYYY MCGEEEEEE" << transition_x << " " << transition_y << endl << "TRANSITIONYYYYY MCGEEEEEE" << endl;

            transition_def_x = 0.0; transition_def_y = 0.0; transition_def_z = 0.0; 
        }

        else if (selections[ii][2]==0.0 && selections[ii+1][2]==0.0){
            // Staying on the surface - continuity of all local variables
            transition_x = (x_imp-starting_points[ii+1][0]);
            transition_y = (y_imp-starting_points[ii+1][1]);
            transition_z = (z_imp-starting_points[ii+1][2]);

            transition_def_x = x_def;
            transition_def_y = y_def;
            transition_def_z = z_def;
        }

        else{
            transition_x = 0.0; transition_y = 0.0; transition_z = 0.0;
            transition_def_x = 0.0; transition_def_y = 0.0; transition_def_z = 0.0; 
        }
    }
}

/**
* TODO: Calculate whether there is a chance for collision or something...?
*/
void DeformationController::calculateNearestContact(int ii,vector<array<double,3>> selections,vector<array<double,7>> starting_points,vector<array<double,7>> attractor_points,bool &dmp_x_limiting,bool &dmp_y_limiting,bool &dmp_z_limiting,double &dmp_x_collision, double &dmp_y_collision, double &dmp_z_collision){
    
    // This is used to contact the "Nearest contact condition" to consider
    // minimizing the deformations
    dmp_x_limiting = false;
    dmp_y_limiting = false;
    dmp_z_limiting = false;

    // TODO: check over all this collision stuff WRT to the new rotations
    // There is a next DMP, check for pos to force transition
    if (ii+1<selections.size()){
        if(selections[ii][2] ==1 && selections[ii+1][2]==0){
            dmp_x_collision = attractor_points[ii][0];
            dmp_y_collision = attractor_points[ii][1];
            dmp_z_collision = attractor_points[ii][2];
            dmp_x_limiting=true;
            dmp_y_limiting=true;
            dmp_z_limiting=true;
        }
    }

    // There is a previous DMP, check for force to pos transition
    if (ii>0){
        if(selections[ii-1][0] ==0 && selections[ii][0]==1){
            dmp_x_collision = starting_points[ii][0];
            dmp_y_collision = starting_points[ii][1];
            dmp_z_collision = starting_points[ii][2];
            dmp_x_limiting=true;
            dmp_y_limiting=true;
            dmp_z_limiting=true;
        }
    }
}

/**
* Calculate the final deformation based on adaptive gains, collisions, workspace limits, etc.
*/
array<double,3> DeformationController::deformationScaling(array<double,3> &rotated_deformation, double var_x, double var_y, double var_z, geometry_msgs::Vector3 selection, double x, double y, double collision_scaling){
    
    array<double,3> final_deformation;

    double k= 50;

    // First scale based on the type of input
    final_deformation[0] = var_x*rotated_deformation[0];
    final_deformation[1] = var_y*rotated_deformation[1];
    final_deformation[2] = var_z*rotated_deformation[2];

    // Parameterized surfaces have a range of 0<=x<=1
    // Make sure it is not over the bounds (for hybrid control)
    double minEdgePercent = 0.02;
    if (selection.z == 0){
        // Direction 1
        if((final_deformation[0]/k+x)>(1.0-minEdgePercent)){
            final_deformation[0] = k*((1.0-minEdgePercent)-x);
        }
        else if((final_deformation[0]/k+x)<(minEdgePercent)){
            final_deformation[0] = k*((minEdgePercent)-x);
        }

        // Direction 2
        if((final_deformation[1]/k+y)>(1.0-minEdgePercent)){
            final_deformation[1] = k*((1.0-minEdgePercent)-y);
        }
        else if((final_deformation[1]/k+y)<(minEdgePercent)){
            final_deformation[1] = k*((minEdgePercent)-y);
        }

    }

    // Scale back based on potential collision
    final_deformation[0] = final_deformation[0]/collision_scaling;
    final_deformation[1] = final_deformation[1]/collision_scaling;
    final_deformation[2] = final_deformation[2]/collision_scaling;

    return final_deformation;
}

/**
* This function takes in all of the relevant parameters for a particular DMP and will figure out the
* starting velocity. This is used for setting the tool orientation as it approaches the surface.
*/
array<double,3> DeformationController::getFirstSurfaceVelocity(array<double,7> attractor_point, array<double,7> starting_point,double dmp_u,double dmp_v, array<double,3> r_u, array<double,3> r_v, double &ddu, double& ddv){
    
    // TODO: can this be zero?
    double k=50;
    array<double,3> vel_hat;

    // These next 2 are equivalent to calculating the first velocity in the DMP ignoring the deformation and damping (since the velocity is initially 0)

    // Calculate vel-U direction
    ddu = k*(attractor_point[0]-starting_point[0])+dmp_u;

    // Calculate vel-V direction
    ddv = k*(attractor_point[1]-starting_point[1])+dmp_v;

    vel_hat[0] = r_u[0]*ddu+r_v[0]*ddv;
    vel_hat[1] = r_u[1]*ddu+r_v[1]*ddv;
    vel_hat[2] = r_u[2]*ddu+r_v[2]*ddv;
    double vel_mag = sqrt(vel_hat[0]*vel_hat[0]+vel_hat[1]*vel_hat[1]+vel_hat[2]*vel_hat[2]);
    vel_hat[0]=vel_hat[0]/vel_mag; vel_hat[1]=vel_hat[1]/vel_mag; vel_hat[2]=vel_hat[2]/vel_mag;
    return vel_hat;
}

array<double,3> DeformationController::map_deformation_input(int method, double dmp_fx,double dmp_fy,double dmp_fz,double dx,double dy, double dz, array<double,4> q_surface){
    // TODO: add the selection vector stuff

    // Methods:
    // 1. Task oriented frame - define in terms of a principal axis for the task
    // 2. Velocity-oriented frame - deformation is a relative direction based on the current velocity

    // Rotate the deformation force into the relevant frame
    array<double,3> deformation_rotation;

    if (method==1){ 
        /////////////////
        // Task-Oriented Frame
        /////////////////
        // This quaternion rotates it to be aligned with the general camera/x-y directions
        // rotating global into the u-v frame -> q_uv ^-1 * def_global
        
        deformation_rotation = vectorIntoConstraintFrame(dmp_fx, dmp_fy, dmp_fz, q_surface[0], q_surface[1], q_surface[2], q_surface[3]);
        // cout << "INPUT:" << dmp_fx << " " << dmp_fy << " " << dmp_fz << endl;
        // cout << "INPUTR:" << deformation_rotation[0] << " " << deformation_rotation[1] << " " << deformation_rotation[2] << endl;
        return deformation_rotation;
    }

    else if (method==2){
        /////////////////
        // Velocity-oriented
        /////////////////
        array<double,3> manifold_vel_dir = {1.0, 0.0, 0.0}; // z is always zero as these are functionally
        // homogeneous coordinates since we are talking about a 2D projection + force

        if(dx!=0.0 || dy != 0.0)
        {
            manifold_vel_dir = {dx, dy, 0.0};
            double mag = sqrt(dx*dx+dy*dy);
            manifold_vel_dir[0] = manifold_vel_dir[0]/mag; manifold_vel_dir[1] = manifold_vel_dir[1]/mag;
        }

        array<double,3> y_temp;
        array<double,3> z_hat = {0.0, 0.0, 1.0};
        y_temp = crossProduct(z_hat,manifold_vel_dir);

        array<double,4> q_out_rc;
        rotationToQuaternion(manifold_vel_dir, y_temp,z_hat, q_out_rc);
        array<double,3> falconRotation = vectorIntoConstraintFrame(dmp_fx,dmp_fy,dmp_fz,0.0, 0.0, 0.7071, 0.7071);
        deformation_rotation = vectorOutOfConstraintFrame(falconRotation[0], falconRotation[1], falconRotation[2], q_out_rc[0], q_out_rc[1], q_out_rc[2], q_out_rc[3]);
        return deformation_rotation;
    }

    else{ // Invalid method - No rotation
        deformation_rotation = {dmp_fx, dmp_fy, dmp_fz};
        cout << "INVALID METHOD FOR INPUT MAPPING" << endl;
        return deformation_rotation;
    }
}

/**
* TODO: fill this out
*/
void DeformationController::forceOnloading(int ii, geometry_msgs::Vector3 selection, vector<array<double,7>> starting_points, vector<array<double,7>> attractor_points, vector<vector<array<double,7>>> dmps, BSplineSurface curr_surface, string surface_name, ros::Publisher &selection_vector_pub, ros::Publisher &constraint_frame_pub, ros::Publisher wrench_goal_pub, ros::Publisher hybrid_pub,double &ddu, double& ddv){
    // Do force onloading with the first sample
    cout << "Force Onloading Started..." << endl;
    geometry_msgs::Wrench ft;
    geometry_msgs::Pose pose;

    ft.force.x = starting_points[ii][0];
    ft.force.y = starting_points[ii][1];
    ft.force.z = starting_points[ii][2];
    
    // identity in the constraint frame
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    // Calculate the starting point for orientation based on the first velocity of the DMP
    array<double,3> r, n_hat, y_hat, x_hat;
    curr_surface.calculateSurfacePoint(starting_points[ii][0],starting_points[ii][1],r,n_hat,x_hat,y_hat);
    array<double,3> vel_hat = getFirstSurfaceVelocity(attractor_points[ii],starting_points[ii],dmps[ii][0][0],dmps[ii][0][1],x_hat,y_hat,ddu,ddv);

    // MH for static orientation
    if (!surfaceBidirectional(surface_name)){
        if(surface_name=="cowling4"){
            vel_hat[0]=x_hat[0]*0.0+y_hat[0]*1.0; vel_hat[1]=x_hat[1]*0.0+y_hat[1]*1.0; vel_hat[2]=x_hat[2]*0.0+y_hat[2]*1.0; // old velocity
        }
        else if(surface_name=="fastener1"){
            vel_hat[0]=x_hat[0]*0.0+y_hat[0]*-1.0; vel_hat[1]=x_hat[1]*0.0+y_hat[1]*-1.0; vel_hat[2]=x_hat[2]*0.0+y_hat[2]*-1.0; // old velocity
            cout << "FASTENER FASTENER" << endl << "FASTENER" << endl;
            cout << "V:" << vel_hat[0] << ", " << vel_hat[1] << ", " << vel_hat[2] << endl;
        }
        else{
            vel_hat[0]=x_hat[0]*-1.0+y_hat[0]*0.0; vel_hat[1]=x_hat[1]*-1.0+y_hat[1]*0.0; vel_hat[2]=x_hat[2]*-1.0+y_hat[2]*0.0; // old velocity
            cout << "ELSEIDEEE BOOO" << endl;
        }
        
    }

    // FORCE ONLOADING BASED ON CURRENT POSITION!!
    try{
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
        r[0] = transformStamped.transform.translation.x;
        r[1] = transformStamped.transform.translation.y;
        r[2] = transformStamped.transform.translation.z;
        actual_orientation[0] = transformStamped.transform.rotation.x;
        actual_orientation[1] = transformStamped.transform.rotation.y;
        actual_orientation[2] = transformStamped.transform.rotation.z;
        actual_orientation[3] = transformStamped.transform.rotation.w;
    }

    catch(tf2::TransformException &ex){
        cout << "Can't get TF2" << endl << ex.what() << endl;
    }

    // Z (normal) x X (vel) = +Y
    array<double,3> y_new = crossProduct(n_hat,vel_hat);

    geometry_msgs::Quaternion constraint_frame; 
    array<double,4> q_out; 

    rotationToQuaternion(vel_hat,y_new,n_hat,q_out);
    constraint_frame.x = q_out[0]; constraint_frame.y = q_out[1]; constraint_frame.z = q_out[2]; constraint_frame.w = q_out[3];

    // Convert the XYZ into the constraint frame
    array<double,3> temp_vec = vectorIntoConstraintFrame(r[0],r[1],r[2],q_out[0],q_out[1],q_out[2],q_out[3]);
    pose.position.x = temp_vec[0]; pose.position.y = temp_vec[1]; pose.position.z=temp_vec[2];

    panda_ros_msgs::HybridPose hybridPose;
    hybridPose.pose = pose;
    hybridPose.wrench.force.x = ft.force.x; hybridPose.wrench.force.y = ft.force.y; hybridPose.wrench.force.z = ft.force.z;
    hybridPose.constraint_frame = constraint_frame;
    hybridPose.sel_vector = {(uint8_t)selection.x, (uint8_t)selection.y, (uint8_t)selection.z, 1, 1, 1};
    hybrid_pub.publish(hybridPose);

    // This loop monitors the robot which has started moving in the
    // force direction until it achieves the desired force (within a tolerance)
    bool proper_contact = false;
    bool previous_sample_force = false;
    int num_good_samples = 0;
    int reqd_good_samples = 50;
    while(!proper_contact)
    {
        //proper_contact=true; // TODO: REMOVE/FIX THIS
        cout << "FZ: " << fz << " " << starting_points[ii][2] << " # good: " << num_good_samples << endl;
        // reaction force so flip sign
        // MH use absolute value for easier computation - should be overdamped
        //if(f_z_rotated>-0.95*starting_points[ii][2] && f_z_rotated<-1.05*starting_points[ii][2])
        if(abs(fz)>0.30*abs(starting_points[ii][2]))
        {
            num_good_samples++;
        }
        else{
            num_good_samples = 0;
        }

        if(num_good_samples>50){
            proper_contact=true;
        }
        
        // get new force readings and wait 1 ms
        ros::spinOnce();
        usleep(1000);
    }
    cout << "Force Onloading Complete..." << endl;
}

void DeformationController::readDemo(vector<vector<array<double,7>>> &dmps,vector<vector<array<double,7>>> &dmps_reverse,vector<array<double,3>> &selections,vector<array<double,7>> &starting_points,vector<array<double,7>> &attractor_points, vector<string> &surfaces, vector<vector<array<double,3>>> &variance_dmp, vector<string> &preactions){
    std::ifstream dmpfile(trajectoryFile);
    double junk;
    string temp;
    string surface;
    string preaction;

    vector<array<double,7>> dmp_temp;
    vector<array<double,7>> dmp_reverse_temp;
    vector<array<double,3>> dmp_variance_temp;

    bool loading_variances = false;
    bool loading_reverse = false;

    // Read entire file into the vectors
    if(dmpfile.good())
    {
        while(getline(dmpfile,temp,',')){
            // Either a new mode definition or continued values
            if(temp=="mode")
            {
                // New mode, need to create vector
                // for trajectory, get selection and
                // starting/attractor points
                // make this surface and stiffness (damping implied from overdamped)

                loading_variances = false;
                loading_reverse = false;

                getline(dmpfile,temp,',');
                surface = temp.c_str();
                surfaces.push_back(surface);
                getline(dmpfile,temp);
                preaction = temp.c_str();
                preactions.push_back(preaction);

                // Read Selection Vector
                array<double,3> selection_temp;

                getline(dmpfile,temp,','); selection_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,','); selection_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp); selection_temp[2] = atof(temp.c_str());
                selections.push_back(selection_temp);

                // Read Starting Points
                array<double,7> starting_temp;
                getline(dmpfile,temp,','); starting_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[2] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[3] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[4] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[5] = atof(temp.c_str());
                getline(dmpfile,temp); starting_temp[6] = atof(temp.c_str());
                starting_points.push_back(starting_temp);

                // Read Attractor Points
                array<double,7> attractor_temp;
                getline(dmpfile,temp,','); attractor_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[2] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[3] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[4] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[5] = atof(temp.c_str());
                getline(dmpfile,temp); attractor_temp[6] = atof(temp.c_str());
                attractor_points.push_back(attractor_temp);

                // Send the previous DMP if it has something in it
                // this avoids sending an empty DMP at the beginning
                if(dmp_temp.size()>0)
                {
                    dmps.push_back(dmp_temp);
                    dmps_reverse.push_back(dmp_reverse_temp);
                    variance_dmp.push_back(dmp_variance_temp);
                }

                // Reinitialize the temporary vector
                // to store the trajectory
                dmp_temp.clear();
                dmp_reverse_temp.clear();
                dmp_variance_temp.clear();
            }

            else if(temp=="variance")
            {
                loading_variances = true;
                getline(dmpfile,temp);
            }

            else if(temp=="reverse")
            {
                loading_reverse = true;
                getline(dmpfile,temp);
            }

            else if(loading_variances){
                array<double,3> variance_temp;
                // temp has first value already
                variance_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                variance_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp);
                variance_temp[2] = atof(temp.c_str());
                dmp_variance_temp.push_back(variance_temp);
            }

            else if(loading_reverse){
                array<double,7> state_vector_temp;
                // temp has first value already
                state_vector_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[2] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[3] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[4] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[5] = atof(temp.c_str());
                getline(dmpfile,temp);
                state_vector_temp[6] = atof(temp.c_str());
                dmp_reverse_temp.push_back(state_vector_temp);
            }

            else{ // Add new value to dmp_vector
                array<double,7> state_vector_temp;
                // temp has first value already
                state_vector_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[2] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[3] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[4] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[5] = atof(temp.c_str());
                getline(dmpfile,temp);
                state_vector_temp[6] = atof(temp.c_str());
                dmp_temp.push_back(state_vector_temp);
            }
        }

        // Publish the final dmp
        if(dmp_temp.size()>0)
        {
            dmps.push_back(dmp_temp);
            dmps_reverse.push_back(dmp_reverse_temp);
            variance_dmp.push_back(dmp_variance_temp);
        }
    }

    //close the filestream
    dmpfile.close();
}

void DeformationController::replay_demo(ros::NodeHandle n){

    std_msgs::String replay_str;
    panda_ros_msgs::HybridPose hybridPose;
    geometry_msgs::Quaternion constraint_frame;

    // ROS path
    std::string rospath = ros::package::getPath("dmp_deformations");
    
    // Publishers specifically used for replay
    ros::Publisher selection_vector_pub = 
        n.advertise<geometry_msgs::Vector3>("/panda/selection", 5);
    ros::Publisher wrench_goal_pub = 
        n.advertise<geometry_msgs::Wrench>("/panda/ee_wrench_goals", 5);
    ros::Publisher constraint_frame_pub = 
        n.advertise<geometry_msgs::Quaternion>("/panda/constraintframe", 5);
    ros::Publisher point_goal_pub = // Used to project the future path for the user TODO: actually
        n.advertise<geometry_msgs::Pose>("/panda/ee_point_goals", 5);
    ros::Publisher dmp_replay_pub = 
        n.advertise<std_msgs::String>("/dmp/replay", 5);
    ros::Publisher event_pub = 
        n.advertise<std_msgs::String>("/panda/commands", 5);

    ros::Publisher hybrid_pub = 
        n.advertise<panda_ros_msgs::HybridPose>("/panda/hybrid_pose", 1);   

    // Publishers specifically for intent visualization
    ros::Publisher defscaling_vector_pub = 
        n.advertise<geometry_msgs::Vector3>("/panda/defscaling", 5);
    ros::Publisher nominaltraj_vector_pub = 
        n.advertise<geometry_msgs::Vector3>("/panda/nominaltraj", 5);

    // Publishers specifically for data logging
    ros::Publisher deformation_publisher = 
        n.advertise<geometry_msgs::Vector3>("/lcsc/deformation", 1);

    // Pause 0.5s to allow publishers to register
    for(int jj=0; jj<500; jj++)
    {
        ros::spinOnce();
        usleep(1000);
    }

    // All of the data from the DMPs are stored in vectors
    vector<vector<array<double,7>>> dmps;
    vector<vector<array<double,7>>> dmps_reverse;
    vector<array<double,3>> selections;
    vector<array<double,7>> starting_points;
    vector<array<double,7>> attractor_points;
    vector<string> surfaces;
    vector<string> preactions;
    vector<vector<array<double,3>>> variance_dmp;

    // Reused ROS messages for publishing
    geometry_msgs::Pose pose;
    geometry_msgs::Wrench ft;
    geometry_msgs::Vector3 selection;
    
    // DMP Parameters - TODO: READ THESE FROM THE FILE
    double k=50;
    double b=sqrt(2*k); // ideal underdamped
    double b_def=2*sqrt(k); // overdamped
    readDemo(dmps,dmps_reverse,selections,starting_points,attractor_points,surfaces,variance_dmp,preactions);

    // Action: Tell the robot the replay is starting over
    // In case something needs to change or be reset in the simulation state
    replay_str.data = "reset";
    dmp_replay_pub.publish(replay_str);

    // Tell the robot to go to the overall starting point
    // Interpolate between the current position and desired position over 2 seconds!
    // TODO: add orientation?
    // TODO MAKE THE ABOVE CLOSED LOOP (waits until it gets to the starting point)!!!!!!!!!!


    // current position from TF2
    ros::spinOnce();
    try{
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
        actual_pos[0] = transformStamped.transform.translation.x;
        actual_pos[1] = transformStamped.transform.translation.y;
        actual_pos[2] = transformStamped.transform.translation.z;
        actual_orientation[0] = transformStamped.transform.rotation.x;
        actual_orientation[1] = transformStamped.transform.rotation.y;
        actual_orientation[2] = transformStamped.transform.rotation.z;
        actual_orientation[3] = transformStamped.transform.rotation.w;
    }

    catch(tf2::TransformException &ex){
        cout << "Can't get TF2" << endl << ex.what() << endl;
    }


    double xs = actual_pos[0];
    double ys = actual_pos[1];
    double zs = actual_pos[2];

    // set up position control for interpolation
    constraint_frame.x = 0.0; constraint_frame.y = 0.0; constraint_frame.z = 0.0; constraint_frame.w = 1.0;
    hybridPose.pose.orientation.x = 0.0; hybridPose.pose.orientation.y = 0.0; hybridPose.pose.orientation.z = 0.0; hybridPose.pose.orientation.w = 1.0;
    hybridPose.wrench.force.x = 0.0; hybridPose.wrench.force.y = 0.0; hybridPose.wrench.force.z = 0.0;
    hybridPose.constraint_frame = constraint_frame;
    hybridPose.sel_vector = {1, 1, 1, 1, 1, 1};

    array<double,4> starting_orientation = {starting_points[0][3], starting_points[0][4], starting_points[0][5], starting_points[0][6]};

    // Interpolate over the course of 2 seconds to starting position
    for(int jj=0; jj<2000; jj++)
    {
        ros::spinOnce();
        hybridPose.pose.position.x = xs+(starting_points[0][0]-xs)*((jj*1.0)/2000.0);
        hybridPose.pose.position.y = ys+(starting_points[0][1]-ys)*((jj*1.0)/2000.0);
        hybridPose.pose.position.z = zs+(starting_points[0][2]-zs)*((jj*1.0)/2000.0);

        // interpolate orientation
        array<double,4> interpq = slerp(actual_orientation,starting_orientation,(jj*1.0)/2000.0);
        hybridPose.pose.orientation.x = interpq[0];
        hybridPose.pose.orientation.y = interpq[1];
        hybridPose.pose.orientation.z = interpq[2];
        hybridPose.pose.orientation.w = interpq[3];

        hybrid_pub.publish(hybridPose);
        usleep(1000);
    }

    // TODO: check if there and wait!!!

    

    // Action: Tell the robot the replay is starting
    cout << "Replay Starting..." << endl;
    replay_str.data = "start";
    dmp_replay_pub.publish(replay_str);   

    // Variables needed for transitions and deformation collision-limiting
    bool previous_dmp_no_contact = true;
    double transition_x=0.0, transition_y=0.0, transition_z=0.0;
    double transition_def_x=0.0, transition_def_y=0.0, transition_def_z=0.0;
    bool dmp_x_limiting; bool dmp_y_limiting; bool dmp_z_limiting;
    double dmp_x_collision = 0.0;
    double dmp_y_collision = 0.0;
    double dmp_z_collision = 0.0;
    double surface_scaling_x = 1.0; double surface_scaling_y = 1.0;
    array<double,3> first_surf_vel = {0.0, 0.0, 0.0};

    array<double,3> u_dir = {1.0, 0.0, 0.0};
    array<double,3> v_dir = {0.0, 1.0, 0.0};

    // first direction on surface in parameterized coordinates
    double ddu;
    double ddv;

    BSplineSurface curr_surface;

    // Loop through to replay the demo
    for(int ii = 0; ii<selections.size();ii++)
    {
        if(surfaces[ii]!="")
        {
            // Load the B-spline surface
            curr_surface.loadSurface(rospath+"/../../devel/lib/dmp_deformations/"+surfaces[ii]+".csv");
            curr_surface.get_uv_dirs(u_dir ,v_dir);

            // get the surface ~ scaling
            surface_scaling_y = 1.0;

        }

        else{
            // surface scaling is 1.0
            u_dir = {1.0, 0.0, 0.0};
            v_dir = {0.0, 1.0, 0.0};

            surface_scaling_x = 1.0; surface_scaling_y = 1.0;
        }

        if(preactions[ii]!="")
        {
            // publish to the event channel
            if(preactions[ii]=="grasp" || preactions[ii]=="release"){
                usleep(1000000);
                std_msgs::String actionstr;
                actionstr.data = preactions[ii];
                event_pub.publish(actionstr);
                usleep(2000000);
            }

            // todo: add conditional logic
        }

        // First, publish selection vector
        selection.x =selections[ii][0];
        selection.y =selections[ii][1];
        selection.z =selections[ii][2];

        if((selection.x==0 || selection.y==0 || selection.z==0)){
            if(previous_dmp_no_contact){
                // If selection has force control and is transitioning from no-contact, need force onloading
                forceOnloading(ii,selection,starting_points,attractor_points,dmps,curr_surface,surfaces[ii],selection_vector_pub,constraint_frame_pub,wrench_goal_pub,hybrid_pub,ddu,ddv);
            }
            previous_dmp_no_contact = false;
        }

        else{
            previous_dmp_no_contact = true;
        }

        // Check for prominent collisions that should be avoided in deformation (aka known contact points from the model)
        calculateNearestContact(ii,selections,starting_points,attractor_points,dmp_x_limiting,dmp_y_limiting,dmp_z_limiting,dmp_x_collision,dmp_y_collision,dmp_z_collision);
        
        // Set up the values for the new DMP
        double ddx, ddy, ddz;
        double dx = 0.0, dy = 0.0, dz = 0.0;
        double x=starting_points[ii][0]+transition_x, y=starting_points[ii][1]+transition_y, z=starting_points[ii][2]+transition_z;

        cout << "XY" << x << " " << y << endl;

        // For the general impedance model
        double ddx_imp, ddy_imp, ddz_imp;
        double dx_imp = 0.0, dy_imp=0.0, dz_imp=0.0;
        double x_imp = starting_points[ii][0]+transition_def_x, y_imp=starting_points[ii][1]+transition_def_y, z_imp=starting_points[ii][2]+transition_def_z;
        double x_def = 0.0, y_def=0.0, z_def=0.0;

        // Set up the values for the Quaternion TODO: Move to CDMP
        double ddqx, ddqy, ddqz, ddqw;
        double dqx = 0.0, dqy = 0.0, dqz = 0.0, dqw=0.0;
        double qx=starting_points[ii][3], qy=starting_points[ii][4], qz=starting_points[ii][5], qw=starting_points[ii][6];
        
        double s = 0; // phase variable used for time
        double delta_s = 1.0;

        // These are used for the forwards/backwards DMP motion
        bool signCorrection = false;
        bool flippedOrientation = false;
        double dir_x, dir_y, dir_z;
        double dx_old, dy_old, dz_old;
        double dx_total_old = 0.0; double dy_total_old=0.0; double dz_total_old=0.0;

        // For the conditional logic
        // current position from TF2
        ros::spinOnce();
        try{
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
            actual_pos[0] = transformStamped.transform.translation.x;
            actual_pos[1] = transformStamped.transform.translation.y;
            actual_pos[2] = transformStamped.transform.translation.z;
            actual_orientation[0] = transformStamped.transform.rotation.x;
            actual_orientation[1] = transformStamped.transform.rotation.y;
            actual_orientation[2] = transformStamped.transform.rotation.z;
            actual_orientation[3] = transformStamped.transform.rotation.w;
        }

        catch(tf2::TransformException &ex){
            cout << "Can't get TF2" << endl << ex.what() << endl;
        }



        double starting_z = 0.208; // MH: hardcoded for task
        cout << "starting z:" << starting_z << endl;

        array<double,2> prev_uv = {-1, -1};

        constraint_frame.x=0.0; constraint_frame.y=0.0; constraint_frame.z=0.0; constraint_frame.w=1.0;

        double qx_old = starting_orientation[0];
        double qy_old = starting_orientation[1];
        double qz_old = starting_orientation[2];
        double qw_old = starting_orientation[3];

        // Now replay the entirety of the demonstration
        while(s<dmps[ii].size()){

            // Conditional Logic
            if(preactions[ii]=="peginhole"){
                // current position from TF2
                ros::spinOnce();
                try{
                    geometry_msgs::TransformStamped transformStamped;
                    transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
                    actual_pos[0] = transformStamped.transform.translation.x;
                    actual_pos[1] = transformStamped.transform.translation.y;
                    actual_pos[2] = transformStamped.transform.translation.z;
                    actual_orientation[0] = transformStamped.transform.rotation.x;
                    actual_orientation[1] = transformStamped.transform.rotation.y;
                    actual_orientation[2] = transformStamped.transform.rotation.z;
                    actual_orientation[3] = transformStamped.transform.rotation.w;
                }

                catch(tf2::TransformException &ex){
                    cout << "Can't get TF2" << endl << ex.what() << endl;
                }

                array<double,3> x_temp = {0.99978826,0.00928849,-0.01836173};
                array<double,3> y_temp = {-0.00907831,0.9998927,0.01149706};
                array<double,3> z_temp = {0.01846655,-0.01132793,0.9997653};
                array<double,4> q_out;

                rotationToQuaternion(x_temp,y_temp,z_temp,q_out);
                array<double,3> surface_pos = vectorIntoConstraintFrame(actual_pos[0], actual_pos[1], actual_pos[2], q_out[0], q_out[1], q_out[2], q_out[3]);
                
                cout << "point:" << surface_pos[2] << endl;
                //cout << "DIFF:" << abs(starting_z-actual_pos[2]) << endl;


                if((s+1)>dmps[ii].size()){
                    // Failed to find the hole
                    // Release the gripper
                    std_msgs::String actionstrtemp;
                    actionstrtemp.data = "release";
                    event_pub.publish(actionstrtemp);

                    // Now go up
                    // current position from TF2
                    ros::spinOnce();
                    try{
                        geometry_msgs::TransformStamped transformStamped;
                        transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
                        actual_pos[0] = transformStamped.transform.translation.x;
                        actual_pos[1] = transformStamped.transform.translation.y;
                        actual_pos[2] = transformStamped.transform.translation.z;
                        actual_orientation[0] = transformStamped.transform.rotation.x;
                        actual_orientation[1] = transformStamped.transform.rotation.y;
                        actual_orientation[2] = transformStamped.transform.rotation.z;
                        actual_orientation[3] = transformStamped.transform.rotation.w;
                    }

                    catch(tf2::TransformException &ex){
                        cout << "Can't get TF2" << endl << ex.what() << endl;
                    }


                    double xs = actual_pos[0];
                    double ys = actual_pos[1];
                    double zs = actual_pos[2];

                    // set up position control for interpolation
                    constraint_frame.x = 0.0; constraint_frame.y = 0.0; constraint_frame.z = 0.0; constraint_frame.w = 1.0;
                    hybridPose.pose.orientation.x = actual_orientation[0]; hybridPose.pose.orientation.y = actual_orientation[1]; hybridPose.pose.orientation.z = actual_orientation[2]; hybridPose.pose.orientation.w = actual_orientation[3];
                    hybridPose.wrench.force.x = 0.0; hybridPose.wrench.force.y = 0.0; hybridPose.wrench.force.z = 0.0;
                    hybridPose.constraint_frame = constraint_frame;
                    hybridPose.sel_vector = {1, 1, 1, 1, 1, 1};

                    array<double,4> starting_orientation = {starting_points[0][3], starting_points[0][4], starting_points[0][5], starting_points[0][6]};

                    // Interpolate over the course of 2 seconds to starting position
                    for(int jj=0; jj<2000; jj++)
                    {
                        ros::spinOnce();
                        hybridPose.pose.position.x = xs;
                        hybridPose.pose.position.y = ys;
                        hybridPose.pose.position.z = zs+(0.37-zs)*((jj*1.0)/2000.0);

                        hybrid_pub.publish(hybridPose);
                        usleep(1000);
                    }
                    
                    break;
                }

                else if((starting_z-surface_pos[2])>0.003){
                    // similar to force onloading... go down until a force for a while
                    // then go up to a fixed height
                    cout << "WOOKA WOOKA" << endl << "WOOKA WOOKA" << endl << "WOOKA WOOKA" << endl << "WOOKA WOOKA" << endl;
                    char d=(char)(7);
                    system("(speaker-test -t sine -f 1000)& pid=$!; sleep 0.2s; kill -9 $pid");
                    // This loop monitors the robot which has started moving in the
                    // force direction until it achieves the desired force (within a tolerance)
                    bool proper_contact = false;
                    bool previous_sample_force = false;
                    int num_good_samples = 0;
                    int reqd_good_samples = 50;
                    while(!proper_contact)
                    {
                        //proper_contact=true; // TODO: REMOVE/FIX THIS
                        cout << "FZ: " << fz << " " << starting_points[ii][2] << " # good: " << num_good_samples << endl;
                        // reaction force so flip sign
                        // MH use absolute value for easier computation - should be overdamped
                        //if(f_z_rotated>-0.95*starting_points[ii][2] && f_z_rotated<-1.05*starting_points[ii][2])
                        if(abs(fz)>0.30*abs(starting_points[ii][2]))
                        {
                            num_good_samples++;
                        }
                        else{
                            num_good_samples = 0;
                        }

                        if(num_good_samples>50){
                            proper_contact=true;
                        }
                        
                        // get new force readings and wait 1 ms
                        ros::spinOnce();
                        usleep(1000);
                    }

                     // current position from TF2
                    ros::spinOnce();
                    try{
                        geometry_msgs::TransformStamped transformStamped;
                        transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
                        actual_pos[0] = transformStamped.transform.translation.x;
                        actual_pos[1] = transformStamped.transform.translation.y;
                        actual_pos[2] = transformStamped.transform.translation.z;
                        actual_orientation[0] = transformStamped.transform.rotation.x;
                        actual_orientation[1] = transformStamped.transform.rotation.y;
                        actual_orientation[2] = transformStamped.transform.rotation.z;
                        actual_orientation[3] = transformStamped.transform.rotation.w;
                    }

                    catch(tf2::TransformException &ex){
                        cout << "Can't get TF2" << endl << ex.what() << endl;
                    }

                    double xs = actual_pos[0];
                    double ys = actual_pos[1];
                    double zs = actual_pos[2];

                    // set up position control for interpolation
                    constraint_frame.x = 0.0; constraint_frame.y = 0.0; constraint_frame.z = 0.0; constraint_frame.w = 1.0;
                    hybridPose.pose.orientation.x = actual_orientation[0]; hybridPose.pose.orientation.y = actual_orientation[1]; hybridPose.pose.orientation.z = actual_orientation[2]; hybridPose.pose.orientation.w = actual_orientation[3];
                    hybridPose.wrench.force.x = 0.0; hybridPose.wrench.force.y = 0.0; hybridPose.wrench.force.z = 0.0;
                    hybridPose.constraint_frame = constraint_frame;
                    hybridPose.pose.position.x = xs; hybridPose.pose.position.y = ys; hybridPose.pose.position.z = zs;
                    hybridPose.sel_vector = {1, 1, 1, 1, 1, 1};
                    hybrid_pub.publish(hybridPose);

                    // Release the gripper
                    std_msgs::String actionstrtemp;
                    actionstrtemp.data = "release";
                    event_pub.publish(actionstrtemp);
                    usleep(1000000);

                    // Now go up
                    array<double,4> starting_orientation = {starting_points[0][3], starting_points[0][4], starting_points[0][5], starting_points[0][6]};

                    // Interpolate over the course of 2 seconds to starting position
                    for(int jj=0; jj<2000; jj++)
                    {
                        ros::spinOnce();
                        hybridPose.pose.position.x = xs;
                        hybridPose.pose.position.y = ys;
                        hybridPose.pose.position.z = zs+(0.37-zs)*((jj*1.0)/2000.0);

                        hybrid_pub.publish(hybridPose);
                        usleep(1000);
                    }
                    
                    break;
                }
            }


            ros::spinOnce();
            double var_x = variance_dmp[ii][(int)floor(s)][0]+(variance_dmp[ii][(int)ceil(s)][0]-variance_dmp[ii][(int)floor(s)][0])*(s-floor(s));
            double var_y = variance_dmp[ii][(int)floor(s)][1]+(variance_dmp[ii][(int)ceil(s)][1]-variance_dmp[ii][(int)floor(s)][1])*(s-floor(s));
            double var_z = variance_dmp[ii][(int)floor(s)][2]+(variance_dmp[ii][(int)ceil(s)][2]-variance_dmp[ii][(int)floor(s)][2])*(s-floor(s));

            // If the variance starts changing (up or down), signal a haptic cue to the user
            //check_for_haptic_cue(var_x,var_y,var_z);
            double x_conv, y_conv, z_conv;
            
            ////////////////////////////////////////////////////////////////
            //      Deformation input mapping                             //
            ////////////////////////////////////////////////////////////////
            // 1 - Task Frame
            // 2 - Velocity Frame
            int input_mapping_method=1;
            array<double,4> q_surface;
            //cout << crossProduct(u_dir,v_dir)[0] << " " << crossProduct(u_dir,v_dir)[1] << " " << crossProduct(u_dir,v_dir)[2] << endl;
            rotationToQuaternion(u_dir,v_dir,crossProduct(u_dir,v_dir),q_surface);

            array<double,3> rotated_deformation = map_deformation_input(input_mapping_method,dmp_fx,dmp_fy,dmp_fz,dx,dy,dz,q_surface);

            /////////////////////////////////////////////////////////////////
            //      Deformation scaling                                    //
            /////////////////////////////////////////////////////////////////
            
            // Compute a potential DMP diminishing scale based on collisions
            double collision_scaling = 1.0;

            if(dmp_x_limiting){
                double dist_to_collision = sqrt((x-dmp_x_collision)*(x-dmp_x_collision)+(y-dmp_y_collision)*(y-dmp_y_collision)+(z-dmp_z_collision)*(z-dmp_z_collision));
                collision_scaling = 1-exp(-75*dist_to_collision);

            }
            
            array<double,3> final_deformation = deformationScaling(rotated_deformation,var_x,var_y,var_z,selection,x_imp,y_imp, collision_scaling);

            // Turn off deformations
            //final_deformation[0] = 0.0; final_deformation[1]=0.0; final_deformation[2]=0.0;
            
            ////////////////////////////////////////////////////////////////
            //      Calculate New DMP values                              //
            ////////////////////////////////////////////////////////////////

            // Interpolate each of the applied forces for the non-integer value s
            // TODO: this isn't right and once fixed, will also fail for the final sample
            double dmp_x, dmp_y, dmp_z;
            if(delta_s>=0.0){ // going forwards
                dmp_x = dmps[ii][(int)floor(s)][0]+(dmps[ii][(int)ceil(s)][0]-dmps[ii][(int)floor(s)][0])*(s-floor(s));
                dmp_y = dmps[ii][(int)floor(s)][1]+(dmps[ii][(int)ceil(s)][1]-dmps[ii][(int)floor(s)][1])*(s-floor(s));
                dmp_z = dmps[ii][(int)floor(s)][2]+(dmps[ii][(int)ceil(s)][2]-dmps[ii][(int)floor(s)][2])*(s-floor(s));
            }

            else{ //going backwards
                double s_max = float(dmps[ii].size());
                dmp_x = dmps_reverse[ii][(int)floor(s_max-s)][0]+(dmps_reverse[ii][(int)ceil(s_max-s)][0]-dmps_reverse[ii][(int)floor(s_max-s)][0])*(s_max-s-floor(s_max-s));
                dmp_y = dmps_reverse[ii][(int)floor(s_max-s)][1]+(dmps_reverse[ii][(int)ceil(s_max-s)][1]-dmps_reverse[ii][(int)floor(s_max-s)][1])*(s_max-s-floor(s_max-s));
                dmp_z = dmps_reverse[ii][(int)floor(s_max-s)][2]+(dmps_reverse[ii][(int)ceil(s_max-s)][2]-dmps_reverse[ii][(int)floor(s_max-s)][2])*(s_max-s-floor(s_max-s));
            }

            // Calculate New X (State 1)
            if(delta_s>=0.0){ //forwards
                ddx = k*(attractor_points[ii][0]-x_imp)-b*dx+dmp_x;
                dx = dx + ddx*0.01*delta_s;
                x_imp = x_imp+dx*0.01*delta_s;
            }

            else{ //backwards
                ddx = k*(starting_points[ii][0]-x_imp)-b*dx+dmp_x;
                dx = dx + ddx*0.01*abs(delta_s);
                x_imp = x_imp+dx*0.01*abs(delta_s);
            }
            
            // deformation
            ddx_imp = k*(-x_def)-b_def*dx_imp+final_deformation[0];
            dx_imp = dx_imp + ddx_imp*0.01*abs(delta_s);
            x_def = x_def + dx_imp*0.01*abs(delta_s);

            x = x_imp+x_def; 

            // Calculate New Y (State 2)
            if(delta_s>=0.0){ //forwards
                ddy = k*(attractor_points[ii][1]-y_imp)-b*dy+dmp_y;
                dy = dy + ddy*0.01*delta_s;
                y_imp = y_imp+dy*0.01*delta_s;
            }

            else{
                ddy = k*(starting_points[ii][1]-y_imp)-b*dy+dmp_y;
                dy = dy + ddy*0.01*abs(delta_s);
                y_imp = y_imp+dy*0.01*abs(delta_s);
            }
            
            //deformation
            ddy_imp = k*(-y_def)-b_def*dy_imp+final_deformation[1];
            dy_imp = dy_imp + ddy_imp*0.01*abs(delta_s);
            y_def = y_def + dy_imp*0.01*abs(delta_s);

            y = y_imp+y_def; 

            cout << "xy_nom: " << x_imp << " " << y_imp << endl; 

            // Calculate New Z (State 3)
            if(delta_s>=0.0){ //forwards
                ddz = k*(attractor_points[ii][2]-z_imp)-b*dz+dmp_z;
                dz = dz + ddz*0.01*delta_s;
                z_imp = z_imp+dz*0.01*delta_s;
            }

            else{
                ddz = k*(starting_points[ii][2]-z_imp)-b*dz+dmp_z;
                dz = dz + ddz*0.01*abs(delta_s);
                z_imp = z_imp+dz*0.01*abs(delta_s);
            }
            
            //deformation
            ddz_imp = k*(-z_def)-b_def*dz_imp+final_deformation[2];
            dz_imp = dz_imp + ddz_imp*0.01*abs(delta_s);
            z_def = z_def + dz_imp*0.01*abs(delta_s);

            z = z_imp+z_def; 

            ////////////////////////////////////////////////
            // Calculate the orientation - NO DEFORMATIONS /
            ////////////////////////////////////////////////
            
            if(delta_s>=0.0){ //forwards
                ddqx = k*(attractor_points[ii][3]-qx)-b*dqx;
                dqx = dqx + ddqx*0.01*delta_s;
                qx = qx+dqx*0.01*delta_s;
                ddqy = k*(attractor_points[ii][4]-qy)-b*dqy;
                dqy = dqy + ddqy*0.01*delta_s;
                qy = qy+dqy*0.01*delta_s;
                ddqz = k*(attractor_points[ii][5]-qz)-b*dqz;
                dqz = dqz + ddqz*0.01*delta_s;
                qz = qz+dqz*0.01*delta_s;
                ddqw = k*(attractor_points[ii][6]-qw)-b*dqw;
                dqw = dqw + ddqw*0.01*delta_s;
                qw = qw+dqw*0.01*delta_s;
            }

            else{ //backwards
                ddqx = k*(starting_points[ii][3]-qx)-b*dqx;
                dqx = dqx + ddqx*0.01*abs(delta_s);
                qx = qx+dqx*0.01*abs(delta_s);
                ddqy = k*(starting_points[ii][4]-qy)-b*dqy;
                dqy = dqy + ddqy*0.01*abs(delta_s);
                qy = qy+dqy*0.01*abs(delta_s);
                ddqz = k*(starting_points[ii][5]-qz)-b*dqz;
                dqz = dqz + ddqz*0.01*abs(delta_s);
                qz = qz+dqz*0.01*abs(delta_s);
                ddqw = k*(starting_points[ii][6]-qw)-b*dqw;
                dqw = dqw + ddqw*0.01*abs(delta_s);
                qw = qw+dqw*0.01*abs(delta_s);
            }


            // Filter the calculated orientation
            array<double,4> q_prev = {qx_old,qy_old,qz_old,qw_old};
            array<double,4> q_new = {qx,qy,qz,qw};
            array<double,4> q_filt;
            
            q_filt = slerp(q_prev, q_new,0.05);

            qw_old = qw;
            qx_old = qx;
            qy_old = qy;
            qz_old = qz;

            //qx = q_filt[0]; qy = q_filt[1]; qz = q_filt[2]; qw = q_filt[3];


            // Sign correction issues for transitioning between forwards and backwards DMPs
            // The sign correction is used between when the value of 's' changes value
            // and when the velocity has actually flipped to the opposite
            if(signCorrection){
                // dot product but keeping in mind the flip above
                double val = dx_old*dx+dy_old*dy+float(selection.z)*dz_old*dz;
                //cout << "DP:" << val << endl; 
                if (dx_old*dx+dy_old*dy+float(selection.z)*dz_old*dz<0.0){
                    signCorrection=false;
                }
            }

            // keep track of velocity for looking for signCorrection above
            dx_old = dx;
            dy_old = dy;
            dz_old = dz;
            dx_total_old = dx+dx_imp;
            dy_total_old = dy+dy_imp;
            dz_total_old = dz+dz_imp;

            // Want to keep the forward direction the same for the velocity heuristic
            // in 2 cases (moving backwards and vel flipped and moving forwards, but sign hasn't been corrected)
            // it needs to be switched in cardinality
            if((delta_s>=0.0 && !signCorrection) || (delta_s<0.0 && signCorrection)){
                dir_x = dx/sqrt(dx*dx+dy*dy+float(selection.z)*dz*dz);
                dir_y = dy/sqrt(dx*dx+dy*dy+float(selection.z)*dz*dz);
                dir_z = dz/sqrt(dx*dx+dy*dy+float(selection.z)*dz*dz);
            }
            else{
                dir_x = -dx/sqrt(dx*dx+dy*dy+float(selection.z)*dz*dz);
                dir_y = -dy/sqrt(dx*dx+dy*dy+float(selection.z)*dz*dz);
                dir_z = -dz/sqrt(dx*dx+dy*dy+float(selection.z)*dz*dz);
            }


            ////////////////////////////////////////////
            // Hybrid Control For Arbitrary Surfaces  //
            ////////////////////////////////////////////

            // If the mode is interaction, then a few things need to be converted:
            // 1. X and Y are the U,V of the parameterized surface and need to be
            // converted back to the X,Y,Z
            // 2. The orientation should be set to be the surface normal for the
            // given U and V
            // 3. Need to set the constraint frame based on the surface normal
            // 4. Rotate the X,Y,Z and orientation into the constraint frame
            x_conv = x; y_conv = y; z_conv = z;
            if(selection.z == 0)
            {
                // Calculate the constraint frame
                array<double,3> r = {0.0, 0.0, 0.0};
                array<double,3> n_hat = {0.0, 0.0, 0.0};
                array<double,3> y_hat;
                array<double,3> x_hat;

                // Make sure to not command a value off the surface
                if(x<0.01){
                    x = 0.01;
                }

                if(x>0.99){
                    x = 0.99;
                }

                if(y<0.01){
                    y = 0.01;
                }

                if(y>0.99){
                    y = 0.99;
                }

                curr_surface.calculateSurfacePoint(x,y,r,n_hat,x_hat,y_hat);

                
                // Using the velocity of x and y (u and v), calculate the time
                // derivative of the surface (i.e., vector-valued function)
                array<double,3> v_hat;
                //v_hat[0]=x_hat[0]*dx+y_hat[0]*dy; v_hat[1]=x_hat[1]*dx+y_hat[1]*dy; v_hat[2]=x_hat[2]*dx+y_hat[2]*dy; // old velocity
                v_hat[0]=x_hat[0]*(dx+dx_imp)+y_hat[0]*(dy+dy_imp); v_hat[1]=x_hat[1]*(dx+dx_imp)+y_hat[1]*(dy+dy_imp); v_hat[2]=x_hat[2]*(dx+dx_imp)+y_hat[2]*(dy+dy_imp); // true velocity
                //fix velocity -> transition_testing
                
                // MH  - Static orientation for the polishing!!!!!
                if(!surfaceBidirectional(surfaces[ii])){
                    if(surfaces[ii]=="cowling4"){
                        v_hat[0]=x_hat[0]*0.0+y_hat[0]*1.0; v_hat[1]=x_hat[1]*0.0+y_hat[1]*1.0; v_hat[2]=x_hat[2]*0.0+y_hat[2]*1.0; // old velocity
                    }

                    else if(surfaces[ii]=="fastener1"){
                        v_hat[0]=x_hat[0]*0.0+y_hat[0]*-1.0; v_hat[1]=x_hat[1]*0.0+y_hat[1]*-1.0; v_hat[2]=x_hat[2]*0.0+y_hat[2]*-1.0; // old velocity
                    }
                    else{
                        v_hat[0]=x_hat[0]*0.0+y_hat[0]*1.0; v_hat[1]=x_hat[1]*0.0+y_hat[1]*1.0; v_hat[2]=x_hat[2]*0.0+y_hat[2]*1.0; // old velocity
                    }
                }
                double v_mag = sqrt(v_hat[0]*v_hat[0] + v_hat[1]*v_hat[1] + v_hat[2]*v_hat[2]);

                //cout << "VHAT: " << v_hat[0] << " " << v_hat[1] << " " << v_hat[2] << endl;
                
                // If the velocity direction is non-zero, re-align the constraint frame
                // If it is zero, it will keep its previous value

                // heuristic to prevent large orientaion movements at slow speeds
                if(v_mag>0.0 && abs(v_mag*delta_s)>0.001)
                {
                    // change the constraint frame
                    v_hat[0]=v_hat[0]/v_mag; v_hat[1]=v_hat[1]/v_mag; v_hat[2]=v_hat[2]/v_mag;

                    
                    // Filter the parameterized velocity!!
                    if(surfaceBidirectional(surfaces[ii]))
                    {
                        double mag_new = sqrt((dx+dx_imp)*(dx+dx_imp)+(dy+dy_imp)*(dy+dy_imp));
                        double curr_u = (dx+dx_imp)/mag_new; double curr_v = (dy+dy_imp)/mag_new;
                        if(!(prev_uv[0]==-1 && prev_uv[1]==-1) && mag_new>0.0){
                            double alpha_filter = 0.08;
                            double dp_uv_fixed = (dx+dx_imp)*ddu+(dy+dy_imp)*ddv;
                            if(dp_uv_fixed<0.0){
                                curr_u = -curr_u;
                                curr_v = -curr_v;
                            }

                            double dp_uv = curr_u*prev_uv[0]+curr_v*prev_uv[1];
                            double theta = acos(dp_uv);
                            double angle_interp = alpha_filter*theta;
                            double anglesine = 1.0;
                            if(curr_v/mag_new*prev_uv[0]-curr_u/mag_new*prev_uv[1]<0.0){
                                anglesine=-1.0;
                            }
                            array<double,2> interp_uv = {cos(anglesine*angle_interp)*prev_uv[0]-sin(anglesine*angle_interp)*prev_uv[1], sin(anglesine*angle_interp)*prev_uv[0]+cos(anglesine*angle_interp)*prev_uv[1]};
                            cout << "actvel:" << curr_u/mag_new << " " << curr_v/mag_new << endl;
                            curr_u = interp_uv[0]; curr_v = interp_uv[1];
                            v_hat[0]=x_hat[0]*interp_uv[0]+y_hat[0]*interp_uv[1]; v_hat[1]=x_hat[1]*interp_uv[0]+y_hat[1]*interp_uv[1]; v_hat[2]=x_hat[2]*interp_uv[0]+y_hat[2]*interp_uv[1];
                        }
                        prev_uv = {curr_u,curr_v};
                    }


                    // Z x X = Y
                    array<double,3> y_new;
                    y_new = crossProduct(n_hat,v_hat);
                
                    array<double,4> q_out;
                    rotationToQuaternion(v_hat,y_new,n_hat,q_out);
                    constraint_frame.x = q_out[0]; constraint_frame.y = q_out[1]; constraint_frame.z = q_out[2]; constraint_frame.w = q_out[3];
                    //cout << "CF:" << q_out[0] << " " << q_out[1] << " " << q_out[2] << " " << q_out[3] << endl;
                }

                // Orientation is now just the identity in the constraint frame
                // NOTE: This is overwriting the DMP for orientation above!
                // TODO: this isn't very general - only for a bidirectional tool
                
                qx = 0.0; qy = 0.0; qz = 0.0; qw = 1.0;

                // double dp_uv = (dx+dx_imp)*ddu+(dy+dy_imp)*ddv;
                // if(dp_uv>=0.0){
                //     qx = 0.0; qy = 0.0; qz = 0.0; qw = 1.0;
                // }

                // else{
                //     qx = 0.0; qy = 0.0; qz = 1.0; qw = 0.0;
                // }


                // Convert the XYZ into the constraint frame
                array<double,3> xyz_conv = vectorIntoConstraintFrame(r[0], r[1], r[2], constraint_frame.x, constraint_frame.y, constraint_frame.z, constraint_frame.w);
                x_conv = xyz_conv[0];
                y_conv = xyz_conv[1];
                z_conv = z; // Z-direction is the force (hybrid control)
                
            }


            // Velocity Heuristic for evolution of the system
            // Looks at how much the deformation is in the opposite direction of the kinematic directions
            double dp_in_dir = dir_x*rotated_deformation[0] + dir_y*rotated_deformation[1]+float(selection.z)*dir_z*rotated_deformation[2];
            //cout << "DIR:" << dir_x << " " << dir_y << " " << dir_z << endl;
            double last_delta_s = delta_s;
            
            // only allow slowing down
            if(dp_in_dir > 0)
            {
                dp_in_dir = 0.0; 
            }

            // Allow up to 30 percent backwards
            delta_s = 1.0+1.9*dp_in_dir;
            //cout << "DS:" << delta_s << endl;

            //delta_s = 1.0;

            // // Set with keyboard instead of velocity direction 
            // if (dhdKbHit()) {
            
            //     char keypress = dhdKbGet();
            //     if (keypress == 'r'){
            //         delta_s = -0.75;
            //     }

            //     if (keypress == 'f'){
            //         delta_s = 1.0;
            //     }
            // }


            // Check for flip of delta s (+ to - or - to +)
            if((last_delta_s<=0.0 && delta_s > 0.0) || (last_delta_s>=0.0 && delta_s < 0.0))
            {
                signCorrection=true;
            }

            //cout << "DS:" << delta_s << endl;
            s+=delta_s;

            if (s<0.0){
                s = 0.0; // don't allow negative time
            }


            /////////////////////////////////////////////////////
            // INTENT AND EXPECTATION ///////////////////////////
            /////////////////////////////////////////////////////
            
            // Publish enough to show the ellipsoid of expectation

            // Need deformation amount for x,y,z
            // Need the nominal value for x,y,z
            geometry_msgs::Vector3 def_range;
            def_range.x = var_x/k; def_range.y = var_y/k; def_range.z = var_z/k;

            if(selection.z==0){
                def_range.x = def_range.x*surface_scaling_x;
                def_range.y = def_range.y*surface_scaling_y;
            }
            
            geometry_msgs::Vector3 nominal;
            nominal.x = x_def; nominal.y = y_def; nominal.z = z_def;

            defscaling_vector_pub.publish(def_range);
            nominaltraj_vector_pub.publish(nominal);


            // PUBLISH THE ROBOT CONTROL PARAMETERS
            
            hybridPose.pose.position.x = x_conv; hybridPose.pose.position.y = y_conv; hybridPose.pose.position.z = z_conv;
            hybridPose.pose.orientation.x = qx; hybridPose.pose.orientation.y = qy; hybridPose.pose.orientation.z = qz; hybridPose.pose.orientation.w = qw;
            hybridPose.wrench.force.x = x_conv; hybridPose.wrench.force.y = y_conv; hybridPose.wrench.force.z = z_conv;
            hybridPose.constraint_frame = constraint_frame;
            hybridPose.sel_vector = {(uint8_t)selection.x, (uint8_t)selection.y, (uint8_t)selection.z, 1, 1, 1};
            hybrid_pub.publish(hybridPose);

            ///////////////
            // Log data  //
            ///////////////

            // deformations
            geometry_msgs::Vector3 def_temp;
            def_temp.x=dmp_fx; def_temp.y=dmp_fy; def_temp.z = dmp_fz;
            deformation_publisher.publish(def_temp);

            // nominal state
            // total state
            
            // Pause for 0.01 seconds, but calculate velocities at 0.001s
            for (int yy=0; yy<10; yy++)
            {
                usleep(1000);
            }
        }

        // AFTER THE DMP IS COMPLETE, look at whether transitions are needed
        // Combat the transition discontinuities if there is still a transition left
        calculateDMPTransition(ii,transition_x, transition_y, transition_z,transition_def_x,transition_def_y,transition_def_z,x_imp,y_imp,z_imp,x_def,y_def,z_def,selections, starting_points, surfaces);

    }

    // Tells simulation that things are over if necessary
    replay_str.data = "end";
    dmp_replay_pub.publish(replay_str);
    // Pause 0.5s to allow final message
    for(int jj=0; jj<500; jj++)
    {
        ros::spinOnce();
        usleep(1000);
    }
}


void DeformationController::readPandaForces(geometry_msgs::Wrench wrench) {
    // Mainly used for force onloading during replay
    fx = wrench.force.x;
    fy = wrench.force.y;
    fz = wrench.force.z;
}

int DeformationController::run_deformation_controller(int argc, char **argv){
    ros::init(argc, argv, "DeformationController");
    ros::NodeHandle n("~");

    int deviceID = init_inputDevice();
    if (deviceID==-1) {
        cout << endl << "Failed to initialize input device." << endl;
        return -1;
    }
    
    cout << "Press 'r' to run replay" << endl << "Press 'q' to quit" << endl << endl;

    ros::Publisher command_pub = 
        n.advertise<std_msgs::String>("/panda/commands", 5);
    ros::Subscriber force_sub = n.subscribe("/panda/control_wrench", 10, &DeformationController::readPandaForces, this);
    
    tf2_ros::TransformListener tfListener(tfBuffer);

    fd_publisher = 
        n.advertise<geometry_msgs::Vector3>("/fd/input", 5);

    ros::Publisher file_pub = 
        n.advertise<std_msgs::String>("/dmp/filepub", 5);
    ros::Publisher reset_pub = 
        n.advertise<std_msgs::String>("/dmp/reset", 5);

    ros::Publisher gripper_toggle = 
        n.advertise<std_msgs::String>("/gripperToggle", 1);

    bool clutch = false;
    bool reset_center = true;
    bool freeze = false;
    bool recording = false;
    bool velocity_mode = false;
    int file_iter = 0;
    bool replay_mode = false;
    auto start = high_resolution_clock::now();
    
    bool timing_press = false;
    chrono::time_point<chrono::steady_clock> start_timer;
    chrono::steady_clock sc;

    // Switch into replay mode
    // TODO: switch this to a loop and something that can go back and forth
    replay_active = true;
    bool quit_replay = false;

    // Thread a process for the zero-displacement falcon controller!
    std::thread t1(&DeformationController::run_zero_displacement_controller, this);

    while(ros::ok() && !quit_replay)
    {
        if (dhdKbHit()) {
            
            char keypress = dhdKbGet();
            if (keypress == 'r'){
                cout << endl << "Running the replay." << endl;
                replay_demo(n);
                cout << endl << endl << "Press 'r' to run replay" << endl << "Press 'q' to quit" << endl << endl;
            }

            else if(keypress == 'q'){
                cout << endl << "Quitting the routine." << endl;
                quit_replay = true;
            }
        }
        usleep(1000);
        ros::spinOnce();
    }
    
    dhdEnableForce (DHD_OFF);
    dhdClose();
    return 0;
}
