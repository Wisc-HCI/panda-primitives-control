#include "BSplineSurface.h"

using namespace std;

BSplineSurface::BSplineSurface(){
    // Empty default constructor
}

void BSplineSurface::initialize(int order, vector<vector<array<double,3>>> &pts){
    // Define the sizes of the B-Spline Surface
    k = order;
    control_pts = pts;
    m = pts.size()-1;
    n = pts.at(0).size()-1;

    // Make sure there are enough control pts
    if(m<=k || n<=k){
        cout << "Not Enough Control Pts for Curve Degree" << endl;
        exit(1);
    }

    // Define the knot vectors
    // Need to add the first and last value k times
    // (total of (m-k+2) = ((m+k+2) - 2k)

    // U-Knot
    vector<double> knot_u_temp;
    for (int ii=0;ii<k;ii++){
        knot_u_temp.push_back(0.0);
    }
    for (int ii=0; ii < (m-k+2); ii++){
        knot_u_temp.push_back(float(ii)/float(m-k+1));
    }
    for (int ii=0;ii<k;ii++){
        knot_u_temp.push_back(1.0);
    }
    
    
    knots_u = knot_u_temp;

    // V-Knot
    vector<double> knot_v_temp;
    for (int ii=0;ii<k;ii++){
        knot_v_temp.push_back(0.0);
    }
    for (int ii=0; ii < (n-k+2); ii++){
        knot_v_temp.push_back(float(ii)/float(n-k+1));
    }
    for (int ii=0;ii<k;ii++){
        knot_v_temp.push_back(1.0);
    }
    knots_v = knot_v_temp;
    calculate_plane_dirs();
}

// calculates the rotation matrix for rotation represented in exponentials
Eigen::MatrixXd BSplineSurface::exp_map(Eigen::VectorXd w){
    Eigen::MatrixXd w_hat(3,3);
    cout << "A" << endl;
    double theta = pow(w[0]*w[0]+w[1]*w[1]+w[2]*w[2],0.5);
    w_hat(0,0)=0.0;
    w_hat(0,1)=-w[2]/theta;
    w_hat(0,2)=w[1]/theta;
    w_hat(1,0)=w[2]/theta;
    w_hat(1,1)=0.0;
    w_hat(1,2)=-w[0]/theta;
    w_hat(2,0)=-w[1]/theta;
    w_hat(2,1)=w[0]/theta;
    w_hat(2,2)=0.0;

    Eigen::MatrixXd R(3,3);
    R = Eigen::MatrixXd::Identity(3, 3)+w_hat*sin(theta)+w_hat*w_hat*(1-cos(theta));
    return R;
}

// Next 3 pieces are for surface optimization
double BSplineSurface::obj_bestplane(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    vector<vector<array<double,3>>> *control_pts = (vector<vector<array<double,3>>> *) data;

    double residual = 0.0;
    
    // Go through each point and sum up the error
    for(int ii=0;ii<control_pts->size();ii++){
        for(int jj=0;jj<control_pts->at(0).size();jj++){
            Eigen::VectorXd w(3);
            Eigen::MatrixXd pt(3,1);
            w(0)=x[0]; w(1)=x[1]; w(2)=0.0;
            pt(0,0)=control_pts->at(ii).at(jj)[0]; pt(1,0)=control_pts->at(ii).at(jj)[1]; pt(2,0)=control_pts->at(ii).at(jj)[2];
            
            Eigen::MatrixXd d_vec(3,1);
            d_vec(0,0)=0.0; d_vec(1,0)=0.0; d_vec(2,0)=x[2];
            Eigen::MatrixXd vec_to_plane_center(3,1);
            cout << "B" << endl;
            Eigen::MatrixXd expmap(3,3);
            expmap = exp_map(w);
            cout << "C" << endl;
            vec_to_plane_center = expmap * d_vec - pt;

            Eigen::MatrixXd normal_vec(3,1);
            normal_vec(0,0) = 0.0; normal_vec(1,0) = 0.0; normal_vec(2,0) = 1.0; 

            Eigen::MatrixXd residual_temp;
            residual_temp = vec_to_plane_center.transpose() * expmap*normal_vec;

            double residual_tempf = residual_temp(0,0);

            residual+=abs(residual_tempf);
        }
    }
    
    return residual;
}

void BSplineSurface::calculate_plane_dirs(){
    // Fit the best plane of the surface to be used
    // for input mapping

    double best_opt = 9999999.9;
    nlopt::opt opt(nlopt::LN_COBYLA, 3);
    
    

    std::vector<double> lb(3);
    lb[0] = 0.0; lb[1] = 0.0; lb[2] = -9999999.9;
    std::vector<double> ub(3);
    ub[0] = 1.570796; ub[1] = 1.570796; ub[2] = 9999999.9;
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    double wx_best, wy_best, d_best;
    for(int i=0;i<5;i++){
        double wx = 1.570796*(float)rand()/RAND_MAX; //random 0->pi/2
        double wy = 1.570796*(float)rand()/RAND_MAX; //random 0->pi/2
        double d = 1.0*((float)rand()/RAND_MAX-0.5); //1.0*(np.random.rand()-0.5)

        // Initial guess for parameters U,V
        std::vector<double> params(3);
        params[0] = wx; params[1] = wy; params[2]=d;
        double minf;

        opt.set_min_objective(obj_bestplane,&control_pts);
        opt.set_xtol_rel(1e-4);

        try{
            nlopt::result result = opt.optimize(params, minf);
            
            // Output new values if found
            wx = params[0]; wy = params[1]; d = params[2];

            if (minf<best_opt){
                wx_best = wx;
                wy_best = wy;
                d_best = d;
            }
            
        }
        catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
    }

    cout << "PLANESIES: " << wx_best << " " << wy_best << " " << d_best << endl;
      
}



void BSplineSurface::calculateSurfacePoint(double u, double v, array<double,3> &r, array<double,3> &n_hat, array<double,3> &r_u, array<double,3> &r_v){
    r = {0.0, 0.0, 0.0};
    //cout << "STARTING" << endl;
    ///////////////////////////////////////////////
    //   Calculate the interpolated point        //
    ///////////////////////////////////////////////

    for(int ii=0;ii<(m+1);ii++){
        for(int jj=0;jj<(n+1);jj++){
            for(int xyz=0; xyz<3; xyz++)
            {
                r[xyz]+=getN(ii,k,u,knots_u)*getN(jj,k,v,knots_v)*control_pts.at(ii).at(jj)[xyz];
            }
        }
    }

    ///////////////////////////////////////////////
    //   Calculate the normal                    //
    ///////////////////////////////////////////////

    // Partial in the U-direction
    r_u = {0.0, 0.0, 0.0};
    vector<double> knots_u_shortened = knots_u;
    knots_u_shortened.erase(knots_u_shortened.begin());
    knots_u_shortened.pop_back();

    
    for(int ii=0;ii<(m);ii++){
        for(int jj=0;jj<(n+1);jj++){
            double scaling_temp = 0.0;
            if((knots_u.at(ii+k+1)-knots_u.at(ii+1))!=0){
                scaling_temp = k/(knots_u.at(ii+k+1)-knots_u.at(ii+1));
            }
            for(int xyz=0; xyz<3; xyz++)
            {
                r_u[xyz]+=getN(ii,k-1,u,knots_u_shortened)*getN(jj,k,v,knots_v)*scaling_temp*(control_pts.at(ii+1).at(jj)[xyz]-control_pts.at(ii).at(jj)[xyz]);
            }
        }
    }


    // Partial in the V-direction
    r_v = {0.0, 0.0, 0.0};
    vector<double> knots_v_shortened = knots_v;
    knots_v_shortened.erase(knots_v_shortened.begin());
    knots_v_shortened.pop_back();
    for(int ii=0;ii<(m+1);ii++){
        for(int jj=0;jj<(n);jj++){
            double scaling_temp = 0.0;
            if((knots_v.at(jj+k+1)-knots_v.at(jj+1))!=0){
                scaling_temp = k/(knots_v.at(jj+k+1)-knots_v.at(jj+1));
            }
            for(int xyz=0; xyz<3; xyz++)
            {
                r_v[xyz]+=getN(ii,k,u,knots_u)*getN(jj,k-1,v,knots_v_shortened)*scaling_temp*(control_pts.at(ii).at(jj+1)[xyz]-control_pts.at(ii).at(jj)[xyz]);
            }
        }
    }

    // Calculate the normal vector based on the U and V parials
    double u_mag = sqrt(r_u[0]*r_u[0]+r_u[1]*r_u[1]+r_u[2]*r_u[2]);
    double v_mag = sqrt(r_v[0]*r_v[0]+r_v[1]*r_v[1]+r_v[2]*r_v[2]);
    for(int i=0; i<3; i++){
        r_u[i] = r_u[i]/u_mag;
        r_v[i] = r_v[i]/v_mag;
    }

    // Cross product for normal vector
    n_hat[0]=r_u[1]*r_v[2]-r_u[2]*r_v[1];
    n_hat[1]=r_u[2]*r_v[0]-r_u[0]*r_v[2];
    n_hat[2]=r_u[0]*r_v[1]-r_u[1]*r_v[0];

}

double BSplineSurface::getN(int i, int p, double x, vector<double> t){
    // Recursive function that calculates the basis
    // function value using De Boor's
    // (simple version of the algorithm with 0-checking)
    //cout << "GETN: "<< i << " " << p << endl;
    if(p==0){
        //cout << t.at(i) << " " << t.at(i+1) << endl;
        if(x>=t.at(i) && x<t.at(i+1)){ // fire condition
            return 1.0;
        }
        else{ // no-fire condition
            return 0.0;
        }
    }

    else{ // Other levels based on recursion of lower levels 
        double part_a = 0.0;
        double part_b = 0.0;

        if((t.at(i+p)-t.at(i))!=0){
            part_a = (x-t.at(i))/(t.at(i+p)-t.at(i))*getN(i,p-1,x,t);
        }

        if((t.at(i+p+1)-t.at(i+1))!=0){
            part_b = (t.at(i+p+1)-x)/(t.at(i+p+1)-t.at(i+1))*getN(i+1,p-1,x,t);
        }

        return part_a + part_b;
    }
}


void BSplineSurface::loadSurface(string filename){
    // Load a CSV file
    std::ifstream surfacefile(filename);
    double k_temp;
    double m_temp;
    double n_temp;
    string temp;

    vector<vector<array<double,3>>> pts_temp;
    vector<array<double,3>> row_temp;
    array<double,3> temp_r;

    // Read entire file into the vectors
    if(surfacefile.good())
    {
        // First row has k, m, and n
        getline(surfacefile,temp,','); k_temp = atof(temp.c_str());
        getline(surfacefile,temp,','); m_temp = atof(temp.c_str());
        getline(surfacefile,temp); n_temp = atof(temp.c_str());
        
        for(int ii=0;ii<(m_temp+1);ii++){
            for(int jj=0;jj<(n_temp+1);jj++){
                if(jj==n_temp){
                    getline(surfacefile,temp);
                }
                else{
                    getline(surfacefile,temp,',');
                }

                // Temp is of the format x y z
                std::vector<std::string> temp_split;
                std::istringstream iss(temp);
                for(std::string s; iss >> s; ){
                    temp_split.push_back(s);
                }
                temp_r[0] = atof(temp_split.at(0).c_str());
                temp_r[1] = atof(temp_split.at(1).c_str());
                temp_r[2] = atof(temp_split.at(2).c_str());

                row_temp.push_back(temp_r);
            }

            // Reverse the row before adding
            pts_temp.push_back(row_temp);
            row_temp.clear();
        }

        // reverse before saving
        initialize(k_temp,pts_temp);
    }

}