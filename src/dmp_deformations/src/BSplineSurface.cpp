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
}

void BSplineSurface::get_uv_dirs(array<double,3> &u_dir , array<double,3> &v_dir){
    u_dir = u_dir_plane;
    v_dir = v_dir_plane;
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

        array<double,3> u_dir;
        getline(surfacefile,temp,','); u_dir[0] = atof(temp.c_str());
        getline(surfacefile,temp,','); u_dir[1] = atof(temp.c_str());
        getline(surfacefile,temp); u_dir[2] = atof(temp.c_str());

        array<double,3> v_dir;
        getline(surfacefile,temp,','); v_dir[0] = atof(temp.c_str());
        getline(surfacefile,temp,','); v_dir[1] = atof(temp.c_str());
        getline(surfacefile,temp); v_dir[2] = atof(temp.c_str());

        // Save directions
        u_dir_plane = u_dir;
        v_dir_plane = v_dir;

        // reverse before saving
        initialize(k_temp,pts_temp);
    }

}