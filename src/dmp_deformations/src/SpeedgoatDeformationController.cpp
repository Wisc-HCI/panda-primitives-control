#include "DeformationController.cpp"
#include "std_msgs/Float64.h"

// DHDC already defined in deformationcontroller for keyboard recognition

class SpeedGoatDeformationController: public DeformationController{
    public:
        int init_inputDevice();
        array<double, 3> getInputDeviceVelocity();
        void run_zero_displacement_controller();
        SpeedGoatDeformationController(string file);
};

SpeedGoatDeformationController::SpeedGoatDeformationController(string file){
    inputDevice_velocity = {0.0, 0.0, 0.0};
    prev_var_x = 0.0; prev_var_y = 0.0; prev_var_z = 0.0;
    var_x_changing = false; var_y_changing = false; var_z_changing = false;
    if(file==""){
        trajectoryFile = "learneddmp.csv";
    }
    else{
        trajectoryFile = file;
    }

    theta_speedgoat = 0.0;
}



int SpeedGoatDeformationController::init_inputDevice() {
   }


 array<double, 3> SpeedGoatDeformationController::getInputDeviceVelocity() {
    double v_x,v_y,v_z;
    v_x = 0.0;
    v_y = 0.0;
    v_z = 0.0;
    dhdGetLinearVelocity(&v_x,&v_y,&v_z);
    return {v_x, v_y, v_z};
}

void SpeedGoatDeformationController::run_zero_displacement_controller(){
    // Subscriber to 
    while(1){
        // Saturate to 1 max
        if(abs(theta_speedgoat)>1.0){
            theta_speedgoat = theta_speedgoat / abs(theta_speedgoat);
        }

        dmp_fx = theta_speedgoat;
        cout << "TSG: " << dmp_fx << endl;
        usleep(1000);
    }
}


int main(int argc, char **argv) {    
    string filename = "";
    if(argc>1){
        filename=argv[1];
    }
    SpeedGoatDeformationController* controller = new SpeedGoatDeformationController(filename);
    int success = controller->run_deformation_controller(argc,argv);
    return 0;
}
