#include "DeformationController.cpp"

// DHDC already defined in deformationcontroller for keyboard recognition

class ForceDimensionDeformationController: public DeformationController{
    public:
        int init_inputDevice();
        array<double, 3> getInputDeviceVelocity();
        void run_zero_displacement_controller();
        ForceDimensionDeformationController(string file);
};

ForceDimensionDeformationController::ForceDimensionDeformationController(string file){
    // why won't this inherit properly
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

int ForceDimensionDeformationController::init_inputDevice() {
    cout <<"Setting up Force Dimension\n";
    int deviceID = dhdOpen();
    if (deviceID < 0) {
        cout << "Unable to open device" << endl;
        dhdSleep (1.0);
        return -1;
    }
    cout << "Force Dimension Setup Complete" << endl;

    // Turn on gravity compensation
    dhdEnableForce (DHD_ON);

    // Second button press to start control & forces
    bool buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    cout << "Press Button to start zero displacement controller" << endl << endl;

    bool buttonPressed=false;
    while(!buttonPressed)
        if(dhdGetButton(0)==1) // button pressed
        {
            buttonPressed = true;
        }
    
    buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    return deviceID;
}

 array<double, 3> ForceDimensionDeformationController::getInputDeviceVelocity() {
    double v_x,v_y,v_z;
    dhdGetLinearVelocity(&v_x,&v_y,&v_z);
    return {v_x, v_y, v_z};
}

void ForceDimensionDeformationController::run_zero_displacement_controller(){
    double viscous_replay = 60; // TODO: want to get to 2 sqrt(stiffness)
    double stiffness = 200; // for replay - everything is 2x of the falcon
    array<double, 3> forceDimensionPos = {0,0,0};

    long count = 0;   

    inputDevice_velocity = getInputDeviceVelocity();
    
    while(1){
        dhdGetPosition (&forceDimensionPos[0],&forceDimensionPos[1],&forceDimensionPos[2]);

        count++;

        // Haptic Cues
        if (haptic_cue_increased){
            if(haptic_cue_count>500){
                haptic_cue_increased=false;
                haptic_cue_count = 0;
            }
            else{
                // goes up by 20 after half a second then back
                stiffness = 2*(200.0-(100.0/62500.0)*(haptic_cue_count-250.0)*(haptic_cue_count-250.0));
                viscous_replay = 3.0*sqrt(stiffness);
                haptic_cue_count++;
                //cout << "HCI" << endl;
            }
        }

        if (haptic_cue_decreased){
            if(haptic_cue_count>500){
                haptic_cue_decreased=false;
                haptic_cue_count = 0;
            }
            else{
                // goes up by 20 after half a second then back
                stiffness = 2*(50.0+(50.0/62500.0)*(haptic_cue_count-250.0)*(haptic_cue_count-250.0));
                viscous_replay = 3.0*sqrt(stiffness);
                haptic_cue_count++;
                //cout << "HCD" << endl;
            }
        }


        // center the device around the center of motion (to scale to -1 to) 
        double x_d = 0.0125;
        double y_d = 0.0;
        double z_d = 0.025;
        
        dhdSetForceAndTorque(-stiffness*(forceDimensionPos[0]-x_d)-viscous_replay*inputDevice_velocity[0], 
                -stiffness*(forceDimensionPos[1]-y_d)-viscous_replay*inputDevice_velocity[1], 
                -stiffness*(forceDimensionPos[2]-z_d)-viscous_replay*inputDevice_velocity[2],0.0,0.0,0.0);

        // Store forcing from device for deformations
        // Note: these should be unit-normalized (i.e., span from -1 to 1)
        
        // the coordinate frame is to correspond with teleop of the panda from behind
        // dmp_fx = -(forceDimensionPos[0]-x_d)/(0.0625);
        // dmp_fy = -(forceDimensionPos[1]-y_d)/(0.12);
        // dmp_fz = (forceDimensionPos[2]-z_d)/(0.105);

        // the coordinate frame is to correspond with teleop of the panda from in front
        dmp_fx = (forceDimensionPos[0]-x_d)/(0.0625);
        dmp_fy = (forceDimensionPos[1]-y_d)/(0.12);
        dmp_fz = (forceDimensionPos[2]-z_d)/(0.105);


        usleep(1000);
    }
}


int main(int argc, char **argv) {    
    string filename = "";
    if(argc>1){
        filename=argv[1];
    }
    ForceDimensionDeformationController* controller = new ForceDimensionDeformationController(filename);
    int success = controller->run_deformation_controller(argc,argv);
    return 0;
}
