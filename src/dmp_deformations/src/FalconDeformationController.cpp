#include "DeformationController.cpp"

#include "falcon/core/FalconDevice.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

using namespace libnifalcon;
using namespace StamperKinematicImpl;

class FalconDeformationController: public DeformationController{
    private:
        FalconDevice m_falconDevice;
        bool last_falcon_updated;
        array<double, 3> last_falcon;
    
    public:
        FalconDeformationController();
        FalconDeformationController(string file);

        int init_inputDevice();
        array<double, 3> getInputDeviceVelocity();
        void run_zero_displacement_controller();
};

// Override the constructor to include the additional velocity tracking
//variables needed for the falcon
// calls default constructor as well
FalconDeformationController::FalconDeformationController(){
    last_falcon_updated = false;
    inputDevice_velocity = {0.0, 0.0, 0.0};
    prev_var_x = 0.0; prev_var_y = 0.0; prev_var_z = 0.0;
    var_x_changing = false; var_y_changing = false; var_z_changing = false;
    trajectoryFile = "learneddmp.csv";
}

FalconDeformationController::FalconDeformationController(string file){
    last_falcon_updated = false;
    inputDevice_velocity = {0.0, 0.0, 0.0};
    prev_var_x = 0.0; prev_var_y = 0.0; prev_var_z = 0.0;
    var_x_changing = false; var_y_changing = false; var_z_changing = false;
    trajectoryFile = file;
    if(file==""){
        trajectoryFile = "learneddmp.csv";
    }
    else{
        trajectoryFile = file;
    }
}

int FalconDeformationController::init_inputDevice() {
    cout <<"Setting up LibUSB\n";
    m_falconDevice.close();
    m_falconDevice.setFalconKinematic<FalconKinematicStamper>();
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware

    if(!m_falconDevice.open(0)) //Open falcon @ index
    {
        cout << "Failed to find Falcon\n";
        return -1;
    }
    else
    {
        cout << "Falcon Found\n";
    }

    bool skip_checksum = false;
    bool firmware_loaded = false;
    
    // MH: forcing the firmware to reload seems to solve the issue where
    // we had to keep re-plugging it in

    if(!firmware_loaded)
    {
        cout << "Loading firmware\n";

        uint8_t* firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
        long firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;

        for(int i = 0; i < 50; ++i)	//Attempt to load firmware 50 times
        {
            if(!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, firmware_size, firmware_block))
            {
                cout << "Firmware loading try failed\n";
            }
            else
            {
                firmware_loaded = true;
                break;
            }
        }
    }

    if(!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
    {
        cout << "No firmware loaded to device, cannot continue\n";
        return -1;
    }
    cout << "Firmware loaded\n";

    m_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    cout << "Homing Set\n";

    m_falconDevice.runIOLoop(); //read in data

    bool stop = false;
    bool homing = false;
    usleep(100000);
    
    while(!stop)
    {
        if(!m_falconDevice.runIOLoop()) {
            continue;
        }
        if(!m_falconDevice.getFalconFirmware()->isHomed())
        {
            if(!homing)
            {
                m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                cout << "Falcon not currently homed. Move control all the way out then push straight all the way in.\n";
            }
            homing = true;
        }

        if(homing && m_falconDevice.getFalconFirmware()->isHomed())
        {
            m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::GREEN_LED);
            cout << "Falcon homed.\n";
            stop = true;
        }
    }
    m_falconDevice.setFalconGrip<libnifalcon::FalconGripFourButton>();

    m_falconDevice.runIOLoop();
    
    return 0;
}

array<double, 3> FalconDeformationController::getInputDeviceVelocity() {
    double delta_T = 1000;
    double v_x=0.0; double v_y=0.0; double v_z=0.0;
    // Compute the velocity to add some viscous friction to help with stability
    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();

    // Provide there is a last position to compute the velocity, calculate with backwards differencing
    if(last_falcon_updated)
    {
        // Velocity calculated as basic differencing
        v_x = (falconPos[0]-last_falcon[0])/delta_T;
        v_y = (falconPos[1]-last_falcon[1])/delta_T;
        v_z = (falconPos[2]-last_falcon[2])/delta_T;
    }

    //Update last falcon for use in the velocity calculation
    last_falcon[0] = falconPos[0];
    last_falcon[1] = falconPos[1];
    last_falcon[2] = falconPos[2];

    last_falcon_updated = true;
    return {v_x, v_y, v_z};
}

void FalconDeformationController::run_zero_displacement_controller(){
    double viscous_replay = 30; // TODO: want to get to 2 sqrt(stiffness)
    double stiffness = 100; // for replay
    array<double, 3> falconPos = {0,0,0};

    long count = 0;   

    inputDevice_velocity = getInputDeviceVelocity();
    
    while(1){
        m_falconDevice.runIOLoop();
        falconPos = m_falconDevice.getPosition();

        count++;

        // Haptic Cues
        if (haptic_cue_increased){
            if(haptic_cue_count>500){
                haptic_cue_increased=false;
                haptic_cue_count = 0;
            }
            else{
                // goes up by 20 after half a second then back
                stiffness = 200.0-(100.0/62500.0)*(haptic_cue_count-250.0)*(haptic_cue_count-250.0);
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
                stiffness = 50.0+(50.0/62500.0)*(haptic_cue_count-250.0)*(haptic_cue_count-250.0);
                viscous_replay = 3.0*sqrt(stiffness);
                haptic_cue_count++;
                //cout << "HCD" << endl;
            }
        }

        //cout << "FP:" << falconPos[0] << " " << falconPos[1] << " " << falconPos[2] << endl;
        // zero displacement mode
        // falcon has offset in z
        m_falconDevice.setForce({
                -stiffness*falconPos[0]-viscous_replay*inputDevice_velocity[0], 
                -stiffness*falconPos[1]-viscous_replay*inputDevice_velocity[1], 
                -stiffness*(falconPos[2]-0.125)-viscous_replay*inputDevice_velocity[2]});

        // Store forcing from device for deformations
        // Note: these should be unit-normalized (i.e., span from -1 to 1)
        // the coordinate frame is to correspond with teleop of the panda from behind
        dmp_fx = -(falconPos[2]-0.125)/(0.05);
        dmp_fy = -(falconPos[0])/(0.06);
        dmp_fz = falconPos[1]/(0.055);
        usleep(1000);
    }
}


int main(int argc, char **argv) {
    string filename = "";
    if(argc>1){
        filename=argv[1];
    }
    FalconDeformationController* controller = new FalconDeformationController(filename);
    int success = controller->run_deformation_controller(argc,argv);
    return 0;
}
