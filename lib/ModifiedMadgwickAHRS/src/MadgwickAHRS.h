//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

class Madgwick{
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain

    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame

    float invSampleFreq; // time step of gyro samples
    //euler angles in degrees
    float roll;
    float pitch;
    float yaw;

    char anglesComputed;
    void computeAngles();

   


public:

    Madgwick(float beta_value=0.1f,float dt=.002); // custom constructor with default values

    void reset();
    
    void setBeta(float beta_value){beta=beta_value;}; //update beta 
    void setDeltaT(float dt){invSampleFreq = dt;};
    
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    
};
#endif