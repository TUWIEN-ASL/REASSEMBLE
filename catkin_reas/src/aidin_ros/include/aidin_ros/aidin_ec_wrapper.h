#ifndef __aidin_ec_wrapper_H__
#define __aidin_ec_wrapper_H__

#include "ethercat.h"
#include <string>
#include "Iir.h"

#define order 4 // 4th order (=2 biquads)

class aidin_ec_wrapper {
    public:
        aidin_ec_wrapper(const std::string& ifname);
        ~aidin_ec_wrapper();

        void init(float sample_rate, float cutoff_frequency);
        void configureSensor(bool bias, bool accel, bool gyro);
        void stop();
        void getForceTorque(float& Fx, float& Fy, float& Fz, float& Tx, float& Ty, float& Tz);
        void getIMUData(float& Ax, float& Ay, float& Az, float& Gx, float& Gy, float& Gz);
        void getTempData(float& Temp);

    private:
        std::string interfaceName;
        char IOmap[4096];
        int expectedWKC;
        volatile int wkc;
        
        Iir::Butterworth::LowPass<order> filt_Fx;
        Iir::Butterworth::LowPass<order> filt_Fy;
        Iir::Butterworth::LowPass<order> filt_Fz;
        Iir::Butterworth::LowPass<order> filt_Tx;
        Iir::Butterworth::LowPass<order> filt_Ty;
        Iir::Butterworth::LowPass<order> filt_Tz;
};

#endif //__aidin_ec_wrapper_H__
