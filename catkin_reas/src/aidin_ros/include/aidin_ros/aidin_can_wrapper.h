#ifndef __aidin_can_wrapper_H__
#define __aidin_can_wrapper_H__

// #define __cplusplus

//////////////////////////////////////////////////////////////////////////
// include files
#include <ECI113.h>
#include <ECI_pshpack1.h>
#include <ECI_poppack.h>
#include <cstdint>
#include <string>
#include <vector>
#include "Iir.h"

#define order 4 // 4th order (=2 biquads)

class aidin_can_wrapper
{
    private:
        bool verbose;
        ECI_CTRL_HDL  dwCtrlHandle  = ECI_INVALID_HANDLE;
        DWORD         dwHwIndex     = 0;
        DWORD         dwCtrlIndex   = 0;
        
        Iir::Butterworth::LowPass<order> filt_Fx;
        Iir::Butterworth::LowPass<order> filt_Fy;
        Iir::Butterworth::LowPass<order> filt_Fz;
        Iir::Butterworth::LowPass<order> filt_Tx;
        Iir::Butterworth::LowPass<order> filt_Ty;
        Iir::Butterworth::LowPass<order> filt_Tz;

        //
    private:
        void chkStatus(const ECI_RESULT hResult, std::string sFuncName);
        ECI_RESULT send_msg(uint8_t const * msg, const uint16_t index, const uint8_t dlc);
    public:
        aidin_can_wrapper();
        aidin_can_wrapper(bool verbose);
        void init_adapter();
        void init_sensor();
        void release();
        ECI_RESULT recieve_msg(ECI_CTRL_MESSAGE * astcCtrlMsg, DWORD& dwCount);
        void print_msgs(ECI_CTRL_MESSAGE const * msgs, const DWORD dwCount);
        void setup(float sample_rate, float cutoff_frequency);
        void start();
        void stop();
        bool process_msg(const ECI_CTRL_MESSAGE msg, float * value);
        void get_force_torque(float * force, float * torque);
};

#endif //__ECIDEMO113_H__

