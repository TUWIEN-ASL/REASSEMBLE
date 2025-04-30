#include "aidin_ros/aidin_can_wrapper.h"
#include <iostream>
#include <stdexcept>
#include "aidin_ros/can_utils.h"
#include <algorithm>


aidin_can_wrapper::aidin_can_wrapper()
{
    this->verbose = false;
}

aidin_can_wrapper::aidin_can_wrapper(bool verbose)
{
    this->verbose = verbose;
}

void aidin_can_wrapper::chkStatus(const ECI_RESULT hResult, std::string sFuncName)
{
    if(ECI_OK == hResult) {
        std::cout << sFuncName << " ...succeeded." << std::endl;
    } else {
        std::cout << sFuncName << " ...failed with errorcode: " << std::hex << hResult << " " << ECI113_GetErrorString(hResult) << std::endl;
    }
}

ECI_RESULT aidin_can_wrapper::send_msg(uint8_t const * msg, const uint16_t index, const uint8_t dlc)
{
    ECI_CTRL_MESSAGE stcCtrlMsg   = {0};
    ECI_RESULT hResult;

    stcCtrlMsg.wCtrlClass                            = ECI_CTRL_CAN;
    stcCtrlMsg.u.sCanMessage.dwVer                   = ECI_STRUCT_VERSION_V0;
    stcCtrlMsg.u.sCanMessage.u.V0.dwMsgId            = index;
    stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc  = dlc;

    std::cout << "dlc " << stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc << std::endl;

    for(unsigned int i = 0; i < sizeof(msg)/sizeof(msg[0]); ++i)
        stcCtrlMsg.u.sCanMessage.u.V0.abData[i] = msg[i];
    
    if(this->verbose)
        EciPrintCtrlMessage(&stcCtrlMsg);
    hResult = ECI113_CtrlSend(this->dwCtrlHandle, &stcCtrlMsg, DWORD(5000));

    return hResult;
}

void aidin_can_wrapper::init_adapter()
{
    ECI_RESULT  hResult     = ECI_OK;
    ECI_HW_PARA stcHwPara   = {0};
    ECI_HW_INFO stcHwInfo   = {0};

    //*** At first call Initialize to prepare ECI driver
    //*** with one board
    stcHwPara.wHardwareClass = ECI_HW_USB;
    hResult = ECI113_Initialize(1, &stcHwPara);

    if(ECI_OK != hResult){
        this->chkStatus(hResult, "ECI113_Initialize");
        throw std::runtime_error("ECIDRV_Initialize failed");
    }

    //*** Retrieve hardware info
    hResult = ECI113_GetInfo(this->dwHwIndex, &stcHwInfo);

    if(ECI_OK == hResult){
        EciPrintHwInfo(&stcHwInfo);
    }else{
        this->chkStatus(hResult, "ECI113_GetInfo");
        throw std::runtime_error("ECI113_GetInfo failed");
    }

    //*** Find first CAN Controller of Board
    hResult = EciGetNthCtrlOfClass(&stcHwInfo,
                                    ECI_CTRL_CAN,
                                    0, //first relative controller
                                    &this->dwCtrlIndex);
}

void aidin_can_wrapper::init_sensor()
{
    ECI_RESULT    hResult       = ECI_OK;

    //*** Open given controller of given board
    ECI_CTRL_CONFIG stcCtrlConfig = {0};

    //*** Set CAN Controller configuration
    stcCtrlConfig.wCtrlClass                = ECI_CTRL_CAN;
    stcCtrlConfig.u.sCanConfig.dwVer        = ECI_STRUCT_VERSION_V0;
    stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0 = ECI_CAN_BT0_1000KB;
    stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1 = ECI_CAN_BT1_1000KB;
    stcCtrlConfig.u.sCanConfig.u.V0.bOpMode = ECI_CAN_OPMODE_STANDARD | ECI_CAN_OPMODE_EXTENDED | ECI_CAN_OPMODE_ERRFRAME;

    //*** Open and Initialize given Controller of given board
    hResult = ECI113_CtrlOpen(&this->dwCtrlHandle, this->dwHwIndex, this->dwCtrlIndex, &stcCtrlConfig);

    if(ECI_OK != hResult){
        this->chkStatus(hResult, "ECI113_CtrlOpen");
        throw std::runtime_error("ECI113_CtrlOpen failed");
    }

    ECI_CTRL_CAPABILITIES stcCtrlCaps = {0};

    hResult = ECI113_CtrlGetCapabilities(this->dwCtrlHandle, &stcCtrlCaps);
    if(ECI_OK == hResult){
        if(this->verbose)
            EciPrintCtrlCapabilities(&stcCtrlCaps);
    }else{
        this->chkStatus(hResult, "ECI113_CtrlGetCapabilities");
        throw std::runtime_error("ECI113_CtrlGetCapabilities failed");
    }

    hResult = ECI113_CtrlStart(dwCtrlHandle);
    if(ECI_OK != hResult){
        this->chkStatus(hResult, "ECI113_CtrlStart");
        throw std::runtime_error("ECI113_CtrlStart failed");
    }
}

void aidin_can_wrapper::release()
{
    ECI_RESULT    hResult       = ECI_OK;
    hResult = ECI113_CtrlStop(dwCtrlHandle, ECI_STOP_FLAG_NONE);
    if(ECI_OK != hResult){
        this->chkStatus(hResult, "ECI113_CtrlStop, stop");
        throw std::runtime_error("ECI113_CtrlStop failed");
    }
    //*** Wait some time to ensure bus idle
    OS_Sleep(1000);

    hResult = ECI113_CtrlStop(dwCtrlHandle, ECI_STOP_FLAG_RESET_CTRL);
    if(ECI_OK != hResult){
        this->chkStatus(hResult, "ECI113_CtrlStop, reset");
        throw std::runtime_error("ECI113_CtrlStop failed");
    }

    //*** Close ECI Controller
    ECI113_CtrlClose(dwCtrlHandle);
    dwCtrlHandle = ECI_INVALID_HANDLE;

    ECI113_Release();

    std::cout << "CAN bus closed" << std::endl;
}

ECI_RESULT aidin_can_wrapper::recieve_msg(ECI_CTRL_MESSAGE* astcCtrlMsg, DWORD& dwCount)
{
    ECI_RESULT    hResult       = ECI_OK;
    //*** Try to read some messages
    hResult = ECI113_CtrlReceive( dwCtrlHandle, &dwCount, astcCtrlMsg, DWORD(5000));

    return hResult;
}

void aidin_can_wrapper::print_msgs(ECI_CTRL_MESSAGE const * msgs, const DWORD dwCount)
{
    unsigned int dwMsgIndex = 0;
    for(unsigned int i=0; i < dwCount; ++i)
    {
        EciPrintCtrlMessage(msgs+i);
    }
}

void aidin_can_wrapper::setup(float sample_rate, float cutoff_frequency){
    // if(sample_rate < 50)
    //     throw std::runtime_error("sample_rate must be above 50Hz");

    ECI_RESULT        hResult         = ECI_OK;

    // For some reason the sampling rate in c++ is twice the requested one
    // idk if there is some bug somewhere, but I cannot find it. For now
    // I simpli divide the sampling rate by two. If there is any problem
    // in your application check if this is not causing it.

    sample_rate = sample_rate / 2;

    int sr_param  = 1000000 / sample_rate;
    std::cout << "sr_param: " << sr_param << std::endl;
    int sr_param1 = sr_param / 256;
    std::cout << "sr_param1: " << sr_param1 << std::endl;
    int sr_param2 = sr_param % 256;
    std::cout << "sr_param2: " << sr_param2 << std::endl;

    if(this->verbose)
        std::cout << "setting" << std::endl;

    uint8_t msg_id[3] = {0x00, 0x01, 0x01};
    hResult = this->send_msg(msg_id, 0x102, 3);
    if(hResult != ECI_OK){
        this->chkStatus(hResult, "ECIDRV_CtrlSend");
        throw std::runtime_error("ECIDRV_CtrlSend failed on setting ID");
    }
    OS_Sleep(500);

    ECI_CTRL_MESSAGE  resp1[20] = {{0}};
    DWORD             dwCount1  = 20;
    hResult = this->recieve_msg(resp1, dwCount1);
    if(hResult == ECI_OK && this->verbose)
        this->print_msgs(resp1, dwCount1);
    OS_Sleep(500);

    // CONFIRM ID
    if(this->verbose)
        std::cout << "confirm" << std::endl;
    
    uint8_t msg_conf[2] = {0xFF, 0xFF};
    hResult = this->send_msg(msg_conf, 0x102, 2);
    if(hResult != ECI_OK){
        this->chkStatus(hResult, "ECIDRV_CtrlSend");
        throw std::runtime_error("ECIDRV_CtrlSend failed on confirming");
    }
    OS_Sleep(500);

    ECI_CTRL_MESSAGE  resp2[20] = {{0}};
    DWORD             dwCount2  = 20;
    hResult = this->recieve_msg(resp2, dwCount2);
    if(hResult == ECI_OK && this->verbose)
        this->print_msgs(resp2, dwCount2);
    OS_Sleep(500);

    // Set frequency
    if(this->verbose)
        std::cout << "setting frequency" << std::endl;

    uint8_t msg_freq[4] = {0x01, 0x05, sr_param1, sr_param2};
    hResult = this->send_msg(msg_freq, 0x102, 4);
    if(hResult != ECI_OK){
        this->chkStatus(hResult, "ECIDRV_CtrlSend");
        throw std::runtime_error("ECIDRV_CtrlSend failed on setting sampling rate");
    }
    OS_Sleep(500);

    ECI_CTRL_MESSAGE  resp3[20] = {{0}};
    DWORD             dwCount3  = 20;
    hResult = this->recieve_msg(resp3, dwCount3);
    if(hResult == ECI_OK && this->verbose)
        this->print_msgs(resp3, dwCount3);
    OS_Sleep(5000);

    // Set bias
    // if(this->verbose)
    //     std::cout << "setting bias" << std::endl;

    // uint8_t msg_bias[3] = {0x01, 0x02, 0x01};
    // hResult = this->send_msg(msg_bias, 0x102, 3);
    // if(hResult != ECI_OK){
    //     this->chkStatus(hResult, "ECIDRV_CtrlSend");
    //     throw std::runtime_error("ECIDRV_CtrlSend failed on setting bias");
    // }
    // OS_Sleep(5000);
    // ECI_CTRL_MESSAGE  resp4[20] = {{0}};
    // DWORD             dwCount4  = 20;
    // hResult = this->recieve_msg(resp4, dwCount4);
    // this->chkStatus(hResult, "ECIDRV_CtrlSend");
    // if(hResult == ECI_OK && this->verbose)
    //     std::cout << dwCount4 << std::endl;
    //     this->print_msgs(resp4, dwCount4);
    // OS_Sleep(1000);

    // Low-pass filter setup
    filt_Fx.setup (sample_rate, cutoff_frequency);
    filt_Fy.setup (sample_rate, cutoff_frequency);
    filt_Fz.setup (sample_rate, cutoff_frequency);
    filt_Tx.setup (sample_rate, cutoff_frequency);
    filt_Ty.setup (sample_rate, cutoff_frequency);
    filt_Tz.setup (sample_rate, cutoff_frequency);
    
}

void aidin_can_wrapper::start()
{
    ECI_RESULT        hResult         = ECI_OK;

    // starting
    OS_Sleep(500);
    if(this->verbose)
        std::cout << "Starting continious" << std::endl;

    uint8_t msg_start[3] = {0x01, 0x03, 0x01};
    hResult = this->send_msg(msg_start, 0x102, 3);
    if(hResult != ECI_OK){
        this->chkStatus(hResult, "ECIDRV_CtrlSend");
        throw std::runtime_error("ECIDRV_CtrlSend failed on starting");
    }
    OS_Sleep(500);
    std::cout << "Running continuous mode." << std::endl;
}

void aidin_can_wrapper::stop()
{
    ECI_RESULT        hResult         = ECI_OK;
    // stopping
    OS_Sleep(500);
    if(this->verbose)
        std::cout << "Stopping" << std::endl;
    
    uint8_t msg_conf[2] = {0xFF, 0xFF};
    hResult = this->send_msg(msg_conf, 0x102, 2);
    OS_Sleep(500);
}

bool aidin_can_wrapper::process_msg(const ECI_CTRL_MESSAGE msg, float * value)
{
    float comp_x = msg.u.sCanMessage.u.V0.abData[0]*256 + msg.u.sCanMessage.u.V0.abData[1];
    float comp_y = msg.u.sCanMessage.u.V0.abData[2]*256 + msg.u.sCanMessage.u.V0.abData[3];
    float comp_z = msg.u.sCanMessage.u.V0.abData[4]*256 + msg.u.sCanMessage.u.V0.abData[5];
    if(msg.u.sCanMessage.u.V0.dwMsgId == 0x01)
    {
        value[0] = comp_x / 100 - 300;
        value[1] = comp_y / 100 - 300;
        value[2] = comp_z / 100 - 300;
        return true;
    } else if(msg.u.sCanMessage.u.V0.dwMsgId == 0x02) {
        value[0] = comp_x / 500 - 50;
        value[1] = comp_y / 500 - 50;
        value[2] = comp_z / 500 - 50;
        return false;
    } else
        throw std::runtime_error("Incorrect indexes in message. Try running your script again");
}

void aidin_can_wrapper::get_force_torque(float * force, float * torque)
{
    ECI_RESULT        hResult         = ECI_OK;
    float measurment[3];
    bool is_force;
    bool got_force = false;
    bool got_torque = false;

    while (!got_force && !got_torque)
    {
        // std::cout << "getting ft data" << std::endl;
        ECI_CTRL_MESSAGE  astcCtrlMsg[1] = {{0}};
        DWORD             dwCount         = 1;
        hResult = this->recieve_msg(astcCtrlMsg, dwCount);
        if(hResult == ECI_OK)
        {
            is_force = this->process_msg(astcCtrlMsg[0], measurment);
            if(is_force) {
                got_force = true;
                force[0] = measurment[0];
                force[1] = measurment[1];
                force[2] = measurment[2];
            } else {
                got_torque = true;
                torque[0] = measurment[0];
                torque[1] = measurment[1];
                torque[2] = measurment[2];
            }
            
            // Low-pass filter measurement data
            force[0] = filt_Fx.filter(force[0]);
            force[1] = filt_Fy.filter(force[1]);
            force[2] = filt_Fz.filter(force[2]);
            torque[0] = filt_Tx.filter(torque[0]);
            torque[1] = filt_Ty.filter(torque[1]);
            torque[2] = filt_Tz.filter(torque[2]);
        }
    }
}