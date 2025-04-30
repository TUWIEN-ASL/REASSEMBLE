#include "aidin_ros/aidin_ec_wrapper.h"
#include <iostream>

aidin_ec_wrapper::aidin_ec_wrapper(const std::string& ifname) : interfaceName(ifname) {
    // bind socket
    if (!ec_init(interfaceName.c_str())) {
        std::cout << "Failed to initialize EtherCAT on interface " << interfaceName  << std::endl;
    } else{
        std::cout << "EtherCAT initialized on " << interfaceName  << std::endl;
    }

    if (ec_config_init(FALSE) <= 0) {
        std::cout << "No slaves found or configuration failed."  << std::endl;
        ec_close();
    } else {
        std::cout << ec_slavecount << " slaves found and configured."  << std::endl;
    }

    ec_config_map(&IOmap);
    ec_configdc();
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // request OP state for all slaves
    ec_writestate(0);
    int chk = 200;
    // wait for all slaves to reach OP state
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    }
    while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state == EC_STATE_OPERATIONAL )
    {
        printf("Operational state reached for all slaves.\n");
    }
    else
    {
        printf("Not all slaves reached operational state.\n");
        ec_readstate();
        int i = 0;
        for(i = 1; i<=ec_slavecount ; i++)
        {
            if(ec_slave[i].state != EC_STATE_OPERATIONAL)
            {
                printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                    i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    }
}

aidin_ec_wrapper::~aidin_ec_wrapper() {
    stop();
}

void aidin_ec_wrapper::init(float sample_rate, float cutoff_frequency) {

    // Low-pass filter setup
    filt_Fx.setup (sample_rate, cutoff_frequency);
    filt_Fy.setup (sample_rate, cutoff_frequency);
    filt_Fz.setup (sample_rate, cutoff_frequency);
    filt_Tx.setup (sample_rate, cutoff_frequency);
    filt_Ty.setup (sample_rate, cutoff_frequency);
    filt_Tz.setup (sample_rate, cutoff_frequency);
    printf("Low-pass filter configured.\n");
}

void aidin_ec_wrapper::configureSensor(bool bias, bool accel, bool gyro) {
    /* RxPDO for Bias, Accel ON, Gyro ON */
    /*
    *(ec_slave[0].outputs+0)=1; // Bias
    *(ec_slave[0].outputs+0)=2; // Accel ON
    *(ec_slave[0].outputs+0)=3; // Bias, Accel ON
    *(ec_slave[0].outputs+0)=4; // Gyro ON
    *(ec_slave[0].outputs+0)=5; // Bias, Gyro ON
    *(ec_slave[0].outputs+0)=6; // Accel ON, Gyro ON
    *(ec_slave[0].outputs+0)=7; // Bias, Accel ON, Gyro ON
    */

    // Use no setting changes
    if (!bias && !accel && !gyro) {
        std::cout << "No configuration change required." << std::endl;
        return;
    }

    std::cout << "Configuring sensor settings on slaves." << std::endl;
    if (ec_slavecount > 0 && ec_slave[0].outputs != nullptr) {
        int setting = 0;
        if (bias) setting |= 1;  // Set the first bit if bias is true
        if (accel) setting |= 2; // Set the second bit if accel is true
        if (gyro) setting |= 4;  // Set the third bit if gyro is true

        *(ec_slave[0].outputs+0) = setting;
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        printf("Expected WKC: %d, Received WKC: %d\n", expectedWKC, wkc);
        std::cout << "Sensor configuration completed with setting " << setting << "." << std::endl;
        std::cout << "Bias: " << bias << " Accel: " << accel << " Gyro: " << gyro << std::endl;
    } else {
        std::cerr << "No slaves configured or outputs pointer is null." << std::endl;
    }
}

void aidin_ec_wrapper::stop() {
    // Cleanup and stop EtherCAT

    // request INIT state for all slaves
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);

    // close socket
    ec_close();

    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    printf("Expected WKC: %d, Received WKC: %d\n", expectedWKC, wkc);

    printf("Sensor shutdown.\n");
}

void aidin_ec_wrapper::getForceTorque(float& Fx, float& Fy, float& Fz, float& Tx, float& Ty, float& Tz) {
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    
    // Process the data only if wkc is as expected
    if(wkc >= expectedWKC) {

        // Get force and torque data from EtherCAT slaves
        float Fx_raw = static_cast<float>(*(ec_slave[0].inputs+1)*256 + *(ec_slave[0].inputs));   // Example mapping
        float Fy_raw = static_cast<float>(*(ec_slave[0].inputs+5)*256 + *(ec_slave[0].inputs+4));
        float Fz_raw = static_cast<float>(*(ec_slave[0].inputs+9)*256 + *(ec_slave[0].inputs+8));
        float Tx_raw = static_cast<float>(*(ec_slave[0].inputs+13)*256 + *(ec_slave[0].inputs+12));
        float Ty_raw = static_cast<float>(*(ec_slave[0].inputs+17)*256 + *(ec_slave[0].inputs+16));
        float Tz_raw = static_cast<float>(*(ec_slave[0].inputs+21)*256 + *(ec_slave[0].inputs+20));

        Fx = (Fx_raw / 100.0) - 300.0;
        Fy = (Fy_raw / 100.0) - 300.0;
        Fz = (Fz_raw / 100.0) - 300.0;
        Tx = (Tx_raw / 500.0) - 50.0;
        Ty = (Ty_raw / 500.0) - 50.0;
        Tz = (Tz_raw / 500.0) - 50.0;

        // // Low-pass filter measurement data
        // Fx = filt_Fx.filter(Fx);
        // Fy = filt_Fy.filter(Fy);
        // Fz = filt_Fz.filter(Fz);
        // Tx = filt_Tx.filter(Tx);
        // Ty = filt_Ty.filter(Ty);
        // Tz = filt_Tz.filter(Tz);
    } else {
        std::cout << "Expected WKC: " << expectedWKC << " Received WKC: " << wkc << std::endl;
    }
}

void aidin_ec_wrapper::getIMUData(float& Ax, float& Ay, float& Az, float& Gx, float& Gy, float& Gz) {
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    
    // Process the data only if wkc is as expected
    if(wkc >= expectedWKC) {
        // Get IMU data from EtherCAT slaves
        float Ax_raw = *(ec_slave[0].inputs+33)*256 + *(ec_slave[0].inputs+32);  // Example mapping
        float Ay_raw = *(ec_slave[0].inputs+37)*256 + *(ec_slave[0].inputs+36);
        float Az_raw = *(ec_slave[0].inputs+41)*256 + *(ec_slave[0].inputs+40);
        float Gx_raw = *(ec_slave[0].inputs+45)*256 + *(ec_slave[0].inputs+44);
        float Gy_raw = *(ec_slave[0].inputs+49)*256 + *(ec_slave[0].inputs+48);
        float Gz_raw = *(ec_slave[0].inputs+53)*256 + *(ec_slave[0].inputs+52);

        Ax = (Ax_raw - 32767) / 1000.0;
        Ay = (Ay_raw - 32767) / 1000.0;
        Az = (Az_raw - 32767) / 1000.0;
        Gx = (Gx_raw - 32767) / 16.4;
        Gy = (Gy_raw - 32767) / 16.4;
        Gz = (Gz_raw - 32767) / 16.4;

        Ax = Ax * -0.7071068 + Ay * 0.7071068;
        Ay = Ay * -0.7071068 + Ax * -0.7071068;
        Az = Az;
        Gx = Gx * -0.7071068 - Gy * 0.7071068;
        Gy = Gy * -0.7071068 - Gx * -0.7071068;
        Gz = Gz;
    } else {
        std::cout << "Expected WKC: " << expectedWKC << " Received WKC: " << wkc << std::endl;
    }
}

void aidin_ec_wrapper::getTempData(float& Temp) {
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    
    // Process the data only if wkc is as expected
    if(wkc >= expectedWKC) {
        // Get IMU data from EtherCAT slaves
        Temp = *(ec_slave[0].inputs+57)*256 + *(ec_slave[0].inputs+56);
    } else {
        std::cout << "Expected WKC: " << expectedWKC << " Received WKC: " << wkc << std::endl;
    }
}
