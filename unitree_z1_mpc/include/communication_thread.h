#ifndef _COMUNC_TH_
#define _COMUNC_TH_

#include "unitree_arm_sdk/control/unitreeArm.h"
#include "mathTypes.h"
using namespace UNITREE_ARM;
using vector_t=Eigen::Matrix<double, Eigen::Dynamic, 1>;
class z1_interface{
    public:
        z1_interface(unitreeArm &arm);
        void back_to_start();
        void run_over();
        void update_joint_target(std::vector<double> solution);//policy recieve call back
        void data_swap_Run();
        ~z1_interface(){};

        vector_t rcv_cur_state;
    private:
        unitreeArm& z1_arm_;
        std::vector<double> KP, KW;
        bool z1_enable=false;
};

#endif