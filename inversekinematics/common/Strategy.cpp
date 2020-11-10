/**
 * @file /src/qt_ros_pkg/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date 2015/10/16
 *
 * @author shengru
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "Strategy.hpp"

using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
Strategy::Strategy()
{
    theta_r0.clear();
    theta_r1.clear();
    theta_r2.clear();
    theta_r3.clear();
    theta_r4.clear();

    theta_l0.clear();
    theta_l1.clear();
    theta_l2.clear();
    theta_l3.clear();
    theta_l4.clear();
}

Strategy::~Strategy()
{

}

void Strategy::CalculatePAndfi(float &P, float &fi)
{
    //P = sqrt(pow(ysin, 2) + pow(xcos, 2));
    //fi = atan2(ysin, xcos);// * 180 / M_PI;
}

void Strategy::DoIK()
{
    //    float R[3] = {0, -4, -31.85};
    //    float L[3] = {0, 4, -31.85};
    //    R_IK(classinfotest->parameters.R_theta, R);
    //    L_IK(classinfotest->parameters.L_theta, L);
    R_IK(classinfotest->parameters.R_theta, classinfotest->parameters.R_Goal);
    L_IK(classinfotest->parameters.L_theta, classinfotest->parameters.L_Goal);
    classinfotest->rosflag = true;
}

void Strategy::R_IK( float *theta, float *Goal )
{
    float p, fi, C3, S3;

    theta[0] = atan2((18.2 - Goal[2]), (-Goal[1]-4));
    theta[4] = theta[0];

    C3 = (((18.2-Goal[2])/sin(theta[0]))*((18.2-Goal[2])/sin(theta[0])) + (Goal[0])*(Goal[0]) - 312.5) / (312.5);
    S3 = sqrt(1-C3*C3);
    theta[2] = atan2(S3, C3);
    p = sqrt((12.5*cos(theta[2])+12.5)*(12.5*cos(theta[2])+12.5) + (156.25*sin(theta[2])*sin(theta[2])));
    fi = atan2(12.5*sin(theta[2]), 12.5*cos(theta[2])+12.5);
    theta[1] = atan2(((Goal[0])/p), sqrt(1 - ((Goal[0])/p)*((Goal[0])/p))) - fi;
    theta[3] = 0 - theta[1] - theta[2];
    theta[0] = 3.13727587 - theta[0];

//    theta[1] = theta[1] + 1.82569142;
//    theta[2] = theta[2] - 0.454453018;
//    theta[3] = theta[3] + 1.770354259;

    double sertyuiwo = theta[1];
    theta[1] = theta[3];
    theta[3] = sertyuiwo;

    theta_r0.push_back(theta[0]);
    theta_r1.push_back(theta[1]);
    theta_r2.push_back(theta[2]);
    theta_r3.push_back(theta[3]);
    theta_r4.push_back(theta[4]);
}

void Strategy::L_IK( float *theta, float *Goal )
{
    float p, fi, C3, S3;

    theta[0] = atan2((Goal[2]-18.2), (Goal[1]-4));
    theta[4] = 3.1519804335 + theta[0];

    C3 = (((Goal[2]-18.2)/sin(theta[0]))*((Goal[2]-18.2)/sin(theta[0])) + (Goal[0])*(Goal[0]) - 312.5) / (312.5);
    S3 = sqrt(1-C3*C3);
    theta[2] = atan2(S3, C3);
    p = sqrt((12.5*cos(theta[2]) + 12.5)*(12.5*cos(theta[2]) + 12.5) + (156.25*sin(theta[2])*sin(theta[2])));
    fi = atan2(12.5*sin(theta[2]), 12.5*cos(theta[2])+12.5);
    theta[1] = atan2(((-Goal[0])/p), sqrt(1 - ((Goal[0])/p)*((Goal[0])/p))) - fi;
    theta[3] = 0 - theta[1] - theta[2];
    if (theta[0] < 0)
        theta[0] = -theta[0];

//    theta[1] = theta[1] - 1.335607373 + PI;
//    theta[2] = theta[2] - 0.462082784;
//    theta[3] = theta[3] + 1.797690198;

    theta_l0.push_back(theta[0]);
    theta_l1.push_back(theta[1]);
    theta_l2.push_back(theta[2]);
    theta_l3.push_back(theta[3]);
    theta_l4.push_back(theta[4]);
}

string Strategy::FtoS(float value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}

void Strategy::SaveData()
{

    string savedText = "R0\tR1\t"
                       "R2\tR3\t"
                       "R4\tL0\t"
                       "L1\tL2\tL3\t"
                       "L4\n";
    char filename[]="/home/shengru/catkin_ws/src/soc/Parameter/IK_Record.ods";

    fstream fp;
    fp.open(filename, ios::out);

    fp<<savedText;

    for(int i = 0; i < theta_r0.size(); i++)
    {
        savedText = FtoS(theta_r0[i]) + "\t"
                + FtoS(theta_r1[i]) + "\t"
                + FtoS(theta_r2[i]) + "\t"
                + FtoS(theta_r3[i]) + "\t"
                + FtoS(theta_r4[i]) + "\t"
                + FtoS(theta_l0[i]) + "\t"
                + FtoS(theta_l1[i]) + "\t"
                + FtoS(theta_l2[i]) + "\t"
                + FtoS(theta_l3[i]) + "\t"
                + FtoS(theta_l4[i]) + "\n";

        fp<<savedText;
    }

    theta_r0.clear();
    theta_r1.clear();
    theta_r2.clear();
    theta_r3.clear();
    theta_r4.clear();

    theta_l0.clear();
    theta_l1.clear();
    theta_l2.clear();
    theta_l3.clear();
    theta_l4.clear();

    fp.close();

}
