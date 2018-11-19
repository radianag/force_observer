//
// Created by radian on 11/17/18.
//
#include "robot_dynamics.h"

RobotDynamics::RobotDynamics(){

}

Eigen::MatrixXd RobotDynamics::calcM(const Eigen::VectorXd &q){

    Eigen::MatrixXd M;
    M.resize(q.size(), q.size());

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    float t2 = q2*2.0;
    float t3 = t2-2.908373974667121E-1;
    float t4 = cos(t2);
    float t5 = sin(t2);
    float t6 = cos(2.908373974667121E-1);
    float t7 = q3*q3;
    float t8 = cos(t3);
    float t9 = cos(q2);
    float t10 = sin(q2);
    float t11 = q2-2.908373974667121E-1;
    float t12 = sin(t11);
    float t13 = t12*5.648514011831588E-3;
    float t14 = t9*3.233715128693166E-5;
    float t15 = t10*(-6.322333279804798E-3)+t13+t14-q3*t10*3.633350698549627E-2+6.036324572650495E-3;
    float t16 = sin(2.908373974667121E-1);
    float t17 = t9*3.633350698549627E-2;
    float t18 = t16*(-2.196176513085578E-3)+9.811510976268357E-3;
    M(0,0) = q3*(-1.370021252172674E-1)+t4*5.589503704456244E-2+t5*9.131214113774334E-3+t6*1.508771032382092E-2+t7*3.385335113329479E-1+t8*1.508771032382092E-2+t9*2.362985957563206E-2+t16*1.508766140985477E-2+cos(q2-5.816747949334241E-1)*2.362985957563206E-2-cos(t2-5.816747949334241E-1)*9.367754837668338E-3+sin(t3)*1.508766140985477E-2-q3*t4*1.370021252172674E-1+q3*t5*9.811510976268357E-3-q3*t6*2.196176513085578E-3-q3*t8*2.196176513085578E-3+t4*t7*3.385335113329479E-1+2.722195153879596E-1;
    M(0,1) = t15;
    M(0,2) = t17;
    M(1,0) = t15;
    M(1,1) = q3*(-2.740042504345347E-1)+t6*3.017542064764185E-2+t7*6.770670226658958E-1+t16*3.017532281970953E-2-q3*t6*4.392353026171156E-3+3.922469263646697E-1;
    M(1,2) = t18;
    M(2,0) = t17;
    M(2,1) = t18;
    M(2,2) = 6.770670226658958E-1;

    return M;
}

Eigen::VectorXd RobotDynamics::calcN(const Eigen::VectorXd &q, const Eigen::VectorXd &qd){

    // N is C and G matrix together
    Eigen::VectorXd N;
    N.resize(q.size());

    // This Hardcode needs to change
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double qd1 = qd(0);
    double qd2 = qd(1);
    double qd3 = qd(2);

    float t2 = q1-q2;
    float t3 = q1+q2;
    float t4 = qd2*qd2;
    float t5 = sin(t3);
    float t6 = sin(t2);
    float t7 = sin(q2);
    float t8 = q2*2.0;
    float t9 = t8-2.908373974667121E-1;
    float t10 = cos(t9);
    float t11 = cos(q2);
    float t12 = cos(t8);
    float t13 = sin(t8);
    float t14 = sin(t9);
    float t15 = cos(t2);
    float t16 = q1+q2-2.908373974667121E-1;
    float t17 = sin(t16);
    float t18 = t17*6.120266493943314E-1;
    float t19 = q1-q2+2.908373974667121E-1;
    float t20 = sin(t19);
    float t21 = t20*6.120266493943314E-1;
    float t22 = cos(t3);
    float t23 = t22*2.300749300157408E-2;
    float t24 = qd1*qd1;
    float t25 = t8-5.816747949334241E-1;
    float t26 = sin(t25);
    float t27 = q3*t5*7.174176609412888E-2;
    float t28 = q3*t6*7.174176609412888E-2;
    float t29 = q2-5.816747949334241E-1;
    float t30 = sin(t29);
    float t31 = cos(2.908373974667121E-1);
    float t32 = q3*q3;
    float t33 = cos(q1);
    float t34 = t11*t11;
    N(0) = t5*(-4.928652039114835E-1)-t6*4.928652039114835E-1-t15*2.300749300157408E-2+t18+t21+t23+t27+t28-t33*3.690362487729971E-1-sin(q1-2.908373974667121E-1)*7.71908746137314E-1-sin(q1+2.908373974667121E-1)*7.71908746137314E-1+sin(q1)*1.371930099108582-qd1*qd3*2.740042504345347E-1-t4*t7*6.467430257386333E-5-t4*t11*1.26446665596096E-2+t4*cos(q2-2.908373974667121E-1)*1.129702802366318E-2+q3*qd1*qd3*1.354134045331792-qd1*qd2*t7*4.725971915126412E-2-qd2*qd3*t7*1.453340279419851E-1+qd1*qd2*t10*6.035064563941906E-2-qd1*qd3*t10*4.392353026171156E-3+qd1*qd2*t12*3.652485645509734E-2-qd1*qd2*t13*2.235801481782498E-1-qd1*qd3*t12*2.740042504345347E-1-qd1*qd2*t14*6.035084129528369E-2+qd1*qd3*t13*1.962302195253671E-2+qd1*qd2*t26*3.747101935067335E-2-qd1*qd2*t30*4.725971915126412E-2-qd1*qd3*t31*4.392353026171156E-3-q3*t4*t11*7.266701397099254E-2+q3*qd1*qd2*t12*3.924604390507343E-2+q3*qd1*qd2*t13*5.480085008690695E-1+q3*qd1*qd3*t12*1.354134045331792+q3*qd1*qd2*t14*8.784706052342312E-3-qd1*qd2*t13*t32*1.354134045331792;
    N(1) = t5*(-4.928652039114835E-1)+t6*4.928652039114835E-1+t15*2.300749300157408E-2+t18-t21+t23+t27-t28-qd2*qd3*5.480085008690695E-1+t7*t24*2.362985957563206E-2-t10*t24*3.017532281970953E-2-t12*t24*1.826242822754867E-2+t13*t24*1.117900740891249E-1+t14*t24*3.017542064764185E-2-t24*t26*1.873550967533668E-2+t24*t30*2.362985957563206E-2+q3*qd2*qd3*2.708268090663583-qd2*qd3*t31*8.784706052342312E-3-q3*t12*t24*1.962302195253671E-2-q3*t13*t24*2.740042504345347E-1-q3*t14*t24*4.392353026171156E-3+t13*t24*t32*6.770670226658958E-1;
    N(2) = t4*2.740042504345347E-1-q3*t4*1.354134045331792+t4*t31*4.392353026171156E-3-t13*t24*9.811510976268357E-3-t11*t33*1.434835321882578E-1+t24*t34*2.740042504345347E-1-q3*t24*t34*1.354134045331792+t24*t31*t34*4.392353026171156E-3+t7*t11*t24*sin(2.908373974667121E-1)*4.392353026171156E-3;

    return N;
}

Eigen::VectorXd RobotDynamics::calcFr(const Eigen::VectorXd &q, const Eigen::VectorXd &qd){

    Eigen::VectorXd Fr;
    Fr.resize(q.size());

//    float x[3];
//
//    for (int i=0;i<3;i++)
//    {
//        if (abs(qd(i)) < deadband(i))
//        {
//            x[i] = 0;
//        }
//        else
//        {
//            if (i==2 & v_int(i) > qd(i))
//            {
//                // This makes it so that friction is only actuating while given a desired command
//                x[i] = v_int(i);
//            }
//            else
//            {
//                x[i] = qd(i);
//            }
//
//        }
//    }
//
//    float a1 =4.0E2;
//    float a2 =4.0E2;
//    float scale = 1;
//
//
//    float Fs_pos = 1;
//    float Fs_neg = -1;
//
//
//    //Impedance Controller
//    float x_e = joint_des(2) - joint_act(2);
//
//        //MidJuly_3dof fourier test 3
//        Fr(0) = x[0]*9.542485682180182E-2+1.0897508499757E-1/(exp(x[0]*-4.0E2)+1.0)+2.751172483473256E-1;
//        Fr(1) = x[1]*1.63259829973678E-1+1.631162536205055E-1/(exp(x[1]*-4.0E2)+1.0)+1.937201881157299E-1;
//
//
//        if(abs(qd(2)) < deadband(2))
//        {
//            if (x_e > pos_deadband)
//            {
//                Fr(2) = Fs_pos;
//            } else if (x_e < -pos_deadband)
//            {
//                Fr(2) = Fs_neg;
//            } else
//            {
//                Fr(2) = 0;
//            }
//
//        }
//        else {
//            // MidJuly_3dof
//            Fr(2) = x[2]*7.662943767346468E-1+1.069206734315578/(exp(x[2]*-4.0E2)+1.0)-3.845210662560725E-1;
//        }
//    Fr(2) = scale* Fr(2);
//
    return Fr;

}

Eigen::MatrixXd RobotDynamics::calcJd_imu(const Eigen::VectorXd &q, const Eigen::VectorXd &qd) {
    Eigen::MatrixXd Jd;
    Jd.resize(3, q.size());

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double qd1 = qd(0);
    double qd2 = qd(1);
    double qd3 = qd(2);

    // Kinematic Jd
    double t2 = 3.141592653589793*(1.0/2.0);
    double t3 = q1+t2;
    double t4 = q2-t2;
    double t5 = sin(t4);
    double t6 = sin(t3);
    double t7 = q3-1.23E2/2.5E2;
    double t8 = cos(t3);
    double t9 = cos(t4);
    Jd(0, 0) = -qd3*t5*t6-qd1*t5*t7*t8-qd2*t6*t7*t9;
    Jd(0, 1) = qd3*t8*t9-qd2*t5*t7*t8-qd1*t6*t7*t9;
    Jd(0, 2) = -qd1*t5*t6+qd2*t8*t9;
    Jd(1, 1) = qd3*t5+qd2*t7*t9;
    Jd(1, 2) = qd2*t5;
    Jd(2, 0) = qd3*t5*t8-qd1*t5*t6*t7+qd2*t7*t8*t9;
    Jd(2, 1) = qd3*t6*t9-qd2*t5*t6*t7+qd1*t7*t8*t9;
    Jd(2, 2) = qd1*t5*t8+qd2*t6*t9;

    return Jd;
}

Eigen::MatrixXd RobotDynamics::calcJd(const Eigen::VectorXd &q, const Eigen::VectorXd &qd) {
    Eigen::MatrixXd Jd;
    Jd.resize(3, q.size());

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double qd1 = qd(0);
    double qd2 = qd(1);
    double qd3 = qd(2);

    // Kinematic Jd
    double t2 = 3.141592653589793*(1.0/2.0);
    double t3 = q2-t2;
    double t4 = q1+t2;
    double t5 = sin(t4);
    double t6 = sin(t3);
    double t7 = cos(t4);
    double t8 = cos(t3);
    double t9 = q3-4.318E-1;
    Jd(0,0) = qd1*t6*t7*(-4.355E-1)-qd3*t5*t6-qd2*t5*t8*4.355E-1-qd1*t6*t7*t9-qd2*t5*t8*t9;
    Jd(0,1) = qd1*t5*t8*(-4.355E-1)-qd2*t6*t7*4.355E-1+qd3*t7*t8-qd1*t5*t8*t9-qd2*t6*t7*t9;
    Jd(0,2) = -qd1*t5*t6+qd2*t7*t8;
    Jd(1,1) = qd3*t6+qd2*t8*4.355E-1+qd2*t8*t9;
    Jd(1,2) = qd2*t6;
    Jd(2,0) = qd1*t5*t6*(-4.355E-1)+qd3*t6*t7+qd2*t7*t8*4.355E-1-qd1*t5*t6*t9+qd2*t7*t8*t9;
    Jd(2,1) = qd2*t5*t6*(-4.355E-1)+qd1*t7*t8*4.355E-1+qd3*t5*t8-qd2*t5*t6*t9+qd1*t7*t8*t9;
    Jd(2,2) = qd1*t6*t7+qd2*t5*t8;

    return Jd;
}

Eigen::MatrixXd RobotDynamics::calcJa_imu(const Eigen::VectorXd &q) {
    Eigen::MatrixXd Ja;
    Ja.resize(3, q.size());

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double t2 = 3.141592653589793*(1.0/2.0);
    double t3 = q1+t2;
    double t4 = q2-t2;
    double t5 = q3-1.23E2/2.5E2;
    double t6 = cos(t3);
    double t7 = sin(t4);
    double t8 = cos(t4);
    double t9 = sin(t3);
    Ja(0, 0) = -t5*t7*t9;
    Ja(0, 1) = t5*t6*t8;
    Ja(0, 2) = t6*t7;
    Ja(1, 1) = t5*t7;
    Ja(1, 2) = -t8;
    Ja(2, 0) = t5*t6*t7;
    Ja(2, 1) = t5*t8*t9;
    Ja(2, 2) = t7*t9;
    return Ja;
}

Eigen::MatrixXd RobotDynamics::calcJa(const Eigen::VectorXd &q) {
    Eigen::MatrixXd Ja;
    Ja.resize(3, q.size());

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double t2 = 3.141592653589793*(1.0/2.0);
    double t3 = q1+t2;
    double t4 = sin(t3);
    double t5 = q2-t2;
    double t6 = sin(t5);
    double t7 = cos(t3);
    double t8 = cos(t5);
    double t9 = q3-4.318E-1;
    Ja(0, 0) = t4*t6*(-4.355E-1)-t4*t6*t9;
    Ja(0, 1) = t7*t8*4.355E-1+t7*t8*t9;
    Ja(0, 2) = t6*t7;
    Ja(1, 1) = t6*4.355E-1+t6*t9;
    Ja(1, 2) = -t8;
    Ja(2, 0) = t6*t7*4.355E-1+t6*t7*t9;
    Ja(2, 1) = t4*t8*4.355E-1+t4*t8*t9;
    Ja(2, 2) = t4*t6;
    return Ja;
}