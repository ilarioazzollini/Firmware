#include <IMBControl.hpp>

IMBControl::IMBControl(){
    zRoll.setZero(); 
    zPitch.setZero();
    zV = 0.0;
}

matrix::Matrix<double, 4, 1> IMBControl::update(matrix::Matrix<double, 12, 1> actualState,
                                                matrix::Matrix<double, 3, 1> setPoint){
    // States are Memorized As
    // 0:x / 1:y / 2:z / 3:xDot / 4:yDot / 5:zDot
    // 6:phi / 7:theta / 8:psi / 9:phiDot / 10:thetaDot / 11:psiDot

    matrix::Matrix<double, 4, 1> controlAction;
    
    // Tau X
    controlAction(0,0) = rollCtrl(actualState(1,0),     /*y*/
                                  actualState(4,0),     /*yDot*/
                                  actualState(6,0),     /*phi*/
                                  actualState(9,0),     /*phiDot*/
                                  setPoint(1,0));       /*yRef*/

    // Tau Y
    controlAction(1,0) = pitchCtrl(actualState(0,0),    /*x*/
                                   actualState(3,0),    /*xDot*/
                                   actualState(7,0),    /*theta*/
                                   actualState(10,0),   /*thetaDot*/
                                   setPoint(0,0));      /*xRef*/

    // Tau Z
    controlAction(2,0) = yawCtrl(actualState(8,0),      /*psi*/
                                 actualState(11,0));    /*psiDot*/

    // Thrust
    controlAction(3,0) = verticalCtrl(actualState(2,0), /*z*/
                                      actualState(5,0), /*zDot*/
                                      setPoint(2,0));   /*zRef*/

    return controlAction;
}

double IMBControl::rollCtrl(double y, double yDot, double phi, double phiDot, double yRef){
    matrix::Matrix<double, 3, 3> Ar;
    matrix::Matrix<double, 3, 1> Br;
    matrix::Matrix<double, 3, 3> Cr;
    matrix::Matrix<double, 3, 1> Dr;
    matrix::Matrix<double, 1, 7> Kr;

    // Parameters Initialization
    Ar(0,0) = 1;        Ar(0, 1) = 4e-3;    Ar(0, 2) = 8e-6;
    Ar(1,0) = 0;        Ar(1, 1) = 1;       Ar(1, 2) = 4e-3;
    Ar(2,0) = 0;        Ar(2, 1) = -17e-4;  Ar(2, 2) = 1;

    Br(0,0) = 1.6e-8;   Br(1,0) = 8e-6;     Br(2,0) = 4e-3;

    Cr(0,0) = 1;        Cr(0,1) = 2e-3;     Cr(0,2) = 4e-6;
    Cr(1,0) = 0;        Cr(1,1) = 1;        Cr(1,2) = 2e-3;
    Cr(2,0) = 0;        Cr(2,1) = -8.45e-4; Cr(2,2) = 1;

    Dr(0,0) = 8e-9;     Dr(1,0) = 4e-6;     Dr(2,0) = 2e-3;

    /*
    Kr(0,0) = 0.5771;  Kr(0,1) = 0.3486;   Kr(0,2) = 1.2558;
    Kr(0,3) = 0.2786;   Kr(0,4) = 0.1387;   Kr(0,5) = 0.2220;
    Kr(0,6) = 0.5648;*/

    /*
    Kr(0,0) = 0.3559;  Kr(0,1) = 0.2414;   Kr(0,2) = 0.9726;
    Kr(0,3) = 0.2426;   Kr(0,4) = 0.0695;   Kr(0,5) = 0.0983;
    Kr(0,6) = 0.3077;*/

    /*
    Kr(0,0) = 0.8697;  Kr(0,1) = 0.5287;   Kr(0,2) = 1.8339;
    Kr(0,3) = 0.3540;   Kr(0,4) = 0.1203;   Kr(0,5) = 0.2950;
    Kr(0,6) = 0.7921;*/

    Kr(0,0) = 0.8375;  Kr(0,1) = 0.5224;   Kr(0,2) = 1.8288;
    Kr(0,3) = 0.3540;   Kr(0,4) = 0.1203;   Kr(0,5) = 0.1586;
    Kr(0,6) = 0.7033;

    // Control Update
    matrix::Matrix<double, 3, 1> etaRoll;
    double e = y - yRef;

    etaRoll = Cr*zRoll + Dr*e;
    zRoll = Ar*zRoll + Br*e;

    matrix::Matrix<double, 7, 1> X;
    X(0,0) = y;             X(1,0) = yDot;          X(2,0) = phi;
    X(3,0) = phiDot;        X(4,0) = etaRoll(0,0);  X(5,0) = etaRoll(1,0);
    X(6,0) = etaRoll(2,0);

    return (-Kr*X)(0,0); // Tau X
}

double IMBControl::pitchCtrl(double x, double xDot, double theta, double thetaDot, double xRef){
    matrix::Matrix<double, 3, 3> Ap;
    matrix::Matrix<double, 3, 1> Bp;
    matrix::Matrix<double, 3, 3> Cp;
    matrix::Matrix<double, 3, 1> Dp;
    matrix::Matrix<double, 1, 7> Kp;

    // Parameters Initialization
    Ap(0,0) = 1;        Ap(0, 1) = 4e-3;    Ap(0, 2) = 8e-6;
    Ap(1,0) = 0;        Ap(1, 1) = 1;       Ap(1, 2) = 4e-3;
    Ap(2,0) = 0;        Ap(2, 1) = -0.0017; Ap(2, 2) = 1;

    Bp(0,0) = 1.6e-8;   Bp(1,0) = 8e-6;     Bp(2,0) = 4e-3;

    Cp(0,0) = 1;        Cp(0,1) = 2e-3;     Cp(0,2) = 4e-6;
    Cp(1,0) = 0;        Cp(1,1) = 1;        Cp(1,2) = 2e-3;
    Cp(2,0) = 0;        Cp(2,1) = -8.45e-4; Cp(2,2) = 1;

    Dp(0,0) = 8e-9;     Dp(1,0) = 4e-6;     Dp(2,0) = 2e-3;

    /*
    Kp(0,0) = -0.5781; Kp(0,1) = -0.3496;   Kp(0,2) = 1.2615;
    Kp(0,3) = 0.2804;   Kp(0,4) = -0.1387;   Kp(0,5) = -0.2218;
    Kp(0,6) = -0.5653;*/

    /*
    Kp(0,0) = -0.3566; Kp(0,1) = -0.2421;   Kp(0,2) = 0.9772;
    Kp(0,3) = 0.2442;   Kp(0,4) = -0.0695;   Kp(0,5) = -0.0982;
    Kp(0,6) = -0.3079;*/

    /*
    Kp(0,0) = -0.8778; Kp(0,1) = -0.5336;   Kp(0,2) = 1.8510;
    Kp(0,3) = 0.3573;   Kp(0,4) = -0.1214;   Kp(0,5) = -0.2978;
    Kp(0,6) = -0.7995;*/

    Kp(0,0) = -0.8453; Kp(0,1) = -0.5273;   Kp(0,2) = 1.8459;
    Kp(0,3) = 0.3573;   Kp(0,4) = -0.1214;   Kp(0,5) = -0.1601;
    Kp(0,6) = -0.7099;

    // Control Update
    matrix::Matrix<double, 3, 1> etaPitch;
    double e = x - xRef;

    etaPitch = Cp*zPitch + Dp*e;
    zPitch = Ap*zPitch + Bp*e;

    matrix::Matrix<double, 7, 1> X;
    X(0,0) = x;             X(1,0) = xDot;          X(2,0) = theta;
    X(3,0) = thetaDot;      X(4,0) = etaPitch(0,0); X(5,0) = etaPitch(1,0);
    X(6,0) = etaPitch(2,0);

    return (-Kp*X)(0,0); // Tau Y
}

double IMBControl::yawCtrl(double psi, double psiDot){
    matrix::Matrix<double, 1, 2> Ky;

    // Parameters Initialization
    Ky(0,0) = 0.3298;   Ky(0,1) = 0.2755;

    // Control Update
    matrix::Matrix<double, 2, 1> X;
    X(0,0) = psi;   X(1,0) = psiDot;

    return (-Ky*X)(0,0); // Tau Z
}

double IMBControl::verticalCtrl(double z, double zDot, double zRef){
    matrix::Matrix<double, 1, 3> Kv;

    double Av, Bv, Cv, Dv;

    // Parameters Initialization
    Av = 1.0; Bv = 0.004; Cv = 1.0; Dv = 0.002;

    Kv(0,0) = -22.2147; Kv(0,1) = -10.1404;   Kv(0,2) = -16.1535;

    // Control Update
    double etaV;
    double e = z - zRef;

    etaV = Cv*zV + Dv*e;
    zV = Av*zV + Bv*e;

    matrix::Matrix<double, 3, 1> X;
    X(0,0) = z; X(1,0) = zDot;  X(2,0) = etaV;

    return (-Kv*X)(0,0); // Thrust
}

void IMBControl::reset(){
    zRoll.setZero();
    zPitch.setZero();
    zV = 0.0;
}