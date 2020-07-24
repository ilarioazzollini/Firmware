#include <IMBControl.hpp>

IMBControl::IMBControl(){
    zRoll.setZero(); 
    zPitch.setZero();
}

matrix::Matrix<double, 4, 1> IMBControl::update(matrix::Matrix<double, 12, 1> actualState,
                                                matrix::Matrix<double, 3, 1> setPoint, double dt){
    // States are Memorized As
    // 0:x / 1:y / 2:z / 3:xDot / 4:yDot / 5:zDot
    // 6:phi / 7:theta / 8:psi / 9:phiDot / 10:thetaDot / 11:psiDot

    matrix::Matrix<double, 4, 1> controlAction;
    
    // Select Scheduling State
    /*if(dt < 2000){
        _state = ONE;
    } else if(dt >= 2000 && dt < 3000){
        _state = TWO;
    } else if(dt >= 3000 && dt < 4000){
        _state = THREE;
    } else if(dt >= 4000 && dt < 5000){
        _state = FOUR;
    } else if(dt >= 5000 && dt < 6000){
        _state = FIVE;
    } else if(dt >= 6000){
        _state = SIX;
    }*/
/*
    if(dt < 1500){
        _state = ONE;
    } else if(dt >= 1500 && dt < 2500){
        _state = TWO;
    } else if(dt >= 2500 && dt < 3500){
        _state = THREE;
    } else if(dt >= 3500 && dt < 4500){
        _state = FOUR;
    } else if(dt >= 4500 && dt < 5500){
        _state = FIVE;
    } else if(dt >= 5500){
        _state = SIX;
    }

    _state = FOUR;*/
    
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

    // Parameters Initialization Depending on DT
    /*switch(_state){
        case ONE:
        Ar(0,0) = 1;        Ar(0, 1) = 1e-3;        Ar(0, 2) = 5e-7;
        Ar(1,0) = 0;        Ar(1, 1) = 1;           Ar(1, 2) = 1e-3;
        Ar(2,0) = 0;        Ar(2, 1) = -4.225e-4;   Ar(2, 2) = 1;

        Br(0,0) = 2.5e-10;   Br(1,0) = 5e-7;     Br(2,0) = 1e-3;

        Cr(0,0) = 1;        Cr(0,1) = 5e-4;         Cr(0,2) = 2.5e-7;
        Cr(1,0) = 0;        Cr(1,1) = 1;            Cr(1,2) = 5e-4;
        Cr(2,0) = 0;        Cr(2,1) = -2.1125e-4;   Cr(2,2) = 1;

        Dr(0,0) = 1.25e-10;     Dr(1,0) = 2.5e-7;     Dr(2,0) = 5e-4;

        Kr(0,0) = 0.8509;   Kr(0,1) = 0.5309;   Kr(0,2) = 1.8550;
        Kr(0,3) = 0.3578;   Kr(0,4) = 0.1225;   Kr(0,5) = 0.1621;
        Kr(0,6) = 0.7159;
        break;

        case TWO:
        Ar(0,0) = 1;        Ar(0, 1) = 2e-3;        Ar(0, 2) = 2e-6;
        Ar(1,0) = 0;        Ar(1, 1) = 1;           Ar(1, 2) = 2e-3;
        Ar(2,0) = 0;        Ar(2, 1) = -8.450e-4;   Ar(2, 2) = 1;

        Br(0,0) = 2.0e-9;   Br(1,0) = 2e-6;     Br(2,0) = 2e-3;

        Cr(0,0) = 1;        Cr(0,1) = 1e-3;         Cr(0,2) = 1e-6;
        Cr(1,0) = 0;        Cr(1,1) = 1;            Cr(1,2) = 1e-3;
        Cr(2,0) = 0;        Cr(2,1) = -4.2250e-4;   Cr(2,2) = 1;

        Dr(0,0) = 1e-9;     Dr(1,0) = 1e-6;     Dr(2,0) = 1e-3;

        Kr(0,0) = 0.8464;   Kr(0,1) = 0.5280;   Kr(0,2) = 1.8463;
        Kr(0,3) = 0.3565;   Kr(0,4) = 0.1217;   Kr(0,5) = 0.1609;
        Kr(0,6) = 0.7117;
        break;

        case THREE:
        Ar(0,0) = 1;        Ar(0, 1) = 1e-3;        Ar(0, 2) = 4.5e-6;
        Ar(1,0) = 0;        Ar(1, 1) = 1;           Ar(1, 2) = 1e-3;
        Ar(2,0) = 0;        Ar(2, 1) = -13e-4;      Ar(2, 2) = 1;

        Br(0,0) = 6.75e-9;   Br(1,0) = 4.5e-6;     Br(2,0) = 3e-3;

        Cr(0,0) = 1;        Cr(0,1) = 15e-4;        Cr(0,2) = 2.25e-6;
        Cr(1,0) = 0;        Cr(1,1) = 1;            Cr(1,2) = 15e-4;
        Cr(2,0) = 0;        Cr(2,1) = -6.3375e-4;   Cr(2,2) = 1;

        Dr(0,0) = 3.375e-9;     Dr(1,0) = 2.25e-6;     Dr(2,0) = 15e-4;

        Kr(0,0) = 0.8419;   Kr(0,1) = 0.5252;   Kr(0,2) = 1.8375;
        Kr(0,3) = 0.3553;   Kr(0,4) = 0.1210;   Kr(0,5) = 0.1597;
        Kr(0,6) = 0.7075;
        break;

        case FOUR:
        Ar(0,0) = 1;        Ar(0, 1) = 4e-3;        Ar(0, 2) = 8e-6;
        Ar(1,0) = 0;        Ar(1, 1) = 1;           Ar(1, 2) = 4e-3;
        Ar(2,0) = 0;        Ar(2, 1) = -17e-4;      Ar(2, 2) = 1;

        Br(0,0) = 1.6e-8;   Br(1,0) = 8e-6;     Br(2,0) = 4e-3;

        Cr(0,0) = 1;        Cr(0,1) = 2e-3;         Cr(0,2) = 4e-6;
        Cr(1,0) = 0;        Cr(1,1) = 1;            Cr(1,2) = 2e-3;
        Cr(2,0) = 0;        Cr(2,1) = -8.45e-4;     Cr(2,2) = 1;

        Dr(0,0) = 8e-9;     Dr(1,0) = 4e-6;     Dr(2,0) = 0.002;

        Kr(0,0) = 0.8375;   Kr(0,1) = 0.5224;   Kr(0,2) = 1.8288;
        Kr(0,3) = 0.3540;   Kr(0,4) = 0.1203;   Kr(0,5) = 0.1586;
        Kr(0,6) = 0.7033;
        break;

        case FIVE:
        Ar(0,0) = 1;        Ar(0, 1) = 5e-3;        Ar(0, 2) = 1.25e-5;
        Ar(1,0) = 0;        Ar(1, 1) = 1;           Ar(1, 2) = 5e-3;
        Ar(2,0) = 0;        Ar(2, 1) = -0.0021;   Ar(2, 2) = 1;

        Br(0,0) = 3.125e-8; Br(1,0) = 1.25e-5;     Br(2,0) = 5e-3;

        Cr(0,0) = 1;        Cr(0,1) = 2.5e-3;       Cr(0,2) = 6.25e-6;
        Cr(1,0) = 0;        Cr(1,1) = 1;            Cr(1,2) = 2.5e-3;
        Cr(2,0) = 0;        Cr(2,1) = -0.0011;      Cr(2,2) = 1;

        Dr(0,0) = 1.5625e-8;     Dr(1,0) = 6.25e-6;     Dr(2,0) = 25e-4;

        Kr(0,0) = 0.8331;   Kr(0,1) = 0.5197;   Kr(0,2) = 1.8202;
        Kr(0,3) = 0.3528;   Kr(0,4) = 0.1195;   Kr(0,5) = 0.1574;
        Kr(0,6) = 0.6992;
        break;

        case SIX:
        Ar(0,0) = 1;        Ar(0, 1) = 6e-3;        Ar(0, 2) = 1.8e-5;
        Ar(1,0) = 0;        Ar(1, 1) = 1;           Ar(1, 2) = 6e-3;
        Ar(2,0) = 0;        Ar(2, 1) = -0.0025;     Ar(2, 2) = 1;

        Br(0,0) = 5.4e-8;   Br(1,0) = 1.8e-5;       Br(2,0) = 6e-3;

        Cr(0,0) = 1;        Cr(0,1) = 3e-4;         Cr(0,2) = 9e-6;
        Cr(1,0) = 0;        Cr(1,1) = 1;            Cr(1,2) = 3e-3;
        Cr(2,0) = 0;        Cr(2,1) = -13e-3;       Cr(2,2) = 1;

        Dr(0,0) = 2.7e-8;     Dr(1,0) = 9e-6;     Dr(2,0) = 3e-3;

        Kr(0,0) = 0.8287;   Kr(0,1) = 0.5169;   Kr(0,2) = 1.8117;
        Kr(0,3) = 0.3515;   Kr(0,4) = 0.1188;   Kr(0,5) = 0.1563;
        Kr(0,6) = 0.6950;
        break;
    }*/
/*
    Kr(0,0) = 0.1150;   Kr(0,1) = 0.1251;   Kr(0,2) = 0.7142;
    Kr(0,3) = 0.2209;   Kr(0,4) = 0.0049;   Kr(0,5) = -0.0156;
    Kr(0,6) = 0.0404;
*/

    Ar(0,0) = 1;        Ar(0, 1) = 4e-3;        Ar(0, 2) = 8e-6;
        Ar(1,0) = 0;        Ar(1, 1) = 1;           Ar(1, 2) = 4e-3;
        Ar(2,0) = 0;        Ar(2, 1) = -17e-4;      Ar(2, 2) = 1;

        Br(0,0) = 1.6e-8;   Br(1,0) = 8e-6;     Br(2,0) = 4e-3;

        Cr(0,0) = 1;        Cr(0,1) = 2e-3;         Cr(0,2) = 4e-6;
        Cr(1,0) = 0;        Cr(1,1) = 1;            Cr(1,2) = 2e-3;
        Cr(2,0) = 0;        Cr(2,1) = -8.45e-4;     Cr(2,2) = 1;

        Dr(0,0) = 8e-9;     Dr(1,0) = 4e-6;     Dr(2,0) = 0.002;

    Kr(0,0) = 0.1195;   Kr(0,1) = 0.1299;   Kr(0,2) = 0.7384;
    Kr(0,3) = 0.2264;   Kr(0,4) = 0.0051;   Kr(0,5) = -0.0162;
    Kr(0,6) = 0.0423;


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

    // Parameters Initialization Depending on DT
 /*   switch(_state){
        case ONE:
        Ap(0,0) = 1;        Ap(0, 1) = 1e-3;        Ap(0, 2) = 5e-7;
        Ap(1,0) = 0;        Ap(1, 1) = 1;           Ap(1, 2) = 1e-3;
        Ap(2,0) = 0;        Ap(2, 1) = -4.225e-4;   Ap(2, 2) = 1;

        Bp(0,0) = 2.5e-10;   Bp(1,0) = 5e-7;     Bp(2,0) = 1e-3;

        Cp(0,0) = 1;        Cp(0,1) = 5e-4;         Cp(0,2) = 2.5e-7;
        Cp(1,0) = 0;        Cp(1,1) = 1;            Cp(1,2) = 5e-4;
        Cp(2,0) = 0;        Cp(2,1) = -2.1125e-4;   Cp(2,2) = 1;

        Dp(0,0) = 1.25e-10;     Dp(1,0) = 2.5e-7;     Dp(2,0) = 5e-4;

        Kp(0,0) = -0.8589;  Kp(0,1) = -0.5358;   Kp(0,2) = 1.8724;
        Kp(0,3) = 0.3611;   Kp(0,4) = -0.1236;   Kp(0,5) = -0.1636;
        Kp(0,6) = -0.7226;
        break;

        case TWO:
        Ap(0,0) = 1;        Ap(0, 1) = 2e-3;        Ap(0, 2) = 2e-6;
        Ap(1,0) = 0;        Ap(1, 1) = 1;           Ap(1, 2) = 2e-3;
        Ap(2,0) = 0;        Ap(2, 1) = -8.450e-4;   Ap(2, 2) = 1;

        Bp(0,0) = 2.0e-9;   Bp(1,0) = 2e-6;     Bp(2,0) = 2e-3;

        Cp(0,0) = 1;        Cp(0,1) = 1e-3;         Cp(0,2) = 1e-6;
        Cp(1,0) = 0;        Cp(1,1) = 1;            Cp(1,2) = 1e-3;
        Cp(2,0) = 0;        Cp(2,1) = -4.2250e-4;   Cp(2,2) = 1;

        Dp(0,0) = 1e-9;     Dp(1,0) = 1e-6;     Dp(2,0) = 1e-3;

        Kp(0,0) = -0.8543;  Kp(0,1) = -0.5330;   Kp(0,2) = 1.8635;
        Kp(0,3) = 0.3599;   Kp(0,4) = -0.1249;   Kp(0,5) = -0.1624;
        Kp(0,6) = -0.7183;
        break;

        case THREE:
        Ap(0,0) = 1;        Ap(0, 1) = 1e-3;        Ap(0, 2) = 4.5e-6;
        Ap(1,0) = 0;        Ap(1, 1) = 1;           Ap(1, 2) = 1e-3;
        Ap(2,0) = 0;        Ap(2, 1) = -13e-4;      Ap(2, 2) = 1;

        Bp(0,0) = 6.75e-9;   Bp(1,0) = 4.5e-6;     Bp(2,0) = 3e-3;

        Cp(0,0) = 1;        Cp(0,1) = 15e-4;        Cp(0,2) = 2.25e-6;
        Cp(1,0) = 0;        Cp(1,1) = 1;            Cp(1,2) = 15e-4;
        Cp(2,0) = 0;        Cp(2,1) = -6.3375e-4;   Cp(2,2) = 1;

        Dp(0,0) = 3.375e-9;     Dp(1,0) = 2.25e-6;     Dp(2,0) = 15e-4;

        Kp(0,0) = -0.8498;  Kp(0,1) = -0.5301;   Kp(0,2) = 1.8547;
        Kp(0,3) = 0.3586;   Kp(0,4) = -0.1221;   Kp(0,5) = -0.1612;
        Kp(0,6) = -0.7141;
        break;

        case FOUR:
        Ap(0,0) = 1;        Ap(0, 1) = 4e-3;        Ap(0, 2) = 8e-6;
        Ap(1,0) = 0;        Ap(1, 1) = 1;           Ap(1, 2) = 4e-3;
        Ap(2,0) = 0;        Ap(2, 1) = -17e-4;      Ap(2, 2) = 1;

        Bp(0,0) = 1.6e-8;   Bp(1,0) = 8e-6;     Bp(2,0) = 4e-3;

        Cp(0,0) = 1;        Cp(0,1) = 2e-3;         Cp(0,2) = 4e-6;
        Cp(1,0) = 0;        Cp(1,1) = 1;            Cp(1,2) = 2e-3;
        Cp(2,0) = 0;        Cp(2,1) = -8.45e-4;     Cp(2,2) = 1;

        Dp(0,0) = 8e-9;     Dp(1,0) = 4e-6;     Dp(2,0) = 0.002;

        Kp(0,0) = -0.8453;  Kp(0,1) = -0.5273;   Kp(0,2) = 1.8459;
        Kp(0,3) = 0.3573;   Kp(0,4) = -0.1214;   Kp(0,5) = -0.1601;
        Kp(0,6) = -0.7099;
        break;

        case FIVE:
        Ap(0,0) = 1;        Ap(0, 1) = 5e-3;        Ap(0, 2) = 1.25e-5;
        Ap(1,0) = 0;        Ap(1, 1) = 1;           Ap(1, 2) = 5e-3;
        Ap(2,0) = 0;        Ap(2, 1) = -0.0021;   Ap(2, 2) = 1;

        Bp(0,0) = 3.125e-8; Bp(1,0) = 1.25e-5;     Bp(2,0) = 5e-3;

        Cp(0,0) = 1;        Cp(0,1) = 2.5e-3;       Cp(0,2) = 6.25e-6;
        Cp(1,0) = 0;        Cp(1,1) = 1;            Cp(1,2) = 2.5e-3;
        Cp(2,0) = 0;        Cp(2,1) = -0.0011;      Cp(2,2) = 1;

        Dp(0,0) = 1.5625e-8;     Dp(1,0) = 6.25e-6;     Dp(2,0) = 25e-4;

        Kp(0,0) = -0.8408;  Kp(0,1) = -0.5245;   Kp(0,2) = 1.8372;
        Kp(0,3) = 0.3561;   Kp(0,4) = -0.1206;   Kp(0,5) = -0.1589;
        Kp(0,6) = -0.7057;
        break;

        case SIX:
        Ap(0,0) = 1;        Ap(0, 1) = 6e-3;        Ap(0, 2) = 1.8e-5;
        Ap(1,0) = 0;        Ap(1, 1) = 1;           Ap(1, 2) = 6e-3;
        Ap(2,0) = 0;        Ap(2, 1) = -0.0025;     Ap(2, 2) = 1;

        Bp(0,0) = 5.4e-8;   Bp(1,0) = 1.8e-5;       Bp(2,0) = 6e-3;

        Cp(0,0) = 1;        Cp(0,1) = 3e-4;         Cp(0,2) = 9e-6;
        Cp(1,0) = 0;        Cp(1,1) = 1;            Cp(1,2) = 3e-3;
        Cp(2,0) = 0;        Cp(2,1) = -13e-3;       Cp(2,2) = 1;

        Dp(0,0) = 2.7e-8;     Dp(1,0) = 9e-6;     Dp(2,0) = 3e-3;

        Kp(0,0) = -0.8364;  Kp(0,1) = -0.5217;   Kp(0,2) = 1.8286;
        Kp(0,3) = 0.3548;   Kp(0,4) = -0.1199;   Kp(0,5) = -0.1578;
        Kp(0,6) = -0.7015;
        break;
    }*/
    /*
    Kp(0,0) = -0.1160;  Kp(0,1) = -0.1262;   Kp(0,2) = 0.7209;
    Kp(0,3) = 0.2230;   Kp(0,4) = -0.0050;   Kp(0,5) = 0.0158;
    Kp(0,6) = -0.0407; */

    Ap(0,0) = 1;        Ap(0, 1) = 4e-3;        Ap(0, 2) = 8e-6;
        Ap(1,0) = 0;        Ap(1, 1) = 1;           Ap(1, 2) = 4e-3;
        Ap(2,0) = 0;        Ap(2, 1) = -17e-4;      Ap(2, 2) = 1;

        Bp(0,0) = 1.6e-8;   Bp(1,0) = 8e-6;     Bp(2,0) = 4e-3;

        Cp(0,0) = 1;        Cp(0,1) = 2e-3;         Cp(0,2) = 4e-6;
        Cp(1,0) = 0;        Cp(1,1) = 1;            Cp(1,2) = 2e-3;
        Cp(2,0) = 0;        Cp(2,1) = -8.45e-4;     Cp(2,2) = 1;

        Dp(0,0) = 8e-9;     Dp(1,0) = 4e-6;     Dp(2,0) = 0.002;

    Kp(0,0) = -0.1207;  Kp(0,1) = -0.1311;   Kp(0,2) = 0.7453;
    Kp(0,3) = 0.2285;   Kp(0,4) = -0.0052;   Kp(0,5) = 0.0163;
    Kp(0,6) = -0.0427;

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
/*
    // Parameters Initialization Depending on DT
    switch(_state){
        case ONE:
        Ky(0,0) = 0.3323;   Ky(0,1) = 0.2771;
        break;

        case TWO:
        Ky(0,0) = 0.3315;   Ky(0,1) = 0.2765;
        break;

        case THREE:
        Ky(0,0) = 0.3306;   Ky(0,1) = 0.2760;
        break;

        case FOUR:
        Ky(0,0) = 0.3298;   Ky(0,1) = 0.275;
        break;

        case FIVE:
        Ky(0,0) = 0.3290;   Ky(0,1) = 0.2750;
        break;

        case SIX:
        Ky(0,0) = 0.3282;   Ky(0,1) = 0.2745;
        break;
    }*/

    //Ky(0,0) = 0.3209;   Ky(0,1) = 0.2698;
    Ky(0,0) = 0.3298;   Ky(0,1) = 0.2755;

    // Control Update
    matrix::Matrix<double, 2, 1> X;
    X(0,0) = psi;   X(1,0) = psiDot;

    return (-Ky*X)(0,0); // Tau Z
}

double IMBControl::verticalCtrl(double z, double zDot, double zRef){
    matrix::Matrix<double, 1, 3> Kv;

    double Av = 0.0, Bv = 0.0, Cv = 0.0, Dv = 0.0;

    // Parameters Initialization Depending on DT
/*    switch(_state){
        case ONE:
        Av = 1.0; Bv = 1e-3; Cv = 1.0; Dv = 5e-4;
        Kv(0,0) = -6.6247; Kv(0,1) = -5.5733;   Kv(0,2) = -2.5993;
        break;

        case TWO:
        Av = 1.0; Bv = 2e-3; Cv = 1.0; Dv = 1e-3;
        Kv(0,0) = -6.6154; Kv(0,1) = -5.5666;   Kv(0,2) = -2.5946;
        break;

        case THREE:
        Av = 1.0; Bv = 3e-3; Cv = 1.0; Dv = 1.5e-3;
        Kv(0,0) = -6.6061; Kv(0,1) = -5.5599;   Kv(0,2) = -2.5900;
        break;

        case FOUR:
        Av = 1.0; Bv = 4e-3; Cv = 1.0; Dv = 2e-3;
        Kv(0,0) = -6.5968; Kv(0,1) = -5.5532;   Kv(0,2) = -2.5853;
        break;

        case FIVE:
        Av = 1.0; Bv = 5e-3; Cv = 1.0; Dv = 2.5e-3;
        Kv(0,0) = -6.5875; Kv(0,1) = -5.5465;   Kv(0,2) = -2.5807;
        break;

        case SIX:
        Av = 1.0; Bv = 6e-3; Cv = 1.0; Dv = 3e-3;
        Kv(0,0) = -6.5783; Kv(0,1) = -5.5399;   Kv(0,2) = -2.5760;
        break;
    }*/
    Av = 1.0; Bv = 4e-3; Cv = 1.0; Dv = 2e-3;
    //Kv(0,0) = -6.4957; Kv(0,1) = -5.4804;   Kv(0,2) = -2.5347;
    Kv(0,0) = -6.5968; Kv(0,1) = -5.5532;   Kv(0,2) = -2.5853;

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