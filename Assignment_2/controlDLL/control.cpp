// controlDLL.cpp : Defines the entry point for the DLL application.
//
#include "servo.h"
#include "param.h"
#include "control.h"
#include "UiAgent.h"

#include "PrVector.h"
#include "PrMatrix.h"
#include "param.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
using std::min;
using std::max;



void PrintDebug(GlobalVariables& gv);

// *******************************************************************
// Spline generation functions
// *******************************************************************

struct CubicSpline {
    double t0 , tf ;
    PrVector3 a0 , a1 , a2 , a3 ;
};

//Global spline parameters;
PrVector3 p0(0.,0.,0.), p1(10.2 * M_PI/180, 52.2 * M_PI/180, -62.4 * M_PI/180);//p1(10.2 * M_PI / 180., 52.2 * M_PI / 180., -62.4 * M_PI / 180.);
CubicSpline m_spline;

//Global parameters for the inverse kinematics
PrVector3 F_q, x_d, vel_d, x_curr, vel_curr;
PrMatrix3 Jacobian;
double x_0, y_0, x_init, y_init, alpha_init, r;
double traj_max_vel, traj_accl;
double traj_t0, traj_tf, traj_tb;
double traj_tb1, traj_tb2, traj_tb3;


// Compute total trajectory length
//TODO this method is for the blend time calculation, we need to implement the new one for the cubic polynom
double computeTf ( GlobalVariables & gv)
{
    double tf1, tf2, tf3;
    /*
     * -First we need to define if the angle difference between the end and start point is far enough, to allow the joint to reach the maximum velocity
     * -If not, we need to calculate the maximum reachable speed inside the acceleration boundary, then average the velocity and calculate the time required
     * -If yes, we calculate the time for acceleration, then the time for the maximum velocity interval and finally summarize
     * -Define the return value for the largest needed time constraint to synchronise the joints later
     */
    double dq1, dq2, dq3, ddq1, ddq2, ddq3;
    PrVector3 S_max(0, 0, 0);
    if (gv.dof == 3) {
        S_max[0] = gv.dqmax[0] * gv.dqmax[0] / gv.ddqmax[0];
        S_max[1] = gv.dqmax[1] * gv.dqmax[1] / gv.ddqmax[1];
        S_max[2] = gv.dqmax[2] * gv.dqmax[2] / gv.ddqmax[2];

        dq1 = gv.dqmax[0];
        dq2 = gv.dqmax[1];
        dq3 = gv.dqmax[2];
        ddq1 = gv.ddqmax[0];
        ddq2 = gv.ddqmax[1];
        ddq3 = gv.ddqmax[2];

    } else if (gv.dof == 6) {
        S_max[0] = gv.dqmax[1] * gv.dqmax[1] / gv.ddqmax[1];
        S_max[1] = gv.dqmax[2] * gv.dqmax[2] / gv.ddqmax[2];
        S_max[2] = gv.dqmax[4] * gv.dqmax[4] / gv.ddqmax[4];

        dq1 = gv.dqmax[0];
        dq2 = gv.dqmax[2];
        dq3 = gv.dqmax[4];
        ddq1 = gv.ddqmax[0];
        ddq2 = gv.ddqmax[2];
        ddq3 = gv.ddqmax[4];
    } else {
        //To be sure it won't be nullpointer
        S_max[0] = S_max[1] = S_max[2] = 0.;
    }

    //TF1
    if (S_max[0] > abs(p1[0] - p0[0])) {
        tf1 = 2 * abs(p1[0] - p0[0]) / (sqrt(abs(p1[0] - p0[0]) / ddq1));
    } else {
        tf1 = 2 * dq1 / ddq1 + (abs(p1[0] - p0[0]) - S_max[0]) / dq1;
    }

    //TF2
    if (S_max[1] > abs(p1[1] - p0[1])) {
        tf2 = 2 * abs(p1[1] - p0[1]) / (sqrt(abs(p1[1] - p0[1]) / ddq2));
    } else {
        tf2 = 2 * dq2 / ddq2 + (abs(p1[1] - p0[1]) - S_max[1]) / dq2;
    }

    //TF3
    if (S_max[2] > abs(p1[2] - p0[2])) {
        tf3 = 2 * abs(p1[2] - p0[2]) / (sqrt(abs(p1[2] - p0[2]) / ddq3));
    } else {
        tf3 = 2 * dq3 / ddq3 + (abs(p1[2] - p0[2]) - S_max[2]) / dq3;
    }

    return  max(tf1, max(tf2, tf3));
}

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
   // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables& gv)
{
   // This code runs on every servo loop, just before the control law
   
   
  // 
    if ((gv.dof == 3) || (gv.dof == 6)) {

        //get the correct joint angles depending on the current mode:
        double q1,q2,q3;
        if (gv.dof == 3) {
            q1 = gv.q[0];
            q2 = gv.q[1];
            q3 = gv.q[2];
        } else if (gv.dof == 6) {
            q1 = gv.q[1];
            q2 = gv.q[2];
            q3 = gv.q[4];
        } else{
            //To be sure it won't be nullpointer
            q1 = q2 = q3 = 0.;
        }

        PrVector3 g123 = PrVector3(0,0,0); //Variable that holds the torque exerted by gravity for each joint

        //Compute g123 here!

        //Setting up the parameters for our configuration
        static double r1 = R2;
        static double r2 = 0.189738;
        static double r3 = R6;

        static double l1 = L2;
        static double l2 = L3;
        static double l3 = L6;

        static double m1 = M2;
        static double m2 = M3 + M4 + M5;
        static double m3 = M6;

        static double g = -9.81;

        //Calculating the distance of the centre of masses from the 1,2,3 joints for gravity torque calclation
        //First index refers to the joint, second index to the centre of mass of the links to the right
        double g21 = r3 * sin(q1 + q2 + q3);
        double g11 = r2 * sin(q1 + q2);
        double g12 = l2 * sin(q1 + q2) + g21;
        double g01 = r1 * cos(q1);
        double g02 = l1 * cos(q1) + g11;
        double g03 = l1 * cos(q1) + g12 + g21;

        g123[0] = g01 * m1 * g + g02 * m2 * g + g03 * m3 * g;
        g123[1] = g12 * m3 * g + g11 * m2;
        g123[2] = g12 * m3 * g;

        //maps the torques to the right joint indices depending on the current mode:
        if (gv.dof == 3) {
            gv.G[0] = g123[0];
            gv.G[1] = g123[1];
            gv.G[2] = g123[2];
        } else if (gv.dof == 6) {
            gv.G[1] = g123[0];
            gv.G[2] = g123[1];
            gv.G[4] = g123[2];
        }

//        printVariable(g123, "g123");
    } else {
        gv.G = PrVector(gv.G.size());
    }   
}

void PostprocessControl(GlobalVariables& gv) 
{
   // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) 
{
    // Control Initialization Code Here
}

void initOpenControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initJgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjtrackControl(GlobalVariables& gv) 
{
    /*
     * -First we need to calculate the required time to follow the trajectory inside the boundaries
     * -Then we use the given formula for calculating the a_i parameters and save it into our global variable
     */

    double a2_q1, a2_q2, a2_q3, a3_q1, a3_q2, a3_q3;
    m_spline.t0=gv.curTime;
    m_spline.tf=gv.curTime + computeTf(gv);

    m_spline.a0 = PrVector3(0., 0., 0.);
    m_spline.a1 = PrVector3(0., 0., 0.);


    a2_q1 = 3.0 / pow((m_spline.tf-m_spline.t0), 2) * (p1[0]-p0[0]);
    a3_q1 = - 2.0 / pow((m_spline.tf-m_spline.t0), 3) * (p1[0]-p0[0]);

    a2_q2 = 3.0 / pow((m_spline.tf-m_spline.t0), 2) * (p1[1]-p0[1]);
    a3_q2 = - 2.0 / pow((m_spline.tf-m_spline.t0), 3) * (p1[1]-p0[1]);


    a2_q3 = 3.0 / pow((m_spline.tf-m_spline.t0), 2) * (p1[2]-p0[2]);
    a3_q3 = - 2.0 / pow((m_spline.tf-m_spline.t0), 3) * (p1[2]-p0[2]);

    m_spline.a2 = PrVector3(a2_q1, a2_q2, a2_q3);
    m_spline.a3 = PrVector3(a3_q1, a3_q2, a3_q3);

}

void initJtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initXtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initHoldControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initGotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initTrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initPfmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initLineControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj1Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
    
}

void initProj2Control(GlobalVariables& gv) 
{
	//Here we initialize the required variables for our trajectory control
	traj_max_vel = 2 * M_PI / 5;
	traj_t0 = gv.curTime;
	//traj_tf = gv.curTime + 2 * M_PI / traj_max_vel;
	r = 0.2;
	x_0 = 0.6;
	y_0 = 0.3;

    x_init = 0.8;
    y_init = 0.3;
    alpha_init = 0;
    //gv.dx = PrVector3 (x_init,y_init,alpha_init) ;
    
    double q1 = gv.q[0];
    double q2 = gv.q[1];
    double q3 = gv.q[2];
	/*
    double x11, x12, x13, x21, x22, x23, x31, x32, x33;

	x11 = - L1 * sin(q1) + L2 * cos(q1 +q2) + L3 * cos(q1 +q2 +q3);
	x12 = L2 * cos(q1 +q2) + L3 * cos(q1 +q2 +q3);
	x13 = L3 * cos(q1 +q2 +q3);

	x21 = L1 * cos (q1) + L2 * sin(q1 +q2) + L3 * sin(q1 +q2 +q3);
	x22	=  L2 * sin(q1 +q2) + L3 * sin(q1 +q2 +q3);
	x23 =  L1 * cos (q1) + L2 * sin(q1 +q2) + L3 * sin(q1 +q2 +q3);
	x33 = x32 = x31 = 1.;

	Jacobian=PrMatrix3(x11, x12, x13,
	        x21, x22, x23,
	        x31, x32, x33);
    */
}

void initProj3Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
    
    traj_max_vel = 2 * M_PI/5; // initialize the traj_max_vel to be zero initially 
    traj_accl = 2 * M_PI/25 ;
	traj_t0 = gv.curTime;
	traj_tf = 20;
    traj_tb = traj_accl/traj_max_vel ; 
	r = 0.2;
	x_0 = 0.6;
	y_0 = 0.3;

    x_init = 0.8;
    y_init = 0.3;
    alpha_init = 0;
    //gv.dx = PrVector3 (x_init,y_init,alpha_init) ;
    
    double q1 = gv.q[0];
    double q2 = gv.q[1];
    double q3 = gv.q[2];
	
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{   
	gv.tau = gv.G;
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void jholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njmoveControl(GlobalVariables& gv)
{
    //Proportional controlling for positional control of the robot joints
    gv.tau = gv.kp  * (gv.qd - gv.q);
}

void jmoveControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njgotoControl(GlobalVariables& gv) 
{
    //Proportional controlling with gravity compensation
    gv.tau = gv.kp  * (gv.qd - gv.q) + gv.G;
}

void jgotoControl(GlobalVariables& gv) 
{
    gv.tau = gv.kp  * (gv.qd - gv.q)  + gv.kv * (gv.dqd - gv.dq) + gv.G;
}

void njtrackControl(GlobalVariables& gv) 
{
   /*
    * Check if t>t final, if yes we should be in the floatcontrol, else calculate
    * We need to calculate the desired values for the position and velocity according to the known parameters of the cubic spline.
    * I'll only implement this in 3DoF mode, but would work the same in 6DoF as well
    */

   if(gv.curTime > m_spline.tf){
       //floatControl(gv);
       gv.tau = gv.kp  * (p1 - gv.q)  + gv.kv * ( - gv.dq) + gv.G;
       return;
   } else{
       //The desired values for the 3 joints
       PrVector3 dq(0,0,0);
       PrVector3 ddq(0,0,0);

       //Calculate the desired positions by the polynom into dq[i] and velocities by the first derivative of polynom into ddq[i]
       double current_t = gv.curTime - m_spline.t0;
       dq[0] = m_spline.a0[0] + m_spline.a1[0] * current_t + m_spline.a2[0] * pow(current_t, 2) + m_spline.a3[0] * pow(current_t, 3);
       dq[1] = m_spline.a0[1] + m_spline.a1[1] * current_t + m_spline.a2[1] * pow(current_t, 2) + m_spline.a3[1] * pow(current_t, 3);
       dq[2] = m_spline.a0[2] + m_spline.a1[2] * current_t + m_spline.a2[2] * pow(current_t, 2) + m_spline.a3[2] * pow(current_t, 3);

       ddq[0]=m_spline.a1[0] + 2 * m_spline.a2[0] * current_t +  3 * m_spline.a3[0] * pow(current_t, 2);
       ddq[1]=m_spline.a1[1] + 2 * m_spline.a2[1] * current_t +  3 * m_spline.a3[1] * pow(current_t, 2);
       ddq[2]=m_spline.a1[2] + 2 * m_spline.a2[2] * current_t +  3 * m_spline.a3[2] * pow(current_t, 2);

       gv.tau[0] = gv.kp[0]  * (dq[0] - gv.q[0])  + gv.kv[0] * (ddq[0] - gv.dq[0]) + gv.G[0];
       gv.tau[1] = gv.kp[1]  * (dq[1] - gv.q[1])  + gv.kv[1] * (ddq[1] - gv.dq[1]) + gv.G[1];
       gv.tau[2] = gv.kp[2]  * (dq[2] - gv.q[2])  + gv.kv[2] * (ddq[2] - gv.dq[2]) + gv.G[2];
   }


}

void jtrackControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void nxtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void xtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}
void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void lineControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void proj1Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj1Control
   
  /*p1 = PrVector3(10.2 * M_PI / 180, 52.2 * M_PI / 180, -62.4 * M_PI / 180);
   initNjtrackControl(gv);
   njtrackControl(gv); */
}

void proj2Control(GlobalVariables &gv) {
 
       // PrVector3 orientation(atan2(0.8,0.3),0,0);

       // proj1Control(gv, p0, orientation);

        
        double t = gv.curTime - traj_t0;

        //Control methode for folowing the circle trajectory
        double q1 = gv.q[0];
        double q2 = gv.q[1];
        double q3 = gv.q[2];

        x_d[0] = x_0 + r * cos(traj_max_vel * t) ;
        x_d[1] = y_0 - r * sin(traj_max_vel * t) ;
        x_d[2] = 0;

        vel_d[0] = -r * sin(traj_max_vel * t)*traj_max_vel;
        vel_d[1] = -r * cos(traj_max_vel * t)*traj_max_vel ;
        vel_d[2] = 0;

       // x_curr[0] = L1 * cos(q1) + L2 * sin(q1 + q2) + L3 * sin(q1 + q2 + q3);
       // x_curr[1] = -L1 * sin(q1) + L2 * cos(q1 + q2) + L3 * cos(q1 + q2 + q3);
       // x_curr[2] = q1 + q2 + q3;

        //vel_curr = Jacobian * gv.dq;

        gv.xd = x_d;
        gv.dxd = vel_d;

        //gv.x = x_curr;

        PrVector3 F( - gv.kp * (gv.x - gv.xd) - gv.kv * (gv.dx - gv.dxd));

        //gv.qd = x_d;
        //gv.dqd = vel_d;

        // gv.tau = Jacobian.transpose() * F + gv.G;
        gv.tau = gv.Jtranspose * F  + gv.G;    
        //gv.tau[1] -= gv.tau[1];
        //gv.tau[2] = 0;

}

void proj3Control(GlobalVariables& gv) 
{
  // floatControl(gv);  // Remove this line when you implement proj3Control

    double t = gv.curTime - traj_t0;
    
    if ( t > 20 ) {
            njtrackControl(gv) ;
            return;
    } else {
    
    //double tnew = t - 15 ;
    double s[3];
    double qd1 = gv.qd[0];
    double qd2 = gv.qd[1];
    double qd3 = gv.qd[2];

        if ((t >= 0) && (t <= 5 )) {
            // acceleration for t between t0 and tb
            //traj_max_vel = traj_max_vel + traj_accl * t ;          // v = u + at
            qd2 = 0.5*traj_accl*pow(t,2) ;                    // s(t) = s(t-1) + 0.5*a*(t + (t-1))  
            s[0] = qd2 ;
        }
        else if ((t > 15 ) && (t <= 20 ) ) {
            // de-acceleration during this phase of the blend
            //traj_max_vel = traj_max_vel - traj_accl * (t-15) ;          // v = u - at
            qd2 = traj_max_vel*(t-15) - 0.5*traj_accl*pow((t-15),2) + 5*M_PI; 
        } 
        else {
            //qd2 = traj_max_vel * (t-5) - M_PI ;                               // s(t) = v(t) * t  
              qd2 = traj_max_vel*(t-5) + M_PI; 
              s[1] = qd2 ;
        }
        
        x_d[0] = x_0 + r * cos(qd2) ;
        x_d[1] = y_0 - r * sin(qd2) ;
        x_d[2] = 0;

        vel_d[0] = -r * sin(qd2);
        vel_d[1] = -r * cos(qd2);
        vel_d[2] = 0;

        gv.xd = x_d;
        gv.dxd = vel_d;

        PrVector3 F( - gv.kp * (gv.x - gv.xd) - gv.kv * (gv.dx - gv.dxd));

        gv.tau = gv.Jtranspose * F  + gv.G; 
    }
        
}


// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
   // Replace this code with any debug information you'd like to get
   // when you type "pdebug" at the prompt.
   printf( "This sample code prints the torque and mass\n" );
   gv.tau.display( "tau" );
   gv.A.display( "A" );
}

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32

/********************************************************
END OF DEFAULT STUDENT FILE 
ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 
*******************************************************/