#ifndef VARIABLE_H
#define VARIABLE_H

#include <ode/ode.h>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrowBoxD
#define dsDrawCylinder dsDrawCylinderD
#endif // dDOUBLE

dWorldID      world;        // 動力学計算用のワールド (for dynamics)
dSpaceID      space;        // 衝突検出用のスペース (for collision)
dGeomID       ground;       // 地面 (ground)
dJointGroupID contactgroup; // 接触点グループ (contact group for collision)
dsFunctions   fn;           // ドロースタッフの描画関数 (function of drawstuff)
dMass mass;

//dJointFeedback *s_feedback1 = new dJointFeedback;
//dJointFeedback *t_feedback1 = new dJointFeedback;
//dJointFeedback *s_feedback3 = new dJointFeedback;
//dJointFeedback *t_feedback3 = new dJointFeedback;
/*
dJointFeedback *c_feedback1 = new dJointFeedback;
dJointFeedback *k_feedback1 = new dJointFeedback;
dJointFeedback *a_feedback1 = new dJointFeedback;
dJointFeedback *c_feedback3 = new dJointFeedback;
dJointFeedback *k_feedback3 = new dJointFeedback;
dJointFeedback *a_feedback3 = new dJointFeedback;
*/



typedef struct {       // MyObject構造体
    dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
    dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
    dReal x0,y0,z0;
  } MyObject;
  MyObject base, roll[4], thigh[4], shin[4],foot[4], wheel[4], clog[4];

typedef struct{
    dJointID joint;
    dReal x0,y0,z0;
  }Position;
  Position comaneci[4],knee[4],ankle[4],slider[4],suspension[4],tire[4];



dReal sides[3]={0.5 ,0.5,0.2};		//ボディ部寸法[m]



dReal l_thigh = 0.08, r_thigh = 0.05;

dReal l_shin = 2*l_thigh, r_shin = r_thigh;

dReal l_foot = l_thigh, r_foot = r_thigh;

dReal l_roll = r_thigh*2, r_roll = r_thigh;

//dReal links[3] = {l_thigh,l_shin,l_foot};

dReal r_wheel = r_shin-0.004;
//dReal l_wheel = l_shin-0.06;


dReal r_clog = r_shin*3;
dReal l_clog = 0.01;



dReal v = 0.0;      //車輪の速度[rad/s]
//dReal axis_sus0 = -M_PI_4; //全輪の角度[rad]
//dReal axis_sus1 = M_PI_4;
dReal axis_sus0 = 0; //全輪の角度[rad]

dReal axis1[2][2]={{0,0},{0,0}};
dReal axis2[2][2]={{0,0},{0,0}};

dReal axis_sus[4]={0,0,0,0};	//joint_suspentionの制御量[rad]


dReal slide = r_wheel;            //0に設定すると謎の振動をする

//dReal origin[2] = {l_thigh*cos(M_PI_4) + l_shin*cos(M_PI_2),-l_thigh*sin(M_PI_4)+l_shin*sin(M_PI_2)};
dReal origin[2] = {l_thigh + l_shin*cos(M_PI_2),l_shin*sin(M_PI_2)};
//dReal moving[2][2] = {{0.00,-0.00},{0.00,-0.00}}; //{stride , swing_up};
dReal moving[2][2] = {{0.0,-0.05},{0.0,-0.05}}; //{stride , swing_up};

//dReal kstride0 = acos(cos(M_PI_4) + (100*l_thigh*(1-cos(axisc[0]-M_PI_2)) +100*moving[0])/100*l_shin);
//dReal kstride0 = 0.1547865271946589;
//dReal kstride1 = acos(100*l_thigh/100*l_shin*(1-cos(axisc[1]+M_PI_2))-100*moving[0]/100*l_shin+cos(M_PI_4));
//dReal kstride1 = 0.0339919246760807;

//dReal axisa[4] = {0.746958 ,-0.746958 ,-0.746958 ,0.746958};
dReal axisc[4];
dReal axisk[4];
dReal axisa[4] = {M_PI_4+axis1[0][0]+axis1[0][1] ,-M_PI_4-axis2[0][0]-axis2[0][1] ,-M_PI_4-axis1[1][0]-axis1[1][1] ,M_PI_4+axis2[1][0]+axis2[1][1]};

dReal add[2]={0,0};
static float view_point[3] = {0.8,-0.5,0.5}, view_direction[3] = {150,-0,0};		//視点の位置(x,y,z)[m]
//static float view_point[3] = {-1.2,0,41}, view_direction[3] = {0,0,0};		//視点の位置(x,y,z)[m],視点の方向(x,y,z)[deg]
//static float view_point[3] = {0.8,-0.5,41}, view_direction[3] = {150,0,0};

#endif

