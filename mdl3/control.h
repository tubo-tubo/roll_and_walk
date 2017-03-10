#ifndef CONTROL_H
#define CONTROL_H

#include "variable.h"

/*** 逆運動学用の関数 ***/
/*
class inverse
{
public:
   void value(double link1,double axis[2], double vol, double rv[2]);       //速度の逆運動学
   void pos3(double link[3], double pos[3], double theta[3]);
   void pos(double l1, double l2, double px, double pz, double stride[2], double theta[3]);
};

void inverse::value(double link1,double axis[3], double vol, double rv[3])
{
  double a1qr = axis[1]-(M_PI_4);
  double a2qr = axis[2]-(M_PI_4);
  double a1qh = axis[1]+(M_PI_4);
  double a2qh = axis[2]+(M_PI_4);
  double a12 = axis[1]+axis[2];

  rv[0] = vol / link1*sin(axis[0])*(sqrt(2)*sin(a1qh)+cos(a12));
  rv[1] = vol*((3*cos(a12)+2*sin(a12))*(2*sin(2*a12)-cos(2*axis[1]))+sqrt(2)*sin(2*a12)*(3*cos(a1qr)+2*sin(a1qr))+
            2*sqrt(2)*(3*pow(cos(a12),2)*sin(a1qr)+2*pow(sin(a12),2)*cos(a1qr))) / 3*sqrt(2)*link1*sin(a2qh)*(2*sin(2*a12)-cos(2*axis[1])+2*sqrt(2)*sin(2*axis[1]+a2qr));
  rv[2] = -vol*(sqrt(2)*(3*cos(a1qr)-2*sin(a1qr))+2*(3*cos(a12)-2*sin(a12))) / 6*sqrt(2)*link1*sin(a2qh);

}

void inverse::pos3(double link[3], double pos[3], double theta[3])
{

  double block_a = pos[1] - link[2]*sin(pos[2]);
  double block_b = pos[0] - link[2]*cos(pos[2]);
  double block_ab = block_a*block_a + block_b*block_b;
  double link01 = link[0]*link[0] + link[1]*link[1];

  theta[1] = M_PI - acos((-block_ab+link01)/2*link[0]*link[1]);
  theta[0] = atan2(block_a,block_b) - atan2(link[1]*sin(theta[1]),link[0]+link[1]*cos(theta[1]));

  theta[2] = pos[2] - theta[1] - theta[2];
}


void inverse::pos(double l1, double l2, double px, double pz, double stride[2], double theta[2])
{
  double link1 = l1;
  double link2 = l2;
  double posx = (px+stride[0]);
  double posz = (pz+stride[1]);

  double sq_l1 = link1*link1;       //pow()を使うより速いらしい
  double sq_l2 = link2*link2;
  double sq_xz = posx*posx + posz*posz;

  theta[0] = -atan(posz/posx) + acos((sq_l1-sq_l2+sq_xz)/(2*link1*sqrt(sq_xz)));
  theta[1] = M_PI_4 - acos(cos(M_PI_4) + (l1*(1-cos(theta[0])) + stride[0])/l2);
  //theta[1] = acos((sq_xz-sq_l1-sq_l2)/2*l1*l2);

  //printf("%f\n",t);
  //theta[1] = acos((sq_xz-sq_l1-sq_l2)/2*link1*link2);
  //theta[0] = atan2(posz,posx) - asin((link1+link2*cos(theta[1]))/sqrt(sq_xz));

  //theta[1] = M_PI - acos((link1*link1 + link2*link2 - posx*posx - posz*posz)/2*link1*link2);
  //theta[0] = atan2(posx,posz) - atan2(link2*sin(theta[1]),(link1 + link2*cos(theta[1])));

  //double Ca = (link1*link1 + sq_xz - link2*link2)/2*link1*sqrt(sq_xz);
  //double Cb = (link1*link1 + link2*link2 - sq_xz)/2*link1*link2;
  //theta[0] = M_PI_2 - atan2(posz,posx) - atan2(sqrt(1-Ca*Ca),Ca);
  //theta[0] = M_PI_2 - atan2(posz,posx) + atan2(sqrt(1 - Ca*Ca),Ca);
  //theta[1] = M_PI - atan2(sqrt(1-Cb*Cb),Cb);
  //theta[1] = M_PI + atan2(sqrt(1-Cb*Cb),Cb);

  //theta[0] += 0.577232;
  //theta[1] -= 1.570779;
  //theta[2] = M_PI_4 -theta[0] - theta[1];

  //printf("theta1 = %f\n",axis[0][0]);
  //printf("theta2 = %f\n",axis[0][1]);
  //printf("theta3 = %f\n",axis[0][2]);
}
*/

/*** P制御用の関数 ***/
class pcontrol
{
private:
  double kp = 1.0;			//比例定数
  double kp1 = 1.0*kp;
  double kp2 = 1.0*kp;
  double fmax = dInfinity;		//最大トルク [Nm]
  //double fmax1 = 100;		//最大トルク [Nm]
  double tmp1;        //ヒンジの現在角
  double tmp2;
  double u1;          // 残差
  double u2;
  double limit_vol[2];
  double round_vol = 800;
/*
  void countInv()
  {
    inverse inv;
    inv.value(l_thigh, axis, round_vol, limit_vol);
  }
*/
public:
  void hingehi(dJointID joint,double target);
  void hingesus(dJointID joint,double target);
  void hingelow(dJointID joint,double target);
  void universal(dJointID joint,double target1,double target2);
  void slider(dJointID joint,double target);
  void run(dJointID joint,double target);
  void show(dJointID joint);
  //void run(dJointID joint,double target1 ,double target2);
};

void pcontrol::hingehi(dJointID joint,double target)
{
  //kp = 1.5;			//比例定数
  //fmax = 200000;		//最大トルク [Nm]
  //countInv();

  tmp1 = dJointGetHingeAngle(joint);
  u1 = kp1*(target - tmp1);

  //if(u1>limit_vol[0]){u1 = limit_vol[0];}

  dJointSetHingeParam(joint,dParamVel,u1);
  dJointSetHingeParam(joint,dParamFMax,fmax);
}

void pcontrol::hingesus(dJointID joint,double target)
{
  //kp = 1.0;			//比例定数
  //fmax = 200000;		//最大トルク [Nm]
  //countInv();

  tmp1 = dJointGetHingeAngle(joint);
  u1 = kp*(target - tmp1);

  //if(u1>limit_vol[2]){u1 = limit_vol[2];}

  dJointSetHingeParam(joint,dParamVel,u1);
  dJointSetHingeParam(joint,dParamFMax,fmax);
}

void pcontrol::hingelow(dJointID joint,double target)
{
  //kp = 1.0;			//比例定数
  //fmax = 200000;		//最大トルク [Nm]
  //countInv();

  tmp1 = dJointGetHingeAngle(joint);
  u1 = kp*(target - tmp1);

  //if(u1>limit_vol[1]){u1 = limit_vol[1];}

  dJointSetHingeParam(joint,dParamVel,u1);
  dJointSetHingeParam(joint,dParamFMax,fmax);
}

void pcontrol::universal(dJointID joint,double target1,double target2)
{
  //kp = 1.0;			//比例定数
  //fmax = 200000;		//最大トルク [Nm]

  tmp1 = dJointGetUniversalAngle1(joint);     //軸1の処理
  u1 = kp*(target1-tmp1);
  //if(u1>limit_vol[1]){u1 = limit_vol[1];}

  tmp2 = dJointGetUniversalAngle2(joint);     //軸2の処理
  u2 = kp*(target2-tmp2);

  dJointSetUniversalParam(joint,dParamVel1,u1);
  dJointSetUniversalParam(joint,dParamFMax1,fmax);
  dJointSetUniversalParam(joint,dParamVel2,u2);
  dJointSetUniversalParam(joint,dParamFMax2,fmax);
}

void pcontrol::slider(dJointID joint,double target)
{
  //kp   = 2.0;
  //fmax = 100;

  tmp1  = dJointGetSliderPosition(joint);  // スライダの現在位置
  u1    = 3*kp*(target - tmp1);

  dJointSetSliderParam(joint, dParamVel,  u1);
  dJointSetSliderParam(joint, dParamFMax, fmax);

}

void pcontrol::run(dJointID joint,dReal target)
{
  //fmax = 10000;		//最大トルク 100Nm

  dJointSetHingeParam(joint,dParamVel,target);
  dJointSetHingeParam(joint,dParamFMax,fmax);
}

void pcontrol::show(dJointID joint)
{

}
/*
void pcontrol::run(dJointID joint,double target1, double target2)
{
  //fmax = 10000;		//最大トルク 100Nm

  dJointSetHinge2Param(joint,dParamVel2,target1);
  dJointSetHinge2Param(joint,dParamFMax2,fmax);

  tmp1 = dJointGetHinge2Angle1(joint);
  u1 = 0.5*kp*(target2 - tmp1);
  dJointSetHinge2Param(joint,dParamVel1,u1);
  dJointSetHinge2Param(joint,dParamFMax1,fmax);
}
*/

#endif
