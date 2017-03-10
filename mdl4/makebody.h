#ifndef MAKEBODY_H
#define MAKEBODY_H

#include "variable.h"


class makebody
{
private:

  unsigned int i;
  dReal hight = r_wheel + l_shin + l_thigh + sides[2] + 40;
  dReal pos[3] = {0.0,0.0,hight};       //ボディ部の重心座標

  dReal m_base=20.0;
  dReal m_thigh = 1.0;
  dReal m_shin = 2.0;
  dReal m_foot = 1.0;
  dReal m_roll = 0.8;
  dReal m_wheel = 0.5;
  dReal m_clog = 1.0;
  //dReal DENSITY_Al =2700;               //アルミニウムの密度[kg/m^3]
  //dReal DENSITY_gom =1100;              //軟質ゴムの密度[kg/m^3]
  //dReal DENSITY_hgom =1190;              //硬質ゴムの密度[kg/m^3]

  dMatrix3 posture_sus,posture_wheel;         //回転行列

  dReal axis_bound = 4*M_PI/9;    //脚部の各関節の限界角度
  dReal slide = l_shin/2;            //0に設定すると謎の振動をする

public:
  void body();
  void legs();
  void wheels();
  void clogs();
  //void sensor();

  void draw();
};

void makebody::body()
{
  base.body=dBodyCreate(world);
  base.geom = dCreateBox(space,sides[0],sides[1],sides[2]);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,m_base,sides[0],sides[1],sides[2]);
  dBodySetMass(base.body,&mass);
  //printf("%f",&mass);
  dBodySetPosition(base.body,pos[0],pos[1],pos[2]);
  dGeomSetBody(base.geom,base.body);
}

void makebody::legs()
{
  i = 0;
  //dRSetIdentity(posture_thigh);
  //dRSetIdentity(posture_sus);

  do{
    roll[i].body=dBodyCreate(world);
    roll[i].geom=dCreateCylinder(space,r_roll,l_roll);
    dMassSetZero(&mass);
    dMassSetCylinderTotal(&mass,m_roll,3,r_roll,l_roll);
    dBodySetMass(roll[i].body,&mass);
    dGeomSetBody(roll[i].geom,roll[i].body);

    thigh[i].body = dBodyCreate(world);
    thigh[i].geom = dCreateCylinder(space,r_thigh,l_thigh);
    dMassSetZero(&mass);
    dMassSetCylinderTotal(&mass,m_thigh,1,r_thigh,l_thigh);
    dBodySetMass(thigh[i].body,&mass);
    dGeomSetBody(thigh[i].geom,thigh[i].body);

    shin[i].body = dBodyCreate(world);
    //shin[i].geom = dCreateCapsule(space,r_shin,l_shin);
    shin[i].geom = dCreateCylinder(space,r_shin,l_shin);
    dMassSetZero(&mass);
    //dMassSetCapsuleTotal(&mass,m_shin,3,r_shin,l_shin);
    dMassSetCylinderTotal(&mass,m_shin,3,r_shin,l_shin);
    dBodySetMass(shin[i].body,&mass);
    dGeomSetBody(shin[i].geom,shin[i].body);

    foot[i].body = dBodyCreate(world);
    foot[i].geom = dCreateCylinder(space,r_foot,l_foot);
    dMassSetZero(&mass);
    dMassSetCylinderTotal(&mass,m_shin,3,r_foot,l_foot);
    dBodySetMass(foot[i].body,&mass);
    dGeomSetBody(foot[i].geom,foot[i].body);


    if(i==0) //right front leg
    {
      roll[i].x0=pos[0]+sides[0]/2-r_roll/2;
      roll[i].y0=roll[i].x0;
    }
    else if(i==1) //left front
    {
      roll[i].x0=pos[0]+sides[0]/2-r_roll/2;
      roll[i].y0=-roll[i].x0;
    }
    else if(i==2) //left rear
    {
      roll[i].x0=pos[0]-sides[0]/2+r_roll/2;
      roll[i].y0=roll[i].x0;
    }
    else if(i==3) //right rear
    {
      roll[i].x0=pos[0]-sides[0]/2+r_roll/2;
      roll[i].y0=-roll[i].x0;
    }
    thigh[i].x0 = roll[i].x0;
    thigh[i].y0 = roll[i].y0;
    shin[i].x0 = roll[i].x0;
    shin[i].y0 = roll[i].y0;
    foot[i].x0 = roll[i].x0;
    foot[i].y0 = roll[i].y0;
    comaneci[i].x0 =  roll[i].x0;
    comaneci[i].y0 = roll[i].y0;
    knee[i].x0 = roll[i].x0;
    knee[i].y0 = roll[i].y0;

    roll[i].z0 = pos[2] - sides[2]/2 + l_roll/2;

    thigh[i].z0 = roll[i].z0 - l_thigh/2;
    shin[i].z0 = thigh[i].z0 - l_thigh/2 - l_shin/2;
    foot[i].z0 = shin[i].z0 - l_shin/2 - l_foot/2;
    comaneci[i].z0 = roll[i].z0;
    knee[i].z0 = thigh[i].z0 - l_thigh/2;
    ankle[i].x0 = foot[i].x0;
    ankle[i].y0 = foot[i].y0;
    ankle[i].z0 = shin[i].z0 - l_shin/2;

    dBodySetPosition(roll[i].body,roll[i].x0,roll[i].y0,roll[i].z0);
    dBodySetPosition(thigh[i].body,thigh[i].x0,thigh[i].y0,thigh[i].z0);
    //dBodySetRotation(thigh[i].body,posture_thigh);
    dBodySetPosition(shin[i].body,shin[i].x0,shin[i].y0,shin[i].z0);
    //dBodySetRotation(shin[i].body,posture_shin);
    dBodySetPosition(foot[i].body,foot[i].x0,foot[i].y0,foot[i].z0);
    //dBodySetRotation(foot[i].body,posture_sus);

    suspension[i].joint = dJointCreateHinge(world,0);
    dJointAttach(suspension[i].joint,base.body,roll[i].body);
    dJointSetHingeAxis(suspension[i].joint,0,0,1);
    dJointSetHingeAnchor(suspension[i].joint,roll[i].x0,roll[i].y0,roll[i].z0);
    //dJointSetHingeParam(suspension[i].joint,dParamLoStop,-axis_bound);
    //dJointSetHingeParam(suspension[i].joint,dParamHiStop,axis_bound);

    knee[i].joint = dJointCreateHinge(world,0);
    dJointAttach(knee[i].joint,thigh[i].body,shin[i].body);
    //dJointSetHingeParam(knee[i].joint,dParamLoStop,-axis_bound);
    //dJointSetHingeParam(knee[i].joint,dParamHiStop,axis_bound);


    ankle[i].joint = dJointCreateUniversal(world,0);
    dJointAttach(ankle[i].joint,shin[i].body,foot[i].body);
    //dJointSetUniversalParam(ankle[i].joint,dParamLoStop1,-axis_bound);
    //dJointSetUniversalParam(ankle[i].joint,dParamHiStop1,axis_bound);
    dJointSetUniversalParam(ankle[i].joint,dParamLoStop2,-M_PI);
    dJointSetUniversalParam(ankle[i].joint,dParamHiStop2,M_PI);

    comaneci[i].joint = dJointCreateHinge(world,0);
    dJointAttach(comaneci[i].joint,thigh[i].body,roll[i].body);
    //dJointSetHingeParam(comaneci[i].joint,dParamLoStop,-axis_bound);
    //dJointSetHingeParam(comaneci[i].joint,dParamHiStop,axis_bound);

    if(i%2==0)
    {
      dJointSetHingeAxis(comaneci[i].joint,1,-1,0);
      dJointSetHingeAxis(knee[i].joint,1,-1,0);
      dJointSetUniversalAxis1(ankle[i].joint,1,-1,0);
    }
    else if(i%2==1)
    {
      dJointSetHingeAxis(comaneci[i].joint,1,1,0);
      dJointSetHingeAxis(knee[i].joint,1,1,0);
      dJointSetUniversalAxis1(ankle[i].joint,1,1,0);
    }
    dJointSetUniversalAxis2(ankle[i].joint,0,0,1);
    dJointSetHingeAnchor(comaneci[i].joint,comaneci[i].x0,comaneci[i].y0,comaneci[i].z0);
    dJointSetHingeAnchor(knee[i].joint,knee[i].x0,knee[i].y0,knee[i].z0);
    dJointSetUniversalAnchor(ankle[i].joint,ankle[i].x0,ankle[i].y0,ankle[i].z0);


    i++;
  }while(i<4);
}

void makebody::wheels()
{
  i = 0;
  dRSetIdentity(posture_wheel);
  dRFromAxisAndAngle(posture_wheel,1,0,0,M_PI_2);

  do{
    wheel[i].body = dBodyCreate(world);
    wheel[i].geom = dCreateSphere(space,r_wheel);
    dMassSetZero(&mass);
    //dMassSetSphere(&mass,DENSITY_gom,r_wheel);
    dMassSetSphereTotal(&mass,m_wheel,r_wheel);
    //printf("wheel = %f kg\n",&mass);
    dBodySetMass(wheel[i].body,&mass);
    dGeomSetBody(wheel[i].geom,wheel[i].body);
    wheel[i].x0 = foot[i].x0;
    wheel[i].y0 = foot[i].y0;
    wheel[i].z0 = foot[i].z0 - l_shin/2 + r_wheel;
    dBodySetPosition(wheel[i].body,wheel[i].x0,wheel[i].y0,wheel[i].z0);
    dBodySetRotation(wheel[i].body,posture_wheel);

    tire[i].joint = dJointCreateHinge(world,0);
    dJointAttach(tire[i].joint,wheel[i].body,foot[i].body);
    dJointSetHingeAxis(tire[i].joint,0,1,0);
    dJointSetHingeAnchor(tire[i].joint,wheel[i].x0, wheel[i].y0, wheel[i].z0);
    i++;
  }while(i<4);

}

void makebody::clogs()
{
  i = 0;

  do{
    clog[i].body = dBodyCreate(world);
    clog[i].geom = dCreateCylinder(space,r_clog,l_clog);
    dMassSetZero(&mass);
    //dMassSetCylinder(&mass,DENSITY_hgom,2,r_clog,l_clog);
    dMassSetCylinderTotal(&mass,m_clog,2,r_clog,l_clog);
    dBodySetMass(clog[i].body,&mass);
    //printf("clog = %f kg\n",&mass);
    dGeomSetBody(clog[i].geom,clog[i].body);
    clog[i].x0 = foot[i].x0;
    clog[i].y0 = foot[i].y0;
    clog[i].z0 = wheel[i].z0 - r_wheel + l_clog/2;
    dBodySetPosition(clog[i].body,clog[i].x0,clog[i].y0,clog[i].z0);

    slider[i].joint=dJointCreateSlider(world,0);
    dJointAttach(slider[i].joint,clog[i].body,foot[i].body);
    dJointSetSliderAxis(slider[i].joint,0,0,1);
    dJointSetSliderParam(slider[i].joint,dParamLoStop,r_clog/2);
    dJointSetSliderParam(slider[i].joint,dParamHiStop,l_foot/4);
    i++;
  }while(i<4);
}

void makebody::draw()
{
  dsSetColor(0.0,1.0,1.0);
  dsDrawBoxD(dBodyGetPosition(base.body),dBodyGetRotation(base.body),sides);

  i = 0;
  do{
    dsSetColor(1.0,0.0,0.0);
    dsDrawCylinderD(dBodyGetPosition(thigh[i].body),dBodyGetRotation(thigh[i].body),l_thigh,r_thigh);
    dsDrawCylinderD(dBodyGetPosition(shin[i].body),dBodyGetRotation(shin[i].body),l_shin,r_shin);
    dsDrawCylinderD(dBodyGetPosition(foot[i].body),dBodyGetRotation(foot[i].body),l_foot,r_foot);
    dsSetColor(1.0,1.0,1.0);
    dsDrawCylinderD(dBodyGetPosition(roll[i].body),dBodyGetRotation(roll[i].body),l_roll,r_roll);
    dsDrawSphereD(dBodyGetPosition(wheel[i].body),dBodyGetRotation(wheel[i].body),r_wheel);
    //dsDrawSphereD(dBodyGetPosition(s_roll[i].body),dBodyGetRotation(s_roll[i].body),r_thigh);
    //dsDrawSphereD(dBodyGetPosition(s_comaneci[i].body),dBodyGetRotation(s_comaneci[i].body),r_thigh);
    //dsDrawSphereD(dBodyGetPosition(s_knee[i].body),dBodyGetRotation(s_knee[i].body),r_thigh);
    //dsDrawSphereD(dBodyGetPosition(s_ankle[i].body),dBodyGetRotation(s_ankle[i].body),r_thigh);
    //dsDrawSphereD(dBodyGetPosition(s_tire[i].body),dBodyGetRotation(s_tire[i].body),r_thigh);
    dsSetColor(0.6,0.4,0.1);
    dsDrawCylinderD(dBodyGetPosition(clog[i].body),dBodyGetRotation(clog[i].body),l_clog,r_clog);
    i++;
  }while(i<4);
}



#endif
