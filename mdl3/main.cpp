//#include <ode/ode.h>                // ODE用ヘッダーファイル
#include <drawstuff/drawstuff.h>    // 描画用ヘッダーファイル
#include <stdio.h>

//#include "lib/new/setdraw.cpp"          // 描画関数
//#include "create.h"
//#include "control.h"
#include "control.h"
#include "makebody.h"

//#include "lib/new/makebody.cpp"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#define BOUNCE (0.0)			//反発係数
#define BOUNCE_VEL (0.0)		//跳ね返りに必要な最低速度[m/s]
#define a (5.0)

static void running()
{
  unsigned int i = 0;
  pcontrol p;
  //inverse inv;
  //inv.pos(l_thigh,l_shin,posx[1],posy[1],moving,axis[1]);
  //inv.pos(l_thigh,l_shin,posx[0],posy[0],moving,axis[0]);

  do{
    p.slider(slider[i].joint,slide);
    p.hingesus(suspension[i].joint,axis_sus[i]);
    p.hingelow(comaneci[i].joint,axisc[i]);
    p.hingehi(knee[i].joint,axisk[i]);
    p.universal(ankle[i].joint,axisa[i],axis_sus0);

    p.run(tire[i].joint,v);

    i++;
  }while(i<4);
}

dReal tmp1;

static void measurement()
{
  FILE *fp;
  fp = fopen("exam.txt","a");

  //ofstream outputfile("exam.txt");
  //tmp1 = dJointGetHingeAngleRate(tire[0].joint);
  //printf("Tire angleRate = %f\n",tmp1);
  //fprintf(fp,"Tire angleRate = %f\n",tmp1);

  tmp1 = dJointGetHingeAngleRate(comaneci[0].joint);
  printf("Comaneci1 angleRate = %f\n",tmp1);
  //fprintf(fp,"Comaneci1 angleRate = %f\n",tmp1);

  tmp1 = dJointGetHingeAngleRate(comaneci[2].joint);
  printf("Comaneci3 angleRate = %f\n",tmp1);
  //fprintf(fp,"Comaneci3 angleRate = %f\n",tmp1);

  tmp1 = dJointGetHingeAngleRate(knee[0].joint);
  printf("Knee1 angleRate = %f\n",tmp1);
  //fprintf(fp,"Knee1 angleRate = %f\n",tmp1);

  tmp1 = dJointGetHingeAngleRate(knee[2].joint);
  printf("Knee3 angleRate = %f\n",tmp1);
  //fprintf(fp,"Knee3 angleRate = %f\n",tmp1);


  tmp1 = dJointGetUniversalAngle1Rate(ankle[0].joint);
  printf("Ankle1 angleRate = %f\n",tmp1);
  //fprintf(fp,"Ankle1 angleRate = %f\n",tmp1);

  tmp1 = dJointGetUniversalAngle1Rate(ankle[2].joint);
  printf("Ankle3 angleRate = %f\n",tmp1);
  //fprintf(fp,"Ankle3 angleRate = %f\n",tmp1);


  tmp1 = dJointGetHingeAngleRate(suspension[0].joint);
  printf("Suspension1 angleRate = %f\n",tmp1);
  //fprintf(fp,"Suspension1 angleRate = %f\n",tmp1);

  tmp1 = dJointGetHingeAngleRate(suspension[2].joint);
  printf("Suspension3 angleRate = %f\n",tmp1);
  //fprintf(fp,"Suspension3 angleRate = %f\n",tmp1);


  ///unit of output value is [rad/s]
}


static void nearCallback(void *data,dGeomID o1,dGeomID o2)      /***  衝突判定　***/
{
    static const int N = 10;
    dContact contact[N];

    int isGroup = ((ground==o1)||(ground==o2));

    int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
    if(isGroup){
        for(int i=0; i<n; i++){
            contact[i].surface.mu=dInfinity;
            contact[i].surface.mode=dContactBounce;
            contact[i].surface.bounce=BOUNCE;
            contact[i].surface.bounce_vel=BOUNCE_VEL;

            dJointID c=dJointCreateContact(world,contactgroup,&contact[i]);
            dJointAttach(c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
        }
    }
}

static void simLoop(int pause)           /***  シミュレーションループ　***/
{
  if (!pause) {
    dSpaceCollide(space,0,&nearCallback); // 衝突検出 (collision detection)
    dWorldStep(world, 0.01);              // ステップ更新 (step a simulation)
    dJointGroupEmpty(contactgroup);       // 接触点グループを空 (empty jointgroup)
  }
//  drawBody();

  makebody make;
  make.draw();

  running();
  //measurement();
}

static void command(int cmd)
{
 // inverse inv;

  switch(cmd)
  {
    case 'b':
      slide = r_wheel;
      v = -5.0;
      //printf("drive bot\n");
      tmp1 = dJointGetHingeAngleRate(tire[1].joint);
      printf("%f\n",tmp1);
      break;

    case 'n':
      axis_sus0 += M_PI/48;
      if(axis_sus0>M_PI_2){axis_sus0=M_PI_2;}

      printf("turn right bot\n");
      break;

    case 'v':
      axis_sus0 -= M_PI/48;
      if(axis_sus0<-M_PI_2){axis_sus0=-M_PI_2;}

      printf("turn left bot\n");
      break;

    case 'y':
      slide = 0.0;
      v = 0.0;
      printf("down all clogs\n");
      break;


/***control right front and left rear***/
    case 'q':
/*
      moving[1] = 0.005;
      inv.pos(l_thigh,l_shimakebody make;
  make.draw();n,posx[1],posy[1],moving,axis[1]);
      axisc[1] = M_PI_2+axis[1][0];
      axisc[3] = M_PI_2+axis[1][0];
      axisk[1] = M_PI_4+axis[1][1];
      axisk[3] = M_PI_4+axis[1][1];
      axisa[1] = M_PI_4+axis[1][0]-axis[1][1];
      axisa[3] = M_PI_4+axis[1][0]-axis[1][1];
*/
      axisc[1] = -M_PI_2 - 0.4197277441762919;
      axisc[3] = M_PI_2 + 0.4197277441762919;
      axisk[1] = -M_PI_4 - 0.06343048919665528;
      axisk[3] = M_PI_4 + 0.06343048919665528;
      axisa[1] = -M_PI_4-0.4197277441762919+0.06343048919665528;
      axisa[3] = M_PI_4+0.4197277441762919-0.06343048919665528;

      //printf("theta1 = %f\n",axis[1][0]);
      //printf("theta2 = %f\n",axis[1][1]);
      //feedback = dJointGetFeedback()
      //FILE *fp;
      //fp = fopen("exam.txt","a");
      //fprintf(fp,"follow right leg\n");
      tmp1 = dJointGetHingeAngleRate(comaneci[1].joint);
      printf("%f\n",tmp1);

      tmp1 = dJointGetHingeAngleRate(knee[1].joint);
      printf("%f\n",tmp1);

      tmp1 = dJointGetUniversalAngle1Rate(ankle[1].joint);
      printf("%f\n",tmp1);
      break;

    case 'w':

      axis_sus[1] = -M_PI_4;
      axis_sus[3] = M_PI_4+M_PI_2;

      //fprintf(fp,"walking mode comaneci\n");

      tmp1 = dJointGetHingeAngleRate(suspension[1].joint);
      printf("%f\n",tmp1);
      tmp1 = dJointGetHingeAngleRate(suspension[3].joint);
      printf("%f\n",tmp1);
      break;

    case 'a':

      axis_sus[1] += M_PI_4;
      axis_sus[3] += M_PI_4;

      //fprintf(fp,"rolling right comaneci\n");
      break;

    case 's':
      axisc[1] = -M_PI_2 + 0.40419545484478053;
      axisc[3] = M_PI_2 - 0.40419545484478053;
      axisk[1] = -M_PI_4 + 0.13826278781360657*a;
      axisk[3] = M_PI_4 - 0.13826278781360657*a;
      axisa[1] = -M_PI_4+0.40419545484478053-0.13826278781360657*a;
      axisa[3] = M_PI_4-0.40419545484478053+0.13826278781360657*a;

      //printf("expand legs\n");
      tmp1 = dJointGetHingeAngleRate(comaneci[1].joint);
      printf("%f",tmp1);
      tmp1 = dJointGetHingeAngleRate(comaneci[3].joint);
      printf("%f\n",tmp1);

      tmp1 = dJointGetHingeAngleRate(knee[1].joint);
      printf("%f",tmp1);
      tmp1 = dJointGetHingeAngleRate(knee[3].joint);
      printf("%f\n",tmp1);

      tmp1 = dJointGetUniversalAngle1Rate(ankle[1].joint);
      printf("%f",tmp1);
      tmp1 = dJointGetUniversalAngle1Rate(ankle[3].joint);
      printf("%f\n",tmp1);
      break;

    case 'e':

      //moving[1] = 0;
      //inv.pos(l_thigh,l_shin,posx[1],posy[1],moving,axis[1]);
      axisc[1] = -M_PI_2;
      axisc[3] = M_PI_2;
      axisk[1] = -M_PI_4;
      axisk[3] = M_PI_4;
      axisa[1] = -M_PI_4;
      axisa[3] = M_PI_4;
      //printf("theta1 = %f\n",axis[1][0]);
      //printf("theta2 = %f\n",axis[1][1]);
      //printf("down right comaneci\n");
      tmp1 = dJointGetHingeAngleRate(comaneci[1].joint);
      printf("%f",tmp1);
      //tmp1 = dJointGetHingeAngleRate(comaneci[3].joint);
      //printf("%f\n",tmp1);

      tmp1 = dJointGetHingeAngleRate(knee[1].joint);
      printf("%f",tmp1);
      //tmp1 = dJointGetHingeAngleRate(knee[3].joint);
      //printf("%f\n",tmp1);

      tmp1 = dJointGetUniversalAngle1Rate(ankle[1].joint);
      printf("%f",tmp1);
      //tmp1 = dJointGetUniversalAngle1Rate(ankle[3].joint);
      //printf("%f\n",tmp1);
      break;

      break;



/***control left front and right rear***/
    case 'p':

      axisc[0] = M_PI_2 + 0.4197277441762919;
      axisc[2] = -M_PI_2 - 0.4197277441762919;
      axisk[0] = M_PI_4 + 0.06343048919665528;
      axisk[2] = -M_PI_4 - 0.06343048919665528;
      axisa[0] = M_PI_4+0.4197277441762919-0.06343048919665528;
      axisa[2] = -M_PI_4-0.4197277441762919+0.06343048919665528;
      printf("follow left comaneci\n");
      break;

    case 'o':
      axis_sus[0] = M_PI_4;
      axis_sus[2] = -M_PI_4-M_PI_2;
      printf("walking mode\n");
      break;

    case 'l':
      axis_sus[0] -= M_PI_4;
      axis_sus[2] -= M_PI_4;
      printf("rolling left comaneci\n");
      break;

    case 'i':
      axisc[0] = M_PI_2;
      axisc[2] = -M_PI_2;
      axisk[0] = M_PI_4;
      axisk[2] = -M_PI_4;
      axisa[0] = M_PI_4;
      axisa[2] = -M_PI_4;
      printf("down left comaneci\n");
      break;

    case 'k':
      axisc[0] = M_PI_2 - 0.40419545484478053;
      axisc[2] = -M_PI_2 + 0.40419545484478053;
      axisk[0] = M_PI_4 - 0.13826278781360657*a;
      axisk[2] = -M_PI_4 + 0.13826278781360657*a;
      axisa[0] = M_PI_4-0.40419545484478053+0.13826278781360657*a;
      axisa[2] = -M_PI_4+0.40419545484478053-0.13826278781360657*a;
      printf("down comaneci\n");
      break;

      ///show status etc


    case 'f':

      //measurement_torque(mode);
      printf("stop bot\n");
      break;

    case 'h':
      printf("\n\n////How to work the bot////\n");
      printf("when you press the following key, the bot being work\n\n");
      printf("if you want to change \"walking mode\"\, \n press key in order from \"y\,q\,w\,e\" and \"p\,o\,i\"\n");
      printf("  key \"w\" :\n");
      break;
  }
}

static void start()                                    /*** 前処理　***/
{
    static float xyz[3] = {view_point[0],view_point[1],view_point[2]};         // 視点の位置
    static float hpr[3] = {view_direction[0],view_direction[1],view_direction[2]};          // 視線の方向
    dsSetViewpoint(xyz,hpr);                     // カメラの設定
}

static void setDrawStuff()           /*** 描画関数の設定 ***/
{
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.command = &command;
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.path_to_textures = "../../drawstuff/textures"; // テクスチャ
}


/*** main関数 ***/
int main(int argc, char *argv[])
{
  //FILE *fp;
  //fp = fopen("exam.txt","a");
  //fprintf(fp,"exam start\n");
  setDrawStuff();                      // 描画関数の設定
  //printf("If you want to know how to work the bot, press \"h\" key. \n\n");


  dInitODE();
  world = dWorldCreate();                  // 世界の創造
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground = dCreatePlane(space,0,0,1,0);
  dWorldSetGravity(world,0,0,-9.80);        // 重力設定

  makebody make;
  make.body();
  make.legs();
  make.wheels();
  make.clogs();
 // make.sensor();

  dsSimulationLoop(argc,argv,640, 480,&fn); // シミュレーションループ
  dSpaceDestroy(space);
  dWorldDestroy(world);                    // 世界の終焉
  dCloseODE();
  return 0;
}



