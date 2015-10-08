#include <drawstuff/drawstuff.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include "MathUtils.h"
#include "ControlPBP.h"
#include "UnityOde.h"
#include <ode/ode.h>
#include <stdio.h>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <unit/from_robot.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>


//dReal body,geom, body1, geom1;
dReal body_capsule;
dsFunctions fn;
float set_angle = 0.0f;

float conv = 0.0;
bool  check;
float pos_robot;
float ang_robot;
int   body, hinge,body1;
dReal geom,geom1;
float last_angle ;
float change;


class objects
{
public:
 int body;
 int geom;
 int joint[2];
};


objects ball, stage, capsule, capsule1, capsule2, capsule3, over_ball,obs ;


void robot(int com)
{
		
	//dQuaternion q_t;
	//dQFromAxisAndAngle (q_t,1,0,0,set_angle) ;   //dQFromAxisAndAngle (dQuaternion q, dReal ax, dReal ay, dReal az, dReal angle);
	//odeGeomSetQuaternion(capsule2.geom,q_t);
	
	
	

	dReal MaxForce = dInfinity;
	//odeJointSetHingeParam(capsule2.joint[0],dParamFMax,dInfinity);
	//odeJointSetHingeParam(capsule2.joint[0],dParamVel, 0.9);
	
	//odeJointSetHingeParam(capsule3.joint[1],dParamFMax,dInfinity);
	//odeJointSetHingeParam(capsule3.joint[1],dParamVel, 0.5 );

	stepOde(0);
 	//set_angle = set_angle + 0.1f ;
	float angle;
	//angle = odeJointGetHingeAngle(capsule2.joint[0]);
	//printf("joint angle %f \n" , angle);

}



void start()
{
	static float xyz[3] = {0.0,-4.0,4.0};
	static float hpr[3] = {110.0,-10.0,0.0};
	dsSetViewpoint (xyz,hpr);
}


void  prepDrawStuff() 
{
	fn.version = DS_VERSION;
	fn.start   = &start;
  	fn.step    = &robot;
  	fn.command = NULL;
  	fn.stop    = NULL;
  	fn.path_to_textures = "/home/rokon/ode-0.12/drawstuff/textures";

  
}




int main(int argc, char **argv)
{
	prepDrawStuff(); 
initOde(30);
	setCurrentOdeContext(ALLTHREADS);
	odeRandSetSeed(0);
	odeSetContactSoftCFM(0);
	odeWorldSetGravity(0, 0, -9.81f);


	body = odeBodyCreate();
	geom = odeCreateBox(0.4f, 0.4f, 0.05f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetBoxTotal(body, 0.94, 0.4f,0.4f, 0.05f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(body,0,0.0,2.55f);
	odeGeomSetBody(geom,body);
	printf("capsule last link body id %f, geom id %f \n", body, geom);




	hinge =odeJointCreateFixed();
	odeJointAttach(hinge,body,0);
	odeJointSetFixed(hinge);

	/*
	hinge=odeJointCreateHinge();
	//printf("joint ID : %d",hinge);

	odeJointAttach(hinge,body,0);
	odeJointSetHingeAnchor(hinge,0,0,2.55);
	odeJointSetHingeAxis(hinge,0,1,0);
	//odeJointSetHingeParam(hinge,dParamFMax,0); //this hinge has no motor, we'll control it directly using torques
	//odeJointSetHingeParam(hinge,dParamVel,1.0f);
	*/



	body1 = odeBodyCreate();
	geom1 = odeCreateSphere( 0.05f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetSphereTotal(body1, 0.04,0.05f); //odeMassSetSphereTotal(int bodyId, float total_mass, float radius)
	odeBodySetPosition(body1,0.0,0.0,3.2f);
	odeGeomSetBody(geom1,body1);
	printf("capsule last link body id %f, geom id %f \n", ball.body, ball.geom);



/*
	///////CREATING BALL//////////

	 geom1 = odeCreateSphere(.03f);
	//Create a body and attach it to the geom
	body1=odeBodyCreate();


	//odeMassSetSphereTotal(body1,.05f,1.0f);
	 //odeMassSetSphere()


    
    odeMassSetSphereTotal(body1,0.04,0.03);
    odeBodySetPosition(body1,0,0,3.554f);
	//odeBodySetMass (body1, 0.04);
	odeGeomSetBody(geom1,body1);

	//Set position of the ball
*/	

	setCurrentOdeContext(0);
	saveOdeState(0);


	

	printf("body %d, body 1 %d", body, body1);


	dsSimulationLoop (argc,argv,900,600,&fn);
	





}
