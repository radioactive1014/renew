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


class objects
{
public:
 dReal body;
 dReal geom;
 dReal joint[2];
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
	initOde(2);
	setCurrentOdeContext(ALLTHREADS);
	odeRandSetSeed(0);
	odeSetContactSoftCFM(0);
	odeWorldSetGravity(0, 0, -9.81f);


	/*
	//creating the stage
   	body=odeBodyCreate();
	geom = odeCreateBox(.4,.4,0.05);
	odeMassSetBoxTotal(body, 0.94f, 0.4, 0.4 , 0.05) ;
    	odeBodySetPosition(body,0,0,2.5);
	odeGeomSetBody(geom,body);
	printf("context 0 : body id %f, geom id %f \n", body,geom);
	*/

	
	/*
	//creating a ball
	body1=odeBodyCreate();
 	geom1 = odeCreateSphere(.03f);
	odeMassSetSphereTotal(body1,0.04,0.03);
    	odeBodySetPosition(body1,0,0,2.554f);
	odeGeomSetBody(geom1,body1);
	printf("context 1 : body id %f, geom id %f \n", body1,geom1);
	*/


	//creating a capsule
	capsule.body = odeBodyCreate();
	capsule.geom = odeCreateCapsule(0, 0.1f, 0.32f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetCapsuleTotal(capsule.body, 1, 0.1, 0.32f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(capsule.body,0,0,0.25f);
	odeGeomSetBody(capsule.geom,capsule.body);
	printf("capsule base id %f, geom id %f \n", capsule.body, capsule.geom);






	//fixed joint
	capsule.joint[0] =odeJointCreateFixed();
	odeJointAttach(capsule.joint[0],capsule.body,0);
	odeJointSetFixed(capsule.joint[0]);
	
	


	//creating the first link
	capsule1.body = odeBodyCreate();
	capsule1.geom = odeCreateCapsule(0, 0.1f, 0.60f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetCapsuleTotal(capsule1.body, 1, 0.1, 0.60f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(capsule1.body,-0.30,0,0.60f);
	odeGeomSetBody(capsule1.geom,capsule1.body);
	printf("capsule first link body id %f, geom id %f \n", capsule1.body, capsule1.geom);


	
		
	dQuaternion q;
	dQFromAxisAndAngle (q,0,1,0,1.57) ;   //dQFromAxisAndAngle (dQuaternion q, dReal ax, dReal ay, dReal az, dReal angle);
	odeGeomSetQuaternion(capsule1.geom,q);

	//fixed joint first link
	capsule1.joint[1] =odeJointCreateFixed();
	odeJointAttach(capsule1.joint[1],capsule1.body,0);
	odeJointSetFixed(capsule1.joint[1]);


	
	//creating the second link 
	capsule2.body = odeBodyCreate();
	capsule2.geom = odeCreateCapsule(0, 0.1f, 0.20f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetCapsuleTotal(capsule2.body, 1, 0.1, 0.20f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(capsule2.body,-0.9,0,0.60f);
	odeGeomSetBody(capsule2.geom,capsule2.body);
	printf("capsule second link body id %f, geom id %f \n", capsule2.body, capsule2.geom);




	dQuaternion q1;
	dQFromAxisAndAngle (q1,0,1,0,1.57) ;   //dQFromAxisAndAngle (dQuaternion q, dReal ax, dReal ay, dReal az, dReal angle);
	odeGeomSetQuaternion(capsule2.geom,q);

	

	
	
	// hinge joint
	capsule2.joint[0] =odeJointCreateHinge();
	odeJointAttach(capsule2.joint[0],capsule2.body,capsule1.body);
	odeJointSetHingeAnchor(capsule2.joint[0],-0.5,0,0.60);
	odeJointSetHingeAxis(capsule2.joint[0],1,0,0);

	//odeJointSetHingeParam(capsule2.joint[0], dParamLoStop , 0);
	//odeJointSetHingeParam(capsule2.joint[0], dParamHiStop , 1.57);
	

	/*
	//creating last link(upright)
	capsule3.body = odeBodyCreate();
	capsule3.geom = odeCreateCapsule(0, 0.1f, 0.2f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetCapsuleTotal(capsule3.body, 1, 0.1, 0.20f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(capsule3.body,-1.1,0.5,0.65f);
	odeGeomSetBody(capsule3.geom,capsule3.body);
	printf("capsule last link body id %f, geom id %f \n", capsule3.body, capsule3.geom);

       
	
	/*
	
	//fixed joint
	capsule3.joint[0] =odeJointCreateFixed();
	odeJointAttach(capsule3.joint[0],capsule3.body,capsule2.body);
	odeJointSetFixed(capsule3.joint[0]);
	
	
	
	
	// hinge joint
	capsule3.joint[1] =odeJointCreateHinge();
	odeJointAttach(capsule3.joint[1],capsule3.body,capsule2.body);
	odeJointSetHingeAnchor(capsule3.joint[1],-1.1,0.5,0.60);
	odeJointSetHingeAxis(capsule3.joint[1],0,1,0);

	///odeJointSetHingeParam(capsule3.joint[1], dParamLoStop, 0);
	//odeJointSetHingeParam(capsule3.joint[1], dParamHiStop , 1.5);



		
	/*
	capsule3.joint[0] =  odeJointCreateHinge2();
	odeJointAttach(capsule3.joint[0],capsule3.body,capsule2.body);
	odeJointSetHinge2Anchor(capsule3.joint[0], -1.1, 0.0, 0.60);
	odeJointSetHinge2Param(capsule3.joint[0], dParamLoStop1 , 0);
	odeJointSetHinge2Param(capsule3.joint[0], dParamLoStop2 , 0);

	odeJointSetHinge2Param(capsule3.joint[0], dParamHiStop1 , 1.57);
	odeJointSetHinge2Param(capsule3.joint[0], dParamHiStop2 , 1.57);
	*/

	 //creating last link (balll)

	capsule3.body = odeBodyCreate();
	capsule3.geom = odeCreateSphere( 0.2f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetSphereTotal(capsule3.body, 1,0.20f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(capsule3.body,-1.3,0.0,0.60f);
	odeGeomSetBody(capsule3.geom,capsule3.body);
	printf("capsule last link ball body id %f, geom id %f \n", capsule3.body, capsule3.geom);

	//capsule3.joint[0] =odeJointCreateFixed();
	//odeJointAttach(capsule3.joint[0],capsule3.body,capsule2.body);
	//odeJointSetFixed(capsule3.joint[0]);
	

	capsule3.joint[1] =odeJointCreateHinge();
	odeJointAttach(capsule3.joint[1],capsule3.body,capsule2.body);
	//odeJointSetHingeAnchor(capsule3.joint[1],-1.1,0.5,0.60);
	odeJointSetHingeAxis(capsule3.joint[1],0,1,0);


	//crealting link over the ball 


	over_ball.body = odeBodyCreate();
	over_ball.geom = odeCreateCapsule(0, 0.1f, 0.2f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetCapsuleTotal(over_ball.body, 1, 0.1, 0.20f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(over_ball.body,-1.3,0.0,0.95f);
	odeGeomSetBody(over_ball.geom,over_ball.body);
	printf("capsule over the ball body id %f, geom id %f \n", over_ball.body, over_ball.geom);

	
	over_ball.joint[0] =odeJointCreateFixed();
	odeJointAttach(over_ball.joint[0],over_ball.body,capsule3.body);
	odeJointSetFixed(over_ball.joint[0]);


	//creating stage

	stage.body = odeBodyCreate();
	stage.geom = odeCreateBox(0.4f, 0.4f, 0.05f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetBoxTotal(stage.body, 0.94, 0.4f,0.4f, 0.05f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(stage.body,-1.3,0.0,1.15f);
	odeGeomSetBody(stage.geom,stage.body);
	printf("capsule stage body id %f, geom id %f \n", stage.body, stage.geom);
	
	stage.joint[0] =odeJointCreateFixed();
	odeJointAttach(stage.joint[0],stage.body,over_ball.body);
	odeJointSetFixed(stage.joint[0]);

	

	//creating obstacle 
	obs.body = odeBodyCreate();
	obs.geom = odeCreateSphere( 0.05f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetSphereTotal(obs.body, 1,0.05f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(obs.body,-1.3,0.0,1.2f);
	odeGeomSetBody(obs.geom,obs.body);
	printf("capsule obstacle body id %f, geom id %f \n", obs.body, obs.geom);
	
	obs.joint[0] =odeJointCreateFixed();
	odeJointAttach(obs.joint[0],stage.body,obs.body);
	odeJointSetFixed(obs.joint[0]);

	//creating the ball 

	ball.body = odeBodyCreate();
	ball.geom = odeCreateSphere( 0.05f); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetSphereTotal(ball.body, 0.04,0.05f); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(ball.body,-1.3f,0.0,2.2f);
	odeGeomSetBody(ball.geom,ball.body);
	printf("ball body id %f, geom id %f \n", ball.body, ball.geom);	

	

	setCurrentOdeContext(0);



	dsSimulationLoop (argc,argv,900,600,&fn);
	





}
