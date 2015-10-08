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

dsFunctions fn;
using namespace AaltoGames;

float h_floor_table = 0.85f ;
float h_base = 0.32f ;
float h_first= 0.60f ;
float h_main = 0.12f ;
float h_sphere= 0.08f;
float h_support = 0.20f ;
float stage_dim[3]= {0.04f, 0.04f, 0.05f };


const int nSamples=50;
float timeStep=1.0f/30.0f;
ControlPBP pbp;
int nTimeSteps=10;		
const int nStateDimensions=2;
const int nControlDimensions=1;
float minControl=-2;	
float maxControl=2;		
float controlMean=0;	
float C=10;
float controlStd=0.8f*C;	
float controlDiffStd=100.0f*C;	
float controlDiffDiffStd=1000.0f*C; 
float mutationScale=0.25f;





class objects
{
public:
 dReal body;
 dReal geom;
 dReal joint;
 dReal joint_extra;
 float radius;
 float length;
};

objects stage, support, LinkBall, mainLink,firstLink,base, ball, obs;

void robot(int com)
{

	setCurrentOdeContext(0);
	restoreOdeState(0);
	saveOdeState(0);


	
	//init all trajectories to the master state
	for (int i=0; i<nSamples; i++)
	{
		//activate the context for this sample
		setCurrentOdeContext(i+1);
		//load the state from the master context
		restoreOdeState(0);
		saveOdeState(i+1,0);


	}



	setCurrentOdeContext(0);
	restoreOdeState(0);

	const dReal *pos = odeBodyGetPosition(ball.body);
	float angle=odeJointGetHingeAngle(LinkBall.joint);
	float stateVector[2]={pos[0],angle};
	pbp.startIteration(true,stateVector);


	//simulate forward
	for (int k=0; k<nTimeSteps; k++)

	{

	


		//signal the start of a planning step
		pbp.startPlanningStep(k);
					//NOTE: for multithreaded operation, one would typically run each iteration of the following loop in a separate thread.
		//The getControl(), getPreviousSampleIdx(), and updateResults() methods of the optimizer are thread-safe.
		//The physics simulation is also thread-safe as we have full copies of the simulated system for each thread/trajectory


		for (int i=0; i<nSamples; i++)
		{
			//get control from C-PBP
			float control;
			pbp.getControl(i,&control);
		
			//get the mapping from this to previous state (affected by resampling operations)
			int previousStateIdx=pbp.getPreviousSampleIdx(i);
			//simulate to get next state.
			setCurrentOdeContext(i+1);
			//printf("previous state id: %d \n",previousStateIdx);
			restoreOdeState(previousStateIdx+1); //continue the a trajectory (selected in the resampling operation inside ControlPBP)

	

			dReal Gain = 1;
			dReal MaxForce = dInfinity;
			odeJointSetHingeParam(LinkBall.joint,dParamFMax,dInfinity);
			odeJointSetHingeParam(LinkBall.joint,dParamVel,control  );

     
			stepOde(0);
			
			
        

			const dReal *pos = odeBodyGetPosition(ball.body);
			float angle=odeJointGetHingeAngle(LinkBall.joint);


			


			float cost=squared((0.01-pos[0])*1050.0f);

	

		
			//store the state and cost to C-PBP. Note that in general, the stored state does not need to contain full simulation state as 					in this simple case.
			//instead, one may use arbitrary state features
			float stateVector[2]={pos[0],angle};

			//float stateVector[2]={angle,aVel};
			pbp.updateResults(i,&control,stateVector,cost);

			//printf("cost in the loop end: %f posz: %f \n",cost,pos[2]);
		}
			



		//save all states, will be used at next step (the restoreOdeState() call above)

		for (int i=0; i<nSamples; i++)
		{
			saveOdeState(i+1,i+1);
		}

			//signal the end of the planning step. this normalizes the state costs etc. for the next step
			pbp.endPlanningStep(k);
	}

	
	pbp.endIteration();

	//deploy the best control found using the master context
	float control;
	pbp.getBestControl(0,&control);
	float cost=(float)pbp.getSquaredCost();
	
	setCurrentOdeContext(0);
	restoreOdeState(0);
	stepOde(0);
	//save the resulting state as the starting state for the next iteration
	saveOdeState(0);


	pos = odeBodyGetPosition(ball.body);
	angle=odeJointGetHingeAngle(LinkBall.joint);




	printf("FINAL Posx %1.3f,posz = %f  angle %1.3f, cost=%1.3f, control %f \n",pos[0],pos[2],angle*180/3.1416,cost,control);
	//printf("position sim %f  angle %1.3f \n",pos[0],angle*180/3.1416);
	//printf("original  ball %f  org angle %f \n", pos_robot, ang_robot*180/3.1416);

	   
	/*	
	int i = 0;
	loop : cin >> i;
	if ( i != 1)
	goto loop;*/








	
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
	initOde(nSamples+1);
	setCurrentOdeContext(ALLTHREADS);
	odeRandSetSeed(0);
	odeSetContactSoftCFM(0);
	odeWorldSetGravity(0, 0, -9.81f);


	//creating stage

	stage.body = odeBodyCreate();
	stage.geom = odeCreateBox(0.4f, 0.4f, 0.05f);
	odeMassSetBoxTotal(stage.body, 0.94, 0.4f,0.4f, 0.05f); 
	odeBodySetPosition(stage.body,0,0.0,h_floor_table+h_base+h_sphere+h_support+stage_dim[2]/2);
	odeGeomSetBody(stage.geom,stage.body);
	printf(" Stage body id %f, geom id %f \n", stage.body, stage.geom);
	
	

	const dReal *pos;
	pos= odeBodyGetPosition(stage.body);
	printf("stage position x%f y%f z%f   \n",pos[0],pos[1], pos[2]);


	//creating support //
	//odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	support.radius = 0.05f ;
	support.length = 0.2f;	

	support.body = odeBodyCreate();
	support.geom = odeCreateCapsule(0, support.radius, support.length-support.radius); 
	odeMassSetCapsuleTotal(support.body, 0.5f, support.radius, support.length-support.radius); 
	odeBodySetPosition(support.body,0.0f,0.0f,h_floor_table+h_base+h_sphere+(h_support/2)-stage_dim[2]/2);
	odeGeomSetBody(support.geom,support.body);
	printf("Support capsule  body id %f, geom id %f \n", support.body, support.geom);

	
	
	pos= odeBodyGetPosition(support.body);
	printf("stage position %f   \n", pos[2]);


	//creating ball link
	LinkBall.radius = 0.08f ; 

	LinkBall.body = odeBodyCreate();
	LinkBall.geom = odeCreateSphere( LinkBall.radius);
	odeMassSetSphereTotal(LinkBall.body, 0.5,LinkBall.radius); 
	odeBodySetPosition(LinkBall.body,0,0,h_floor_table+h_base-support.radius);
	odeGeomSetBody(LinkBall.geom,LinkBall.body);
	printf("Ball link body id %f, geom id %f \n", LinkBall.body, LinkBall.geom);



	



	//creating main link
	mainLink.radius= 0.05f;
	mainLink.length= 0.12f;

	mainLink.body = odeBodyCreate();
	mainLink.geom = odeCreateCapsule(0, mainLink.radius, mainLink.length-mainLink.radius); 
	odeMassSetCapsuleTotal(mainLink.body, 0.5,mainLink.radius, mainLink.length-mainLink.length); 
	odeBodySetPosition(mainLink.body,(mainLink.length+mainLink.radius),0, h_floor_table+h_base-mainLink.radius);
	odeGeomSetBody(mainLink.geom,mainLink.body);
	printf("capsule main link link body id %f, geom id %f \n", mainLink.body, mainLink.geom);




	dQuaternion q1;
	dQFromAxisAndAngle (q1,0,1,0,1.57) ;   //dQFromAxisAndAngle (dQuaternion q, dReal ax, dReal ay, dReal az, dReal angle);
	odeGeomSetQuaternion(mainLink.geom,q1);

	pos= odeBodyGetPosition(mainLink.body);
	printf("main link position x %f y %f z%f    \n",pos[0],pos[1], pos[2]);	

	
	




	
	//creating the first link
	firstLink.radius= 0.05f;
	firstLink.length= 0.60f;


	firstLink.body = odeBodyCreate();
	firstLink.geom = odeCreateCapsule(0, firstLink.radius, firstLink.length-firstLink.radius); 
	odeMassSetCapsuleTotal(firstLink.body, 0.7, firstLink.radius, firstLink.length-firstLink.radius); 
	odeBodySetPosition(firstLink.body,(firstLink.length-firstLink.radius/2),0,h_floor_table+h_base-firstLink.radius);
	odeGeomSetBody(firstLink.geom,firstLink.body);
	printf("capsule first link body id %f, geom id %f \n", firstLink.body, firstLink.geom);


	
		
	dQuaternion q;
	dQFromAxisAndAngle (q,0,1,0,1.57) ;   //dQFromAxisAndAngle (dQuaternion q, dReal ax, dReal ay, dReal az, dReal angle);
	odeGeomSetQuaternion(firstLink.geom,q);


	pos= odeBodyGetPosition(firstLink.body);
	printf("First link position x %f y %f z%f    \n",pos[0],pos[1], pos[2]);	

	


	//creating base
	base.radius= 0.05f;
	base.length= 0.32f+h_floor_table;

	base.body = odeBodyCreate();
	base.geom = odeCreateCapsule(0, base.radius, base.length-base.radius); 
	odeMassSetCapsuleTotal(base.body, 0.7, base.radius, base.length-base.radius); 
	odeBodySetPosition(base.body,(h_sphere+h_main+h_first+base.radius),0, (base.length+base.radius)/2-firstLink.radius);
	odeGeomSetBody(base.geom,base.body);
	printf("capsule base id %f, geom id %f \n", base.body, base.geom);


	//Creating Ball 
	
	ball.radius = 0.03f ; 
	
	ball.body = odeBodyCreate();
	ball.geom = odeCreateSphere( ball.radius); //odeCreateCapsule(int spaceId, float radius, float length)
	odeMassSetSphereTotal(ball.body, 0.04,ball.radius); //odeMassSetCapsuleTotal(int bodyId, float total_mass, float radius, float length)
	odeBodySetPosition(ball.body,0.1,0.0,2.2f);
	odeGeomSetBody(ball.geom,ball.body);
	printf("ball body id %f, geom id %f \n", ball.body, ball.geom);	


      	//creating Obstacle
	obs.radius = 0.05f;


	obs.body = odeBodyCreate();
	obs.geom = odeCreateSphere( obs.radius); 
	odeMassSetSphereTotal(obs.body, 0.05,obs.radius); 
	odeBodySetPosition(obs.body,0,0.0,h_floor_table+h_base+h_sphere+h_support+stage_dim[2]+obs.radius);
	odeGeomSetBody(obs.geom,obs.body);
	printf("capsule obstacle body id %f, geom id %f \n", obs.body, obs.geom);
	
	








	

	 
	////FIXING ALL JOINTS//

	//base joint with ground
	base.joint =odeJointCreateFixed();
	odeJointAttach(base.joint,base.body,0);
	odeJointSetFixed(base.joint);


	//FirstLink with world  
	firstLink.joint =odeJointCreateFixed();
	odeJointAttach(firstLink.joint,firstLink.body,0);
	odeJointSetFixed(firstLink.joint);

	
	// Main link and first link
	mainLink.joint =odeJointCreateHinge();
	odeJointAttach(mainLink.joint,mainLink.body,0);
	//odeJointSetHingeAnchor(capsule2.joint[0],-0.5,0,0.60);
	odeJointSetHingeAxis(mainLink.joint,1,0,0);

	
	//Ball link and Main link	
	LinkBall.joint =odeJointCreateHinge();
	odeJointAttach(LinkBall.joint,LinkBall.body,mainLink.body);
	//odeJointSetHingeAnchor(capsule3.joint[1],-1.1,0.5,0.60);
	odeJointSetHingeAxis(LinkBall.joint,0,1,0);


	// Support and Ball link
	support.joint =odeJointCreateFixed();
	odeJointAttach(support.joint,support.body,LinkBall.body);
	odeJointSetFixed(support.joint);

	//Stage and support link

	stage.joint =odeJointCreateFixed();
	odeJointAttach(stage.joint,stage.body,support.body);
	odeJointSetFixed(stage.joint);


	//Stage and Obstacle
	obs.joint =odeJointCreateFixed();
	odeJointAttach(obs.joint,stage.body,obs.body);
	odeJointSetFixed(obs.joint);



	setCurrentOdeContext(0);
	saveOdeState(0);






	pbp.init(nSamples,nTimeSteps,nStateDimensions,nControlDimensions,&minControl,&maxControl,&controlMean,&controlStd,&controlDiffStd,&controlDiffDiffStd,mutationScale,NULL);

	pbp.setParams(0.1f,0.5f,true,0.001f);






	
	dsSimulationLoop (argc,argv,900,600,&fn);



	

}
	



















