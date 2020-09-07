
/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine.  This application acts as an example for utilizing the 
 *  ControllabeSpring actuator.
 *
 * compare this code to the matlab code compareRBDL4 in :
 * C:\Users\ostr\Google Drive (ostr@post.bgu.ac.il)\PHD\C++\RBDL
 * activate ./myactuator 0.25 and look at the last line of
 * 4link_states_degrees.sto for the angle and velocities
 * the initial angles are read from  T0_0_140_0.txt(read the remark below
 * this file changed from the file in windows dir)
 * the torques are read from tou0_140_0.txt. 
*/
//==============================================================================
//==============================================================================
//#include "PistonActuator.h"
//#include "ControllableSpring.h"
#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/DelpActuator.h>
#include "OpenSim/Common/STOFileAdapter.h"
#define BUILD 1
#include "additions.h"

using std::string;
#define MAXN 1000
using namespace OpenSim;
using namespace SimTK;
using namespace std::chrono;
using  std::cout;
using  std::endl;
int main(int argc, char *argv[])
{
try
{

        // Create a new OpenSim model.
        Model osimModel;
        osimModel.setName("4linkModel");
        osimModel.setAuthors("barak ostraich");

            
        // Get the ground body.
        Ground& ground = osimModel.updGround();
        ground.attachGeometry(new Mesh("checkered_floor.vtp"));

        // Create first linkage body.
        double linkageMass1 = 2.32 , linkageLength1 = 0.2084, linkageDiameter = 0.01;
        double linkageMass2 = 7.44 , linkageLength2 = 0.4182;
        double linkageMass3 = 16   , linkageLength3 = 0.4165;
        double linkageMass4 = 54.24, linkageLength4 = 0.7780;
        
        Vec3 linkageMassCenter1(0.,linkageLength1-0.1140836,0.);
        Vec3 linkageMassCenter2(0.,linkageLength2-0.18647538,0.);
        Vec3 linkageMassCenter3(0.,linkageLength3-0.17055675,0.);
        Vec3 linkageMassCenter4(0.,linkageLength4-0.4237,0.);
        double exothick=0.025;
        Vec3 exocenter(0.,exothick/2,0.);

        OpenSim::Body* linkage1 = new OpenSim::Body("foot", linkageMass1,
                linkageMassCenter1, Inertia(0.1,0.1,0.00662,0.,0.,0.));
        OpenSim::Body* linkage2 = new OpenSim::Body("shank", linkageMass2,
                linkageMassCenter2, Inertia(0.1,0.1,0.1057,0.,0.,0.));
        OpenSim::Body* linkage3 = new OpenSim::Body("thigh", linkageMass3,
                linkageMassCenter3, Inertia(0.2,0.1,0.217584,0.,0.,0.));
        OpenSim::Body* linkage4 = new OpenSim::Body("HAT", linkageMass4,
                linkageMassCenter4, Inertia(1.1,1.1,1.48,0.,0.,0.));

        //create z vector for all joints 
        Rotation R = Rotation(-Pi/2, ZAxis); 
        // Graphical representation of foot.
        Sphere sphere(0.04);
        Cylinder exo(.04, exothick);
        linkage1->attachGeometry(sphere.clone());
        Cylinder cyl1(linkageDiameter/2, linkageLength1/2);
        Frame* cyl1Frame = new PhysicalOffsetFrame(*linkage1, 
            Transform(Vec3(0.0, linkageLength1/2 , 0.0)));
        cyl1Frame->setName("Cyl1_frame");
        cyl1Frame->attachGeometry(cyl1.clone());
        cyl1Frame->attachGeometry(exo.clone());
        // cyl1Frame->attachGeometry(new Mesh("foot.vtp"));
        osimModel.addComponent(cyl1Frame);

        // Create shank body.
        linkage2->attachGeometry(sphere.clone());
        Cylinder cyl2(linkageDiameter/2, linkageLength2/2);
        Frame* cyl2Frame = new PhysicalOffsetFrame(*linkage2,
            Transform(Vec3(0.0, linkageLength2/2 , 0.0)));
        cyl2Frame->setName("Cyl2_frame");
        cyl2Frame->attachGeometry(cyl2.clone());
        cyl2Frame->attachGeometry(exo.clone());
        //cyl2Frame->attachGeometry(new Mesh("tibia_r.vtp"));
        osimModel.addComponent(cyl2Frame);
        // Create thigh body.
        linkage3->attachGeometry(sphere.clone());
        Cylinder cyl3(linkageDiameter/2, linkageLength3/2);
        Frame* cyl3Frame = new PhysicalOffsetFrame(*linkage3,
            Transform(Vec3(0.0, linkageLength3/2 , 0.0)));
        cyl3Frame->setName("Cyl3_frame");
        cyl3Frame->attachGeometry(cyl3.clone());
        cyl3Frame->attachGeometry(exo.clone());
        //cyl3Frame->attachGeometry(new Mesh("femur_r.vtp"));
        osimModel.addComponent(cyl3Frame);
        // Create HAT body.
        linkage4->attachGeometry(sphere.clone());
        Cylinder cyl4(linkageDiameter/2, linkageLength4/2);
        Frame* cyl4Frame = new PhysicalOffsetFrame(*linkage4,
            Transform(Vec3(0.0, linkageLength4 / 2.0, 0.0)));
        Frame* exo4Frame = new PhysicalOffsetFrame(*linkage4,
            Transform(Vec3(0.0, 0.2, 0.0)));
        cyl4Frame->setName("Cyl4_frame");
        cyl4Frame->attachGeometry(cyl4.clone());
        exo4Frame->attachGeometry(exo.clone());
        osimModel.addComponent(cyl4Frame);
        osimModel.addComponent(exo4Frame);

        // Create 1 degree-of-freedom pin joints between the bodies to create a
        // kinematic chain from ground through the block.
        Vec3 orientationInGround(0.);
        Vec3 locationInGround(0.);
        Vec3 locationInParent1(0.0, linkageLength1, 0.0);
        Vec3 locationInParent2(0.0, linkageLength2, 0.0);
        Vec3 locationInParent3(0.0, linkageLength3, 0.0);
        Vec3 locationInParent4(0.0, linkageLength4, 0.0);
        Vec3 orientationInChild(0.);
        Vec3 locationInChild(0.);

        PinJoint* tip   = new PinJoint("tip",
                ground, locationInGround, orientationInGround,
                *linkage1, locationInChild, orientationInChild);

        PinJoint* ankle = new PinJoint("ankle",
                *linkage1, locationInParent1, orientationInChild,
                *linkage2, locationInChild, orientationInChild);

        PinJoint* knee = new PinJoint("knee",
                *linkage2, locationInParent2, orientationInChild,
                *linkage3, locationInChild, orientationInChild);

        PinJoint* hip = new PinJoint("hip",
                *linkage3, locationInParent3, orientationInChild,
                *linkage4, locationInChild, orientationInChild);
        
    /*	double allStiff = 10000, allDamping = 5., allTransition = 10.;
    	auto toeLimitForce = new CoordinateLimitForce("q0", toeRange[1]*180/Pi,
        allStiff, toeRange[0]*180/Pi, allStiff, allDamping, allTransition);
    	auto ankleLimitForce = new CoordinateLimitForce("q1", ankleRange[1]*180/Pi,
        allStiff, ankleRange[0]*180/Pi, allStiff, allDamping, allTransition);
    	auto kneeLimitForce = new CoordinateLimitForce("q2", kneeRange[1]*180/Pi,
        allStiff, kneeRange[0]*180/Pi, allStiff, allDamping, allTransition);
    	auto hipLimitForce = new CoordinateLimitForce("q3", hipRange[1]*180/Pi,
        allStiff, hipRange[0]*180/Pi, allStiff, allDamping, allTransition);
    	//tip->addComponent(toeLimitForce);
    	//ankle->addComponent(ankleLimitForce);
    	//knee->addComponent(kneeLimitForce);
    	//hip->addComponent(hipLimitForce);*/



        // Add the bodies to the model
        osimModel.addBody(linkage1);
        osimModel.addBody(linkage2);
        osimModel.addBody(linkage3);
        osimModel.addBody(linkage4);

        // Add the joints to the model
        osimModel.addJoint(tip);
        osimModel.addJoint(ankle);
        osimModel.addJoint(knee);
        osimModel.addJoint(hip);

        tip->updCoordinate().setName("q0");
        ankle->updCoordinate().setName("q1");
        knee->updCoordinate().setName("q2");
        hip->updCoordinate().setName("q3");
        
        // define the simulation times
 	DelpActuator* a1=addDelpActuator(osimModel, "q1","ap","src/delp1.txt", 1,16);
 	DelpActuator* a2=addDelpActuator(osimModel, "q2","kp","src/delp4.txt", 1,18);
 	DelpActuator* a3=addDelpActuator(osimModel, "q3","hp","src/delp5.txt", 1,20);
 	DelpActuator* a_1=addDelpActuator(osimModel, "q1","am","src/delp2.txt", 1,16);
 	DelpActuator* a_2=addDelpActuator(osimModel, "q2","km","src/delp3.txt", 1,18);
 	DelpActuator* a_3=addDelpActuator(osimModel, "q3","hm","src/delp6.txt", 1,20);

        double toeRange[2] = {0, Pi/2};
        double ankleRange[2]={a1->getDelpLowAngle(),a1->getDelpHighAngle()};
        double kneeRange[2] ={a2->getDelpLowAngle(),a2->getDelpHighAngle()} ;
        double hipRange[2] ={a3->getDelpLowAngle(),a3->getDelpHighAngle()} ;
        cout<<ankleRange[0]*180/Pi<<","<<ankleRange[1]*180/Pi<<endl;
        cout<<kneeRange[0]*180/Pi<<","<<kneeRange[1]*180/Pi<<endl;
        cout<<hipRange[0]*180/Pi<<","<<hipRange[1]*180/Pi<<endl;

        tip->updCoordinate().setRange(toeRange);
        ankle->updCoordinate().setRange(ankleRange);
        knee->updCoordinate().setRange(kneeRange);
        hip->updCoordinate().setRange(hipRange);
        // add the controller to the model
        PrescribedController* actcontroller =  new PrescribedController();
        actcontroller->addActuator(*a1);
        actcontroller->addActuator(*a2);
        actcontroller->addActuator(*a3);
        actcontroller->addActuator(*a_1);
        actcontroller->addActuator(*a_2);
        actcontroller->addActuator(*a_3);

        actcontroller->prescribeControlForActuator("ap", new Constant(0));
        actcontroller->prescribeControlForActuator("kp", new Constant(0));
        actcontroller->prescribeControlForActuator("hp", new Constant(0));
        actcontroller->prescribeControlForActuator("am", new Constant(0));
        actcontroller->prescribeControlForActuator("km", new Constant(0));
        actcontroller->prescribeControlForActuator("hm", new Constant(0));
        actcontroller->set_interpolation_method(3);
    
        osimModel.addController(actcontroller);

        //const ControllerSet &cs= osimModel.getControllerSet();
        osimModel.setGravity(Vec3(0., -9.81   , 0.));
        // debugging.
	
//start wrap spring
//      original length is 8 cm and stiffenes is 350N to each 8cm---for each spring
//      350N/0.08m=4375
	double pullymass=.001,resting_length=0.4,stiffness=4454.76*4.,dissipation=0.01;
	double pullyrad=0.05,pullylength=0.05;
           // body that acts as the pulley that the path wraps over

cout<<__LINE__<<endl;
        OpenSim::Body* pulleyBody1 =
        new OpenSim::Body("PulleyBody1", pullymass ,Vec3(0),  pullymass*Inertia::sphere(0.1));
        OpenSim::Body* pulleyBody2 =
        new OpenSim::Body("PulleyBody2", pullymass ,Vec3(0),  pullymass*Inertia::sphere(0.1));
        OpenSim::Body* pulleyBody3 =
        new OpenSim::Body("PulleyBody3", pullymass ,Vec3(0),  pullymass*Inertia::sphere(0.1));

	WrapCylinder* pulley1 = new WrapCylinder();
    	pulley1->set_radius(pullyrad); pulley1->set_length(pullylength); pulley1->set_quadrant("-y");
	pulley1->setName("wrap1");
	WrapCylinder* pulley2 = new WrapCylinder();
    	pulley2->set_radius(pullyrad); pulley2->set_length(pullylength); pulley2->set_quadrant("-x");
	pulley2->setName("wrap2");
	WrapCylinder* pulley3 = new WrapCylinder();
    	pulley3->set_radius(pullyrad); pulley3->set_length(pullylength); pulley3->set_quadrant("-x");
	pulley3->setName("wrap3");
    	
	// Add the wrap object to the body, which takes ownership of it

    	pulleyBody1->addWrapObject(pulley1);
    	pulleyBody2->addWrapObject(pulley2);
    	pulleyBody3->addWrapObject(pulley3);
    	osimModel.addBody(pulleyBody1);
    	osimModel.addBody(pulleyBody2);
    	osimModel.addBody(pulleyBody3);
        WeldJoint* weld1 =
        new WeldJoint("weld1", *linkage3, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody1, Vec3(0), Vec3(0));
        WeldJoint* weld2 =
        new WeldJoint("weld2", *linkage4, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody2, Vec3(0), Vec3(0));
        WeldJoint* weld3 =
        new WeldJoint("weld3", *linkage2, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody3, Vec3(0), Vec3(0));
        osimModel.addJoint(weld1);
        osimModel.addJoint(weld2);
        osimModel.addJoint(weld3);

    	PathSpring* spring1 =
        new PathSpring("path_spring1",resting_length,stiffness ,dissipation);
    	spring1->updGeometryPath().
        appendNewPathPoint("origin1", *linkage3, Vec3(pullyrad, 0.2, 0));
    	spring1->updGeometryPath().
        appendNewPathPoint("insert1", *linkage2, Vec3(pullyrad,linkageLength2-.2,0));
    	spring1->updGeometryPath().addPathWrap(*pulley1);
    	PathSpring* spring2 =
        new PathSpring("path_spring2",resting_length,stiffness ,dissipation);
    	spring2->updGeometryPath().
        appendNewPathPoint("origin2", *linkage4, Vec3(-pullyrad, 0.2, 0));
    	spring2->updGeometryPath().
        appendNewPathPoint("insert2", *linkage3, Vec3(-pullyrad,linkageLength3-.2,0));
    	spring2->updGeometryPath().addPathWrap(*pulley2);
    	PathSpring* spring3 =
        new PathSpring("path_spring3",0.3,stiffness ,dissipation);
    	spring3->updGeometryPath().
        appendNewPathPoint("origin3", *linkage2, Vec3(-pullyrad, 0.2, 0));
    	spring3->updGeometryPath().
        appendNewPathPoint("insert3", *linkage1, Vec3(-pullyrad,linkageLength1-.1,0));
    	spring3->updGeometryPath().addPathWrap(*pulley3);
cout<<__LINE__<<endl;
    	osimModel.addForce(spring1);
    	osimModel.addForce(spring2);
    	osimModel.addForce(spring3);
        osimModel.finalizeConnections();
cout<<__LINE__<<endl;
 

        // Initialize system
        osimModel.buildSystem();
cout<<__LINE__<<endl;
        //default activation must come before initstate

        State &si = osimModel.initializeState();
cout<<__LINE__<<endl;

       double q0=80,q1=-110,q2=60,q3=-80 ;
        // Pin joint initial states
        CoordinateSet &coordinates = osimModel.updCoordinateSet();
        coordinates[0].setValue(si, q0*Pi/180, true);
        coordinates[1].setValue(si,q1*Pi/180, true);
        coordinates[2].setValue(si, q2*Pi/180, true);
        coordinates[3].setValue(si,q3*Pi/180, true);

        // Setup ForceReporter and Manager
        ForceReporter* forces = new ForceReporter(&osimModel);  
        osimModel.updAnalysisSet().adoptAndAppend(forces);
cout<<__LINE__<<endl;
//
//find initial optimal values for initial state(after setting angles)
	Vector udot(4,0.);
        InverseDynamicsSolver insol(osimModel);
        Vector init_tou=insol.solve(si,udot);
        cout<<"torques for static equilibrium:"<<init_tou<<endl;

        si.getQ().dump("Initial q's");
        si.getU().dump("Initial u's");
        si.setTime(0);
cout<<__LINE__<<endl;


        
        cout<<"___________\nSpringdata:"<<endl;
        osimModel.getMultibodySystem().realize(si, Stage::Position);
        osimModel.getMultibodySystem().realize(si, Stage::Velocity);
        cout<<"spring length:"<<spring1->getLength(si)<<"\t"<<spring2->getLength(si)
	<<"\t"<<spring3->getLength(si)<<endl;
        cout<<"spring stretch:"<<spring1->getStretch(si)<<"\t"<<spring2->getStretch(si)
	<<"\t"<<spring3->getStretch(si)<<endl;
        cout<<"spring tension:"<<spring1->getTension(si)<<"\t"<<spring2->getTension(si)
	<<"\t"<<spring3->getTension(si)<<endl;
        //cout<<"spring arm:"<<spring1->computeMomentArm(si, coordinates[2])
        
        osimModel.print("results/base3springs.osim");
cout<<__LINE__<<endl;
    }
    catch (const std::exception& ex)
    {
        std::cout << "Exception in 4links_example: " << ex.what() << std::endl;
        return 1;
    }

    cout << "Done." << endl;
    return 0;
}
