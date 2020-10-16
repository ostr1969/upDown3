/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleSlidingMass.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


//#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/DelpActuator.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>
#include <Moco/osimMoco.h>
//#include <Moco/Components/ActivationCoordinateActuator.h>
#include <Moco/MocoGoal/MocoGoal.h>
//#include "ActivationCoordinateActuator.h"
#include <OpenSim/Common/STOFileAdapter.h>
#include "console.h"
//#include "readdelp.h"
#include "additions.h"
#include "MocoJumpGoal.h"

using namespace OpenSim;
using namespace SimTK;

    double qi0L,qi0H,qi1L,qi1H,qi2L,qi2H,qi3L,qi3H;
    double q0L,q0H,q1L,q1H,q2L,q2H,q3L,q3H;
    double q0,q1,q2,q3;
Model buildmodel(){
	Model osimModel(data.strings[0].val);
        cout<<"read src/base3springs.osim"<<endl;
	osimModel.buildSystem();
        cout<<"built system"<<endl;
        // Pin joint initial states
        CoordinateSet &coordinates = osimModel.updCoordinateSet();
        coordinates[0].set_default_value( q0);
        coordinates[1].set_default_value( q1);
        coordinates[2].set_default_value( q2);
        coordinates[3].set_default_value( q3);
        auto& sp1=osimModel.updComponent<PathSpring>("/forceset/path_spring1");
        sp1.setStiffness(4454.76*data.ints[3].val);
        cout<<"numsprings:"<<data.ints[3].val<<endl;
        auto& sp2=osimModel.updComponent<PathSpring>("/forceset/path_spring2");
        sp2.setStiffness(4454.76*data.ints[4].val);
        auto& sp3=osimModel.updComponent<PathSpring>("/forceset/path_spring3");
        sp3.setStiffness(4454.76*data.ints[5].val);

	OpenSim::Array<std::string> actuNames;
        for (const auto& actu : osimModel.getComponentList<DelpActuator>()) {
                actuNames.append(actu.getName());
        }
        updateDelpActuator(osimModel, actuNames[0],"src/delp1.txt",.011,.068, 1,16);
        updateDelpActuator(osimModel, actuNames[1],"src/delp4.txt",.011,.068, 1,18);
        updateDelpActuator(osimModel, actuNames[2],"src/delp5.txt",.011,.068, 1,20);
        updateDelpActuator(osimModel, actuNames[3],"src/delp2.txt",.011,.068,-1,16);
        updateDelpActuator(osimModel, actuNames[4],"src/delp3.txt",.011,.068,-1,18);
        updateDelpActuator(osimModel, actuNames[5],"src/delp6.txt",.011,.068,-1,20);
//	const ControllerSet &controllerSet = osimModel.getControllerSet();
	osimModel.updControllerSet().remove(0);

        double allStiff = 10000, allDamping = 5., allTransition = 5.;
         CoordinateLimitForce* toeLimitForce = new  CoordinateLimitForce("q0", 85,
        allStiff, 0, allStiff, allDamping, allTransition);
         CoordinateLimitForce* ankleLimitForce = new  CoordinateLimitForce("q1", q1H*180/Pi-5,
        allStiff, q1L*180/Pi+5, allStiff, allDamping, allTransition);
         CoordinateLimitForce* kneeLimitForce = new  CoordinateLimitForce("q2", q2H*180/Pi-5,
        allStiff, q2L*180/Pi+5, allStiff, allDamping, allTransition);
         CoordinateLimitForce* hipLimitForce = new  CoordinateLimitForce("q3", q3H*180/Pi-5,
        allStiff, q3L*180/Pi+5, allStiff, allDamping, allTransition);
        //osimModel.addForce(toeLimitForce);
        //osimModel.addForce(ankleLimitForce);
        //osimModel.addForce(kneeLimitForce);
        //osimModel.addForce(hipLimitForce);

	osimModel.buildSystem();
        State &si = osimModel.initializeState();
        si.getQ().dump("initial q");
	//Vector stateInitVars(14,0.);
        //stateInitVars(6)=q0;stateInitVars(8)=q1;stateInitVars(10)=q2;stateInitVars(12)=q3;
        OpenSim::Array<std::string> statelabs=osimModel.getStateVariableNames();
        for (int i=0;i<statelabs.size();i++) cout<<statelabs[i]<<"\n";
        //
	//osimModel.setStateVariableValues(si,stateInitVars);
	//setting optimal forces at the initial angle and activations
        //Vector udot(4,0.);
        //InverseDynamicsSolver insol(osimModel);
        //Vector init_tou=insol.solve(si,udot);
        //cout<<"Torques for static equilibrium:"<<init_tou<<endl;
	//setInitDelpActivation(osimModel,si,init_tou);
	//acts=osimModel.getStateVariableValues(si);
        //cout<<"initsatate:(not used)"<<acts<<endl;

        

	q0L=30*Pi/180;q0H=90*Pi/180;
	auto& actuA = osimModel.updComponent<DelpActuator>("am");
	q1L=actuA.getDelpLowAngle();
	q1H=actuA.getDelpHighAngle();
	auto& actuK = osimModel.updComponent<DelpActuator>("km");
	q2L=actuK.getDelpLowAngle();
	q2H=actuK.getDelpHighAngle();
	auto& actuH = osimModel.updComponent<DelpActuator>("hm");
	q3L=actuH.getDelpLowAngle();
	q3H=actuH.getDelpHighAngle();
        cout<<"qLimits:"<<endl<<q0L<<","<<q0H<<endl<<
	q1L<<","<<q1H<<endl<<
	q2L<<","<<q2H<<endl<<
	q3L<<","<<q3H<<endl;

	//auto& tip= osimModel.updJointSet().get("tip");

        osimModel.print("results/mycolo_initial.osim");


	
	return osimModel;
	}

int main() {
    data=readvars();
    cout<<"starting moco study\n";
    MocoStudy study;Vector initActivations(14,0.);
        q0=data.doubles[4].val;q1=data.doubles[5].val;q2=data.doubles[6].val;q3=data.doubles[7].val;
	
        q0=q0*Pi/180.; q1=q1*Pi/180.; q2=q2*Pi/180.; q3=q3*Pi/180.;

    study.setName("4link");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = study.updProblem();
	//build the initial model with initial angles and get initial activations
	Model osimModel=buildmodel();

    // Model (dynamics).
    // -----------------
      problem.setModelCopy(osimModel);
    // Bounds.
    // -------
    // Initial time must be 0, final time can be within [0, 5].
    problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(
		data.doubles[3].val, data.doubles[0].val));

    // Initial position must be 0, final position must be 1.
    problem.setStateInfo("/jointset/tip/q0/value", MocoBounds(q0L,q0H),
                         {50*Pi/180,70*Pi/180}, MocoFinalBounds(q0L,q0H));
    problem.setStateInfo("/jointset/ankle/q1/value", {q1L,q1H}, q1, {q1L,q1H});
    problem.setStateInfo("/jointset/knee/q2/value",  {q2L,q2H}, q2, {q2L,q2H});
    problem.setStateInfo("/jointset/hip/q3/value",   {q3L,q3H}, {-70*Pi/180,-15*Pi/180}, {q3L,q3H});

    //problem.setStateInfo("/jointset/tip/q0/value", MocoBounds(qi0L,qi0H),
    //                     MocoInitialBounds(q0), MocoFinalBounds(qi0L,qi0H));
    //problem.setStateInfo("/jointset/ankle/q1/value", {q1l,q1h}, q1, {q1l,q1h});
    //problem.setStateInfo("/jointset/knee/q2/value",  {q2l,q2h},  q2, {q2l,q2h});
    //problem.setStateInfo("/jointset/hip/q3/value",   {q3l,q3h},  q3, {q3l,q3h});
    // Initial and final speed must be 0. Use compact syntax.
    double v=55.;
    problem.setStateInfoPattern("/jointset/.*/speed", {-v, v}, 0, {});
    //problem.setStateInfoPattern("/forceset/.*LimitForce/dissipatedEnergy",{-1e6,1e6} , 0, {});
    //problem.setStateInfo("/jointset/ankle/q1/speed", {-v, v}, 0, {});
    //problem.setStateInfo("/jointset/knee/q2/speed", {-v, v}, 0, {});
    //problem.setStateInfo("/jointset/hip/q3/speed", {-v, v}, 0, {});
    //problem.setStateInfo("/ap/activation", {0, 1}, initActivations[0], {0,1});
    problem.setStateInfo("/ap/activation", {0, 1},0, {0,1});
    problem.setStateInfo("/kp/activation", {0, 1},0, {0,1});
    problem.setStateInfo("/hp/activation", {0, 1},0, {0,1});
    problem.setStateInfo("/am/activation", {-1,0},0, {-1,0});
    problem.setStateInfo("/km/activation", {-1,0},0, {-1,0});
    problem.setStateInfo("/hm/activation", {-1,0},0, {-1,0});

    // Applied force must be between -50 and 50.
    problem.setControlInfo("/ap", MocoBounds(0.,1. ));
    problem.setControlInfo("/kp", MocoBounds(0.,1. ));
    problem.setControlInfo("/hp", MocoBounds(0.,1. ));
    problem.setControlInfo("/am", MocoBounds(-1.0,0. ));
    problem.setControlInfo("/km", MocoBounds(-1.0,0. ));
    problem.setControlInfo("/hm", MocoBounds(-1.0,0. ));
MocoParameter p0;
p0.setName("knee stiffness");
p0.appendComponentPath("/forceset/path_spring1");
p0.setPropertyName("stiffness");
MocoBounds Bounds(0, data.ints[6].val*4454.);
p0.setBounds(Bounds);
//problem.addParameter(p0);

MocoParameter p1;
p1.setName("hip stiffness");
p1.appendComponentPath("/forceset/path_spring2");
p1.setPropertyName("stiffness");
MocoBounds Bounds1(0,  data.ints[7].val*4454.);
p1.setBounds(Bounds1);
//problem.addParameter(p1);

MocoParameter p2;
p2.setName("ankle stiffness");
p2.appendComponentPath("/forceset/path_spring3");
p2.setPropertyName("stiffness");
MocoBounds Bounds2(0,  data.ints[8].val*4454.);
p2.setBounds(Bounds2);
//problem.addParameter(p2);
    // Cost.
    // -----
    //problem.addGoal<MocoFinalTimeGoal>();
    problem.addGoal<MocoJumpGoal>("Jump");
   // problem.addGoal<MocoFinalTimeGoal>("Time");
   // problem.updGoal("Time").setWeight(0.5);


    // Configure the solver.
    // =====================
   MocoCasADiSolver& solver = study.initCasADiSolver();
 //MocoTropterSolver& solver=study.initTropterSolver();
    solver.set_num_mesh_intervals(data.ints[1].val);
    //solver.set_verbosity(2);
    cout<<data.strings[1].val<<endl;
    solver.set_optim_finite_difference_scheme(data.strings[1].val);
    solver.set_optim_solver("ipopt");
    solver.set_optim_max_iterations(data.ints[0].val);
    solver.set_optim_convergence_tolerance(data.doubles[2].val);
    solver.set_optim_constraint_tolerance(data.doubles[2].val);
    solver.set_parameters_require_initsystem(data.ints[2].val);


    // Now that we've finished setting up the tool, print it to a file.
    study.print("results/mycolo.omoco");
    if (data.ints[9].val==1)
       solver.setGuessFile(data.strings[2].val);

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();
    if (solution.isSealed()){
        cout<<"*********DID NOT CONVERGED*********\n";
        return 0;}

    //solution.unseal();
    debugLog<<"got objective:"<<solution.getObjective()<<endl;
	//solution.resampleWithNumTimes(50);
    solution.write("results/mycolo_traj.sto");

    string sp1s=to_string(data.ints[3].val);
    string sp2s=to_string(data.ints[4].val);
    string sp3s=to_string(data.ints[5].val);
    TimeSeriesTable ts=solution.convertToTable();
    ts.updTableMetaData().setValueForKey(data.ints[3].label,sp1s) ;
    ts.updTableMetaData().setValueForKey(data.ints[4].label,sp2s) ;
    ts.updTableMetaData().setValueForKey(data.ints[5].label,sp3s) ;
    ts.updTableMetaData().setValueForKey(data.doubles[0].label,to_string(data.doubles[0].val)) ;
    ts.updTableMetaData().setValueForKey(data.doubles[2].label,to_string(data.doubles[2].val)) ;
    char filn[80]="results/traj";
        strcat(filn,sp1s.c_str());strcat(filn,".");strcat(filn,sp2s.c_str());strcat(filn,".");
        strcat(filn,sp3s.c_str());strcat(filn,".sto");

    STOFileAdapter::write(ts, filn);
    STOFileAdapter::write(ts, "results/lasttraj.sto");



    TimeSeriesTable statesTable=solution.exportToStatesTable();
timSeriesToBinFile(statesTable,"results/mycolo_states.bin");
   // Storage statestorage=solution.exportToStatesStorage();
    STOFileAdapter::write(statesTable, "results/mycolo_states.sto");
    TimeSeriesTable controlTable=solution.exportToControlsTable();
    STOFileAdapter::write(controlTable, "results/mycolo_controls.sto");

timSeriesToBinFile(controlTable,"results/mycolo_controls.bin");


    //solution.resampleWithNumTimes(1300);
    //solution.write("mycolo_hightraj.sto");
    // Visualize.
    // ==========
    //study.visualize(solution);
    double fwdjump=fwdCheck(osimModel , solution );
 cout<<solution.getObjectiveTermByIndex(0)<<"\t"<<solution.getObjectiveTermByIndex(0)<<endl;

    cout<<"numsprings[KHA]:"<<data.ints[3].val<<","<<data.ints[4].val<<","<<data.ints[5].val
        <<"         fwdjump:"<<fwdjump<<endl;

    return EXIT_SUCCESS;
}
