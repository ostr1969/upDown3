/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoJumpGoal.cpp                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

//#include "MocoGoal.h"

//#include <Moco/MocoGoal/MocoGoal.h>
#include <OpenSim/OpenSim.h>
#include "MocoJumpGoal.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;


void MocoJumpGoal::initializeOnModelImpl(const Model& model ) const {
    setRequirements(0, 2);
	//model.print("aaa.osim");
    
}

void MocoJumpGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
	 Vector_<SpatialVec>  forcesAtMInG;
        getModel().realizeAcceleration(input.state);
      const SimbodyMatterSubsystem& matter= getModel().getMultibodySystem().
                        getMatterSubsystem();
     matter.calcMobilizerReactionForces( input.state,forcesAtMInG);
     double f=forcesAtMInG[0][1][1];
     double fn,t1,t2,k;
    if (f>=0){
    t1=0;t2=5000;double y1=0,y2=5;
    k=std::max(0.,std::min(1.,(f-t1)/(t2-t1)));
    fn=k*k*(3-2*k)*(y2-y1)+y1;
    fn=0;}
    if (f<0){
    t1=-30000;t2=0;double y1=80000,y2=0;
    k=std::max(0.,std::min(1.,(f-t1)/(t2-t1)));
    fn=k*k*(3-2*k)*(y2-y1)+y1;
    //cout<<"f:"<<f<<" fn:"<<fn<<endl;
    fn=f*f;
	}
    integrand = fn;
//    getModel().realizeVelocity(input.state);
//    const auto& controls = getModel().getControls(input.state);
//    integrand = controls.normSqr();

}


void MocoJumpGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
	 Vector_<SpatialVec>  forcesAtMInG;
       SimTK::Real timeInitial = input.initial_state.getTime();
        SimTK::Real timeFinal = input.final_state.getTime();
        //SimTK::Vec3 comInitialV =
        //        model.calcMassCenterVelocity(input.initial_state);
        SimTK::Vec3 comFinalV =
                getModel().calcMassCenterVelocity(input.final_state);
        SimTK::Vec3 comFinalP =
                getModel().calcMassCenterPosition(input.final_state);
        SimTK::Vec3 comInitialP =
                getModel().calcMassCenterPosition(input.initial_state);
	getModel().realizeAcceleration(input.final_state);
	 getModel().getMultibodySystem().getMatterSubsystem().
			calcMobilizerReactionForces( input.final_state,forcesAtMInG);

    double vy=comFinalV(1),vx=comFinalV(0),posx=comFinalP(0),posy=comFinalP(1);
    double posx0=comInitialP(0);
    cost[0]=-vy-posy;//maximize end posy and end vy

    /*double f=forcesAtMInG[0][1][1];
    if (f>=0){
    t1=0;t2=28000;double y1=0,y2=28;
    k=std::max(0.,std::min(1.,(f-t1)/(t2-t1)));
    cost[1]=k*k*(3-2*k)*(y2-y1)+y1;}
    if (f<0){
    t1=-10000;t2=0;double y1=500,y2=0;
    k=std::max(0.,std::min(1.,(f-t1)/(t2-t1)));
    //cost[1]=k*k*(3-2*k)*(y2-y1)+y1;}
    	}*/
    double t1=-.5,t2=0,y1=10,y2=0;
    double k1=std::max(0.,std::min(1.,(posx-t1)/(t2-t1)));
    double dirac1=k1*k1*(3-2*k1)*(y2-y1)+y1;
    double k0=std::max(0.,std::min(1.,(posx0-t1)/(t2-t1)));
    double dirac0=k0*k0*(3-2*k0)*(y2-y1)+y1;
    //cost[0]=-vy*vy/2./9.81*dirac-comFinalP(1);
    //cost[0] = dirac0;
    cost[1] = dirac1;//positive punishment for negative end posx	
    //cost[2] = input.integral;
    //cout<<comFinalP<<","<<cost<<endl;
}
