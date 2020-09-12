//#include <Moco/Components/ActivationCoordinateActuator.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <Moco/osimMoco.h>
#include "myPrescribedController.cpp"
//#include "console.h"
using namespace OpenSim;
using namespace SimTK;
using std::cout;
using std::endl;

template<typename T>
struct Var{
T val;
string label;};

struct InpVars{
vector<Var<string>> strings;
vector<Var<int>> ints;
vector<Var<double>> doubles;
};
InpVars readvars(){
  //FILE * ifile;
  //ifile = fopen ("src/input.txt","r");
  InpVars data;string typ,label,val;
  std::ifstream ifile("src/inpdata.txt");
  while (ifile>>typ>>label>>val) {
	//ifile>>typ>>label>>val;
	if (!typ.compare("d"))
	{Var<double> tmp;tmp.label=label;tmp.val=atof(val.c_str());
           data.doubles.push_back(tmp);}
        if (!typ.compare("i"))
	{Var<int> tmp;tmp.label=label;tmp.val=atoi(val.c_str());
           data.ints.push_back(tmp);}
        if (!typ.compare("s"))
	{Var<string> tmp;tmp.label=label;tmp.val=val;
           data.strings.push_back(tmp);}
}
  ifile.close();
return data;
}   
InpVars data;
 void updateDelpActuator(Model& osimModel,string actu,string filename,double activation,
                double deactivation,int qfac_dir,double maxvel){
        auto& actuP = osimModel.updComponent<DelpActuator>(actu);
        actuP.SetCurve(filename);
        actuP.set_activation_time_constant(activation);
        actuP.set_dactivation_time_constant(deactivation);
        actuP.set_qfac_dir(qfac_dir);
        actuP.set_max_velocity(maxvel);

}
int setInitDelpActivation(Model& osimModel,State& osimState,Vector init_tou){
        auto& actuN = osimModel.updComponent<DelpActuator>("/ap");
        double optmax=actuN.getDelpOptimal(osimState);//get the optimal for this angle
        actuN.setStateVariableValue(osimState,"activation",init_tou(1)/optmax);

        auto& actuN2 = osimModel.updComponent<DelpActuator>("/km");
        optmax=actuN2.getDelpOptimal(osimState);//get the optimal for this angle
        actuN2.setStateVariableValue(osimState,"activation",init_tou(2)/optmax);

        auto& actuN3 = osimModel.updComponent<DelpActuator>("/hp");
        optmax=actuN3.getDelpOptimal(osimState);//get the optimal for this angle
        actuN3.setStateVariableValue(osimState,"activation",init_tou(3)/optmax);
}
 DelpActuator* addDelpActuator(Model& model, std::string coordName,
        string actuname,string filename,int qfac_dir,double maxvel) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new DelpActuator();
//"Smaller value means activation can change more rapidly "
    actu->set_activation_time_constant(0.011);
    actu->set_dactivation_time_constant(0.067);
    actu->set_default_activation(0.);
    actu->setName(actuname);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(1);
    actu->setMinControl(-1);
    actu->setMaxControl(1);

    actu->SetCurve(filename);
    actu->set_qfac_dir(qfac_dir);
    actu->set_max_velocity(maxvel);


    model.addComponent(actu);
    return actu;
}
PathSpring* addWrapSpring(
	string springname,double resting_length,double stiffness,double dissipation,
	 OpenSim::Body orglink,Vec3 orgpoint, OpenSim::Body insertlink,Vec3 insertpoint,
		WrapCylinder pulley) 
{
//add cylinder as wrap object and a spring connected to two links
    	PathSpring* spring =
        new PathSpring(springname,resting_length,stiffness ,dissipation);
    	spring->updGeometryPath().
        appendNewPathPoint("origin", orglink, orgpoint);
    	spring->updGeometryPath().
        appendNewPathPoint("insert", insertlink, insertpoint);
    	spring->updGeometryPath().addPathWrap(pulley);
return spring;

}
#ifndef BUILD
#ifndef MYFWD 
ofstream debugLog("results/mycolo_debug.csv", ofstream::out);
ofstream fwddebugLog("results/fwd_debug.csv", ofstream::out);


class MyReporter : public PeriodicEventReporter {
public:
    MyReporter(const MultibodySystem& system, Real interval)
            : PeriodicEventReporter(interval), system(system) {}

    void handleEvent(const State& state) const override {
        Vector_<SpatialVec>  forcesAtMInG;
        system.realize(state, Stage::Acceleration);
        Vec3 COM_position = system.getMatterSubsystem().
                        calcSystemMassCenterLocationInGround(state);

        system.getMatterSubsystem().calcMobilizerReactionForces( state,forcesAtMInG);
        std::cout << state.getTime() << "\t" << forcesAtMInG[0][1][1] <<
			 "\t" << COM_position[1] << std::endl;
    }
private:
    const MultibodySystem& system;
};
void trajToBinFile(MocoTrajectory solution, string filename)
{   
	//write to binary trajectory, int numrows,int numcols,
	//double*numrows for time
	//double*numrows for each actuator
    	const SimTK::Vector& time = solution.getTime();
        vector<string>actuNames=solution.getControlNames();
        int numrows=time.nrow();int numactu=actuNames.size();
        ofstream contfile;
        contfile.open(filename,ofstream::binary);
        if (contfile.is_open()){
        contfile.write((char*)&numrows,sizeof(int));
        contfile.write((char*)&numactu,sizeof(int));
        for(int i=0;i<time.nrow();i++) contfile.write((char*)&time(i),sizeof(double));
        cout<<numrows<<","<<numactu<<endl;
        for(int j=0;j<numactu;j++){
        const auto con = solution.getControl(actuNames[j]);
        cout<<actuNames[j]<<endl;
        for(int i=0;i<time.nrow();i++) contfile.write((char*)&con(i),sizeof(double));}
        contfile.close();
        cout<<"Wrote trajectory:"<<numrows<<" rows and "<<numactu<<" actuators to "<<filename<<endl;
        }
        else cout<<"cant open file writing\n";
}
 

//
double fwdCheck(Model &osimModel ,MocoTrajectory solution){
	Storage results=solution.exportToStatesStorage();
         cout<<"getstatevector..."<<endl;
	StateVector* state = results.getStateVector(0);
         cout<<"getcolumlabels..."<<endl;
        Array<string> labs=results.getColumnLabels();
         cout<<"getdata..."<<endl;
	Array<double>& st=state->getData();
        double s1=solution.getParameter("knee stiffness");
        double s2=solution.getParameter("hip stiffness");
        double s3=solution.getParameter("ankle stiffness");
	//osimModel.printSubcomponentInfo();
	auto& sp1=osimModel.updComponent<PathSpring>("/forceset/path_spring1");
        cout<<"rss"<<endl;
        sp1.setStiffness(s1);
	auto& sp2=osimModel.updComponent<PathSpring>("/forceset/path_spring2");
        sp1.setStiffness(s2);
	auto& sp3=osimModel.updComponent<PathSpring>("/forceset/path_spring3");
        sp1.setStiffness(s3);

	//for (int i=1; i<=st.getSize();i++)
	//cout<<labs[i]<<":"<<st[i-1]<<endl;
        //Model osimModel(modelfile);
        osimModel.updControllerSet().remove(0);
        osimModel.initSystem();
    OpenSim::Array<std::string> actuNames;
    const auto modelPath = osimModel.getAbsolutePath();
    for (const auto& actu : osimModel.getComponentList<DelpActuator>()) {
        actuNames.append(actu.getAbsolutePathString());
    }
        updateDelpActuator(osimModel, actuNames[0],"src/delp1.txt",.011,.068, 1,16);
        updateDelpActuator(osimModel, actuNames[1],"src/delp4.txt",.011,.068, 1,18);
        updateDelpActuator(osimModel, actuNames[2],"src/delp5.txt",.011,.068, 1,20);
        updateDelpActuator(osimModel, actuNames[3],"src/delp2.txt",.011,.068,-1,16);
        updateDelpActuator(osimModel, actuNames[4],"src/delp3.txt",.011,.068,-1,18);
        updateDelpActuator(osimModel, actuNames[5],"src/delp6.txt",.011,.068,-1,20);

    const SimTK::Vector& time = solution.getTime();
    auto* controller = new PrescribedController();
    controller->setName("prescribed_controller");

    for (int i = 0; i < actuNames.size(); ++i) {
        const auto control = solution.getControl(actuNames[i]);
        auto* funL=new PiecewiseLinearFunction(time.nrow(), &time[0], &control[0]);

        const auto& actu = osimModel.getComponent<DelpActuator>(actuNames[i]);
        controller->addActuator(actu);
        controller->prescribeControlForActuator( actu.getName(), funL);
        fwddebugLog<<"time,"<<actuNames[i]<<endl;
        for (int j=0;j<time.nrow();j++)
        fwddebugLog<<time[j]<<","<<control[j]<<endl;  
    }

    
    osimModel.addController(controller);
	
         cout<<"after initsystem..."<<endl;
        State &si = osimModel.initSystem();
        const MultibodySystem& system=osimModel.updMultibodySystem();
        system.addEventReporter(new MyReporter(system,.01));
        State &osimState = osimModel.initializeState();
	//set initial state from first line of statesfile
	cout<<"apply first line of states...."<<endl;
	 for (int i=0; i<st.getSize();i++)
	{osimModel.setStateVariableValue(osimState,labs[i+1],st[i]);
         cout<<labs[i+1]<<":"<<st[i]<<endl;}
	//cout<<"get end time from control file..."<<endl;
        double tf=time(time.nrow()-1);
	cout<<"Endtime:"<<tf<<endl;

        Manager manager(osimModel);
        manager.setIntegratorMaximumStepSize(tf/300);

        manager.setIntegratorAccuracy(1.0e-4);
        si.getQ().dump("Initial q's");
        //cout<<"read model from:"<< modelfile<<endl;
        //cout<<"read traj solutionfrom:"<< solfile <<endl;
        //cout<<"read initial state:"<< statesfile <<endl;
        manager.initialize(osimState);

        osimModel.print("results/fwd_integ.osim");
        std::cout<<"Integrating from 0  to " << tf << std::endl;
        manager.integrate(tf);

        Vec3 COM_position = osimModel.getMultibodySystem().getMatterSubsystem().
                        calcSystemMassCenterLocationInGround(manager.getState());
        Vec3 COM_velocity = osimModel.getMultibodySystem().getMatterSubsystem().
                        calcSystemMassCenterVelocityInGround(manager.getState());
        double g = -osimModel.getGravity()[1];
        Vector_<SpatialVec>  forcesAtMInG;
        system.realize(manager.getState(), Stage::Acceleration);
        system.getMatterSubsystem().calcMobilizerReactionForces(manager.getState(),forcesAtMInG);
        std::cout << manager.getState().getTime() << "\t" << forcesAtMInG[0][1][1] <<
			"\t"<<COM_position[1]<<  std::endl;

        cout<<"end integration at time:"<<manager.getState().getTime()<<endl;
        auto statesTable = manager.getStatesTable();
        //STOFileAdapter::write(statesTable, "stateTable.sto");
        //osimModel.updSimbodyEngine().convertRadiansToDegrees(statesTable);
        STOFileAdapter::write(statesTable, "results/fwd_states.sto");
        cout<<"Wrote states to results/fwd_states.sto"<<endl;

        double maxHeight = (COM_velocity[1]>0?COM_position[1] +
                                pow(COM_velocity[1], 2.0)/(2.0*g):0);
        auto row0=statesTable.getNearestRow(0,true)*180/Pi;
        cout<<"initAng:"<<row0[6]<<","<<row0[8]<<","<<row0[10]<<","<<row0[12]<<endl;


        cout<<"fwd check jump:"<<maxHeight<<endl;


return maxHeight;


}
#endif
void timSeriesToBinFile(TimeSeriesTable controlTable ,std::string filename)
{    std::vector<double> tim=controlTable.getIndependentColumn();
    int numrows=controlTable.getNumRows();
    int numcols=controlTable.getNumColumns();
    auto labels=controlTable.getColumnLabels();
    auto act=controlTable.getDependentColumnAtIndex(0);
    std::ofstream contfile;
    contfile.open(filename,std::ofstream::binary);
    contfile.write((char*)&numrows,sizeof(int));
    contfile.write((char*)&numcols,sizeof(int));
    for (int i=0;i<numrows;i++) contfile.write((char*)&tim[i],sizeof(double));
    for (int i=0;i<numcols;i++)
        {auto act=controlTable.getDependentColumnAtIndex(i);
        unsigned int size=labels[i].size();
        contfile.write((char*)&size,sizeof(size));
        contfile.write(labels[i].c_str(),labels[i].length());
        for (int j=0;j<numrows;j++) contfile.write((char*)&act[j],sizeof(double));}
    contfile.close();
}
struct myTrajectory
{int numrows;
int numcols;
Vector time;
vector<Vector> actu;
vector<string> labels;
Vector getControl(string actname) {
for (int i=0;i<numcols;i++)
if (!actname.compare(labels[i])) return actu[i];
cout<<"error: No such label in this traj"<<endl;
cout<<"Allowd values:\n";
for (int i=0;i<numcols;i++)cout<<labels[i]<<endl;
exit(1);
}
Vector getFirstRow(){
Vector firstrow(numcols);
for (int i=0;i<numcols;i++)firstrow(i)=actu[i](0);
return firstrow;
}
};
myTrajectory binToTraj(std::string filename)
{
    myTrajectory res;
    int numrows; int numcols;
    ifstream infile(filename,ifstream::binary);
    infile.read((char*)&numrows,sizeof(int));
    infile.read((char*)&numcols,sizeof(int));
    // std::vector<double> tim(numrows,0.);
    Vector tim(numrows,0.);
    Vector act(numrows,0.);
    vector<string> labels(numcols);
    for (int i=0;i<numrows;i++) infile.read((char*)&tim[i],sizeof(double));
    for (int i=0;i<numcols;i++)
        {unsigned int length;
        infile.read((char*)&length,sizeof(length));
        string actname;actname.resize(length);
        infile.read((char*)&actname[0],length);
        labels[i]=actname;
        for (int j=0;j<numrows;j++) infile.read((char*)&act[j],sizeof(double));
        res.actu.push_back(act);}
    infile.close();
        res.labels=labels;res.time=tim;res.numrows=numrows;res.numcols=numcols;
    return res;

}

#endif
