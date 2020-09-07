#include <OpenSim/OpenSim.h>
#include <Moco/Components/ActivationCoordinateActuator.h>
using namespace OpenSim;
using namespace SimTK;
using namespace std;
#ifndef GAFWD
Vector bestpars;int sigstop;
ofstream optLog("results/ga_opt.Log", ofstream::out);
ofstream bestSoFarLog("results/ga_bestSoFar.Log", ofstream::out);
ofstream verboseLog("results/ga_verbose.Log", ofstream::out);
ofstream debugLog("results/ga_debug.Log", ofstream::out);
ifstream myfile("src/ga_input.txt");
ofstream actTable("results/ga_actTable.csv");
ofstream Log1("results/ga_1Table.csv", ofstream::out);
ofstream Log2("results/ga_2Table.csv", ofstream::out);
ofstream Log3("results/ga_3Table.csv", ofstream::out);
#endif




 ActivationCoordinateActuator* addActivationCoordinateActuator(Model& model, std::string coordName,
        string actuname) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new ActivationCoordinateActuator();
//"Smaller value means activation can change more rapidly "
    actu->set_activation_time_constant(0.011);
    actu->set_dactivation_time_constant(0.011);
    actu->set_default_activation(0.);
    actu->setName(actuname);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(1);
    actu->setMinControl(-1);
    actu->setMaxControl(1);
    model.addComponent(actu);
    return actu;
}
void setInitOptF(Model& _model,State& s,Vector acts){

        double ankang=180.+_model.getCoordinateSet().get(1).getValue(s)*180./Pi;
        double kneeang=180.-_model.getCoordinateSet().get(2).getValue(s)*180./Pi;
        double hipang=180.+_model.getCoordinateSet().get(3).getValue(s)*180./Pi;
//transtting torques on 4 joints to torqus on 6 actuators
//the first act is the the torqu on tip(need to be about 0)
        Vector torqs(6,0.);
        if (acts(1)>0) torqs(0)=acts(1); else torqs(3)=acts(1);
        if (acts(2)>0) torqs(1)=acts(2); else torqs(4)=acts(2);
        if (acts(3)>0) torqs(2)=acts(3); else torqs(5)=acts(3);

        auto& sp1=_model.updComponent<ActivationCoordinateActuator>("/ap");
        ActivationCoordinateActuator* act1=dynamic_cast<ActivationCoordinateActuator*>(&sp1);
        act1->setOptimalForce(300.);
        auto& sp2=_model.updComponent<ActivationCoordinateActuator>("/kp");
        ActivationCoordinateActuator* act2=dynamic_cast<ActivationCoordinateActuator*>(&sp2);
        act2->setOptimalForce(300.);
        auto& sp3=_model.updComponent<ActivationCoordinateActuator>("/hp");
        ActivationCoordinateActuator* act3=dynamic_cast<ActivationCoordinateActuator*>(&sp3);
        act3->setOptimalForce(300.);

        auto& sp4=_model.updComponent<ActivationCoordinateActuator>("/am");
        ActivationCoordinateActuator* act4=dynamic_cast<ActivationCoordinateActuator*>(&sp4);
        act4->setOptimalForce(300.);
        auto& sp5=_model.updComponent<ActivationCoordinateActuator>("/km");
        ActivationCoordinateActuator* act5=dynamic_cast<ActivationCoordinateActuator*>(&sp5);
        act5->setOptimalForce(300.);
        auto& sp6=_model.updComponent<ActivationCoordinateActuator>("/hm");
        ActivationCoordinateActuator* act6=dynamic_cast<ActivationCoordinateActuator*>(&sp6);
        act6->setOptimalForce(300.);

//here acts are torques computed by id tool

        act1->setStateVariableValue(s,"activation",torqs(0)/300.);
        act2->setStateVariableValue(s,"activation",torqs(1)/300.);
        act3->setStateVariableValue(s,"activation",torqs(2)/300.);
        act4->setStateVariableValue(s,"activation",torqs(3)/300.);
        act5->setStateVariableValue(s,"activation",torqs(4)/300.);
        act6->setStateVariableValue(s,"activation",torqs(5)/300.);


        cout<<"set initial optimal to:"<<act1->getOptimalForce()<<","<<
		act2->getOptimalForce()<<","<<
		act3->getOptimalForce()<<","<<
		act4->getOptimalForce()<<","<<
		act5->getOptimalForce()<<","<<
		act6->getOptimalForce()<<","<<endl;
}
struct UserLine{
       Vector v;
       Vector initsta;
       double endtim;
       double f;
       double realEndtim;};

ifstream linefile("src/userline.txt");
UserLine  readuser(){
  double f,realendtim;
  int ngen,runtim,tdiv;
  linefile.precision(15);
  UserLine userline;
  linefile>>tdiv;
  Vector v(tdiv);
  linefile>>ngen>>f>>v>>realendtim>>runtim;
  userline.f=f;
  userline.endtim=v[tdiv-1];
  userline.v=v;
  userline.realEndtim=realendtim;
  cout.precision(15);
 cout<<ngen<<","<<","<<f<<endl;
  cout<<"in userline.txt:    "<<v<<endl;

  return  userline;

 }
UserLine readbin(){
  	double f,realendtim,tmpd;
  	int ngen,runtim,tdiv;
  	UserLine userline;
	Vector sta(14);
	char tmp;

	ifstream infile("src/ga_best.bin",ifstream::binary);
	ifstream sinfile("src/ga_state0.bin",ifstream::binary);
    	infile.read((char*)&tdiv,sizeof(int));
  	Vector v(tdiv);
    	infile.read((char*)&ngen,sizeof(int));
    	infile.read((char*)&f,sizeof(f));
        infile.read((char*)&realendtim,sizeof(realendtim));
        readUnformatted(infile,v);
        readUnformatted(sinfile,sta);
	userline.f=f;userline.v=v;userline.realEndtim=realendtim;
	userline.endtim=v[tdiv*3];userline.initsta=sta;
	cout<<"readbin: "<<ngen<<" "<<f<<" "<<v<<" "<<realendtim<<" "<<userline.endtim<<endl;
	cout<<"state(0):"<<tmp<<" " <<sta<<endl;
        infile.close();
        sinfile.close();

return userline;
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
cout<<"error: No such label " <<actname<<" in this traj"<<endl;
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
