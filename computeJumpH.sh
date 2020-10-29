#!/bin/bash
function onejump {
sed -i  "s|results/traj.*sto|$1|g" analyze2.xml
 ~/MOCOBIN/opensim-core/bin/opensim-cmd run-tool ./analyze2.xml &>/dev/null
 ypos=$(tail -1 Analyzes/4linkModel_BodyKinematics_pos_global.sto |cut -f 45)
 yvel=$(tail -1 Analyzes/4linkModel_BodyKinematics_vel_global.sto |cut -f 45)
 read num1 num2 num3<<<${fil//[^0-9]/ }
 echo -ne $num1 $num2 $num3  " "
 python -c "print($ypos+$yvel*$yvel/2/9.81)"
 }
function some {
for fil in $1;do 
	onejump $fil
done
echo
}
if [ $# -eq 0 ]; then
some 'results/traj*.0.0.sto'
some 'results/traj0.*.0.sto'
some 'results/traj0.0.*.sto'
else
	fil="results/traj$1.$2.$3.sto"
	if [ -a $fil ];then
		onejump $fil
	fi
fi
