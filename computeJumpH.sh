#!/bin/bash
function some {
for fil in $1;do 
sed -i  "s|results/traj.*sto|${fil}|g" analyze2.xml
 ~/MOCOBIN/opensim-core/bin/opensim-cmd run-tool ./analyze2.xml &>/dev/null
ypos=$(tail -1 Analyzes/4linkModel_BodyKinematics_pos_global.sto |cut -f 45)
yvel=$(tail -1 Analyzes/4linkModel_BodyKinematics_vel_global.sto |cut -f 45)
read num1 num2 num3<<<${fil//[^0-9]/ }
echo -ne $num1 $num2 $num3  " "
python -c "print($ypos+$yvel*$yvel/2/9.81)"
done
echo
}
some 'results/traj*.0.0.sto'
some 'results/traj0.*.0.sto'
