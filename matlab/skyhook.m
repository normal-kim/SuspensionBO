function SH_Uin = skyhook(m1_acc,m1_vel,m2_vel)
%%%%%%%%%%%%%% inputs %%%%%%%%%%%%%%%
%%% m1_acc : acc of m1 of former steps
%%% m1_vel : vel of m1 of former steps
%%% m2_vel : vel of m2 of former steps
%%%%%%%%%%%%%% outputs %%%%%%%%%%%%%%
%%% SH_Uin : control inputs in terms of DAMPING FORCE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cmin = 300;
Cmax = 4000;
alpha = 0.5;
vel = m1_vel - m2_vel;
sat_limit = 500;
if (vel == 0 )
        vel =.00000001;
end
if(m1_acc*vel>=0)
    C = (alpha*Cmax*vel + (1-alpha)*Cmax*m1_acc)/vel;
else
    C = Cmin;
end
C = min(abs(C),Cmax)*sign(C);

Uin = C * vel;
SH_Uin = min(abs(Uin),sat_limit)*sign(Uin);
assignin('base','SH_Uin',SH_Uin);

end
