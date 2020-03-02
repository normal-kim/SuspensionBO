function [Yout, Yout_LQR] = ForLoop_Quarter(count,road,Uin,Yin,Kx,sat_on,LQR_on,sat_level)
%%
%%%%%%%%%%%%%%% inputs %%%%%%%%%%%%%%%%%%%
%%% count : the time steps
%%% road : road condition in that step
%%% Uin : Control inputs (or action) from deep RL in DAMPING FORCE
%%% Kx : to give LQR gain ( it is more efficient to calculate only when
%%% the episode begins)
%%% Yin : states [m1_vel m1_pos m2_vel m2_pos]^T of former steps
%%%%%%%%%%%%%%% sat_on %%%%%%%%%%%%%%%%%%%%
%%% 1 : saturation filter on
%%% 0 : saturation filter off
%%%%%%%%%%%%%%% LQR_on %%%%%%%%%%%%%%%%%%%%
%%% 1 : LQR calculation on (if expert-augment is needed)
%%% 0 : LQR calculation off (if expert-augment is not needed to save
%%% calculation time)
%%%%%%%%%%%%%%% outputs %%%%%%%%%%%%%%%%%%%%%%
%%% Yout : [m1_acc m1_vel m1_pos m2_acc m2_vel m2_pos]^T by deep RL
%%% Yout_LQR : [m1_acc m1_vel m1_pos m2_acc m2_vel m2_pos]^T by LQR
%%
c = 50; %nominal damping
m1 = 315;
m2 = 37.5;
k1 = 29.5*10^3;
k2 = 210*10^3;
Cmax = 10000; % originally 4000
Cmin = 0.1; % originally 300
% sat_level = 1500; %originally 500, needs to be identified
m1_vel = Yin(1); m1_pos = Yin(2); m2_vel = Yin(3); m2_pos = Yin(4); %%%%% Yin : 4 elements
%%
vel = m1_vel - m2_vel;
if (sat_on == 1)
    if(vel>0)
        u1 = min(Uin,vel*Cmax);
        u2 = max(u1,vel*Cmin);
    else
        u1 = max(Uin,vel*Cmax);
        u2 = min(u1,vel*Cmin);
    end
    Uin = min(abs(u2),sat_level)*sign(u2);
    Uin_LQR = -Kx*[m1_vel,m1_pos,m2_vel,m2_pos]';
    if(vel>0)
        u1 = min(Uin_LQR,vel*Cmax);
        u2 = max(u1,vel*Cmin);
    else
        u1 = max(Uin_LQR,vel*Cmax);
        u2 = min(u1,vel*Cmin);
    end
    Uin_LQR = min(abs(u2),sat_level)*sign(u2);
end
%%
initVal = [m1_vel,m1_pos,m2_vel,m2_pos];
[t,Y] = ode45(@(t,Y) suspension_active(t,Y,Uin,road,c),[0.01*(count-1),0.01*count],initVal);
Yout(2) = Y(end,1);
Yout(3) = Y(end,2);
Yout(5) = Y(end,3);
Yout(6) = Y(end,4);
Yout(1) = -c/m1*Yout(2) -k1/m1*Yout(3) + c/m1*Yout(5) + k1/m1*Yout(6) - Uin/m1;
Yout(4) = c/m2*Yout(2) + k1/m2*Yout(3) - c/m2*Yout(5) - (k1+k2)/m2*Yout(6) + k2/m2*road + 1/m2*Uin;
assignin('base','Yout',Yout);
if (LQR_on == 1)
    [t,Y_LQR] = ode45(@(t,Y) suspension_active(t,Y,Uin_LQR,road,c),[0.01*(count-1),0.01*count],initVal);
    Yout_LQR(2) = Y_LQR(end,1);
    Yout_LQR(3) = Y_LQR(end,2);
    Yout_LQR(5) = Y_LQR(end,3);
    Yout_LQR(6) = Y_LQR(end,4);
    Yout_LQR(1) = -c/m1*Yout_LQR(2) -k1/m1*Yout_LQR(3) + c/m1*Yout_LQR(5) + k1/m1*Yout_LQR(6) - Uin_LQR/m1;
    Yout_LQR(4) = c/m2*Yout_LQR(2) + k1/m2*Yout_LQR(3) - c/m2*Yout_LQR(5) - (k1+k2)/m2*Yout_LQR(6) + k2/m2*road + 1/m2*Uin_LQR;
elseif (LQR_on == 0)
    Yout_LQR = zeros(size(Yout));
end
assignin('base','Yout_LQR',Yout_LQR);
end
