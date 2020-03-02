function [x, z, v, t_end] = Fcn_Bump_Road_mod(Vel_from,Vel_to,width,hight,B)
% width: Bump width (mm)
% hight: Bump hight (mm)
% B: Sampling Interval (m)
% Vel_from : Initial Velocity (kph)
% Vel_to : Final Velocity (kph)

% mm2m
width = width/1000;
hight = hight/1000;

temp = [-width/2:B:width/2]';

% Road hight (m)
z = sqrt(1-temp.^2/(width/2)^2)*hight;

add_zeros_before = 50;

% Road distance (m)
x = [0:B:(size(z,1)-1+add_zeros_before)*B]';

% Add zero range
add_zero_after = 50;
z = [zeros(add_zero_after,1);z;zeros(add_zeros_before-add_zero_after,1);];

% Random Velocity
v = (Vel_from+(Vel_to-Vel_from)*rand(1,1))*ones(size(z));

% kph to mps
v_mps = v / 3.6;

% End Time
t_end = x/v_mps(1);

