function y = Road_Generator_Fullcar_final(index)
L  = 600;  % Length of Road Profile (m)
B  = 0.1 ; % Sampling Interval (m)
dn = 1/L;  % Frequency Band (1/m)
n0 = 0.1; % Spatial Frequency (cycles/m)
%%%%%%%%%%%%% index : for bumper adding or not %%%%%%%%
%%% 1 : bumper on
%%% 0 : bumper off
%%%%%%%%%%%%% function output %%%%%%%%%%%%%%%%%%%%%%
%%%  t : timeseries with 100Hz, 20 seconds
%%%  Road_z : road condition in z-direction of corresponding 't'
%%%  Kx1 : LQR Gain of Quarter car (basic version)
%%%  y : y = 1 ; dummy value to prevent possible errors
%%%  road_index : 1: bumper, 2: smooth_road, 3: rough_road
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
y = 1;
width = 3000;
hight = 100;
[x_Bump, z_Bump, v_Bump, t_Bump] = Fcn_Bump_Road_mod(20,30,width,hight,0.01);

Road_Length = randi([600,620],1,1);
[x_Rough, z_Rough_LH, z_Rough_RH, v_Rough, t_Rough] = Fcn_General_Road(70,90,3,Road_Length,B,dn,n0);
[x_Smooth, z_Smooth_LH, z_Smooth_RH, v_Smooth, t_Smooth] = Fcn_General_Road(100,120,1.5,Road_Length,B,dn,n0);

z_difference_rough_LH = zeros(length(x_Rough),1);
z_difference_smooth_LH = zeros(length(x_Rough),1);

z_difference_rough_RH = zeros(length(x_Rough),1);
z_difference_smooth_RH = zeros(length(x_Rough),1);

x_difference_rough = zeros(length(x_Rough),1);
x_difference_smooth = zeros(length(x_Rough),1);

t_difference_rough = zeros(length(x_Rough),1);
t_difference_smooth = zeros(length(x_Rough),1);

for i = 1 : length(x_Rough) - 1
    z_difference_rough_LH(i) = z_Rough_LH(i+1) - z_Rough_LH(i);
    z_difference_smooth_LH(i) = z_Smooth_LH(i+1) - z_Smooth_LH(i);
    
    z_difference_rough_RH(i) = z_Rough_RH(i+1) - z_Rough_RH(i);
    z_difference_smooth_RH(i) = z_Smooth_RH(i+1) - z_Smooth_RH(i);
    
    x_difference_rough(i) = x_Rough(i+1) - x_Rough(i);
    x_difference_smooth(i) = x_Smooth(i+1) - x_Smooth(i);

    t_difference_rough(i) = t_Rough(i+1) - t_Rough(i);
    t_difference_smooth(i) = t_Smooth(i+1) - t_Smooth(i);
end
enum = randperm(20);
ROAD_x = zeros(length(x_Rough),1);
ROAD_z_LH = zeros(length(x_Rough),1);
ROAD_z_RH = zeros(length(x_Rough),1);
road_index = zeros(length(x_Rough),1);
Velocity = zeros(length(x_Rough),1);
Time = zeros(length(x_Rough),1);

N = floor(length(z_Rough_LH)/20);
dummy_z_RH = zeros(N,1); dummy_z_LH = zeros(N,1); 
dummy_time = zeros(N,1); dummy_x = zeros(N,1);

cnt = 1;

for k = 1 : length(enum)
    s = (cnt - 1)*N;
    if mod(enum(k),2) == 0 % smooth
        for i = 1 : N
            ROAD_z_RH(s+i+1) = ROAD_z_RH(s+i) + z_difference_smooth_RH(s+i);
            ROAD_z_LH(s+i+1) = ROAD_z_LH(s+i) + z_difference_smooth_LH(s+i);
            ROAD_x(s+i+1) = ROAD_x(s+i) + x_difference_smooth(s+i);
            Time(s+i+1) = Time(s+i) + t_difference_smooth(s+i);
        end
        road_index(s+1:s+N) = 2*ones(N,1);
        Velocity(s+1:s+N) = v_Smooth(s+1:s+N);
    elseif mod(enum(k),2) == 1 % Rough
        for i = 1 : N
            ROAD_z_RH(s+i+1) = ROAD_z_RH(s+i) + z_difference_rough_RH(s+i);
            ROAD_z_LH(s+i+1) = ROAD_z_LH(s+i) + z_difference_rough_LH(s+i);
            ROAD_x(s+i+1) = ROAD_x(s+i) + x_difference_rough(s+i);
            Time(s+i+1) = Time(s+i) + t_difference_rough(s+i);
        end
        road_index(s+1:s+N) = 1*ones(N,1);
        Velocity(s+1:s+N) = v_Rough(s+1:s+N);
    end
    cnt = cnt + 1;
end

% n = length(Velocity);
% m = find( Time > 20 ,1);
% 
ROAD_z_LH = ROAD_z_LH(1:end-10);
ROAD_z_RH = ROAD_z_RH(1:end-10);
ROAD_x = ROAD_x(1:end-10);
road_index = road_index(1:end-10);
Velocity = Velocity(1:end-10);
Time = Time(1:end-10);
%%
% one more detreding
ROAD_z_LH = detrend(ROAD_z_LH,'linear');
ROAD_z_RH = detrend(ROAD_z_RH,'linear'); 

%% select the location where the bumper will be located
N1 = randi([2,5],1);
N2 = randi([8,11],1);
m0 = floor(length(ROAD_z_LH)/20);

% N_bump = floor(t_Bump(end)/0.01);
% Time_bump = 0 : 0.01 : 0.01*(N_bump-1);
% ROAD_z_Bump = interp1(

b = length(z_Bump);
if index == 0
    ROAD_z_LH(m0*N1 : m0*N1 + b-1) = ROAD_z_LH(m0*N1) + z_Bump;
    ROAD_z_LH(m0*N2 : m0*N2 + b-1) = ROAD_z_LH(m0*N2) + z_Bump;

    ROAD_z_RH(m0*N1 : m0*N1 + b-1) = ROAD_z_RH(m0*N1) + z_Bump;
    ROAD_z_RH(m0*N2 : m0*N2 + b-1) = ROAD_z_RH(m0*N2) + z_Bump;

    Velocity(m0*N1 : m0*N1 + b-1) = v_Bump;
    Velocity(m0*N2 : m0*N2 + b-1) = v_Bump;
    %% accumulating parts; time and x direction
    time_shift1 = Time(m0*N1 + b-1) - Time(m0*N1) - t_Bump(end);

    Time(m0*N1 : m0*N1 + b-1) = Time(m0*N1) + t_Bump;
    Time(m0*N1 + b : end) = Time(m0*N1 + b : end) - time_shift1;

    time_shift2 = Time(m0*N2 + b-1) - Time(m0*N2) - t_Bump(end);
    Time(m0*N2 : m0*N2 + b-1) = Time(m0*N2) + t_Bump;
    Time(m0*N2 + b : end) = Time(m0*N2 + b : end) - time_shift2;


    road_shift1 = ROAD_x(m0*N1 + b-1) - ROAD_x(m0*N1) - x_Bump(end);

    ROAD_x(m0*N1 : m0*N1 + b-1) = ROAD_x(m0*N1) + x_Bump;
    ROAD_x(m0*N1 + b : end) = ROAD_x(m0*N1 + b : end) - road_shift1;

    road_shift2 = ROAD_x(m0*N2 + b-1) - ROAD_x(m0*N2) - x_Bump(end);
    ROAD_x(m0*N2 : m0*N2 + b-1) = ROAD_x(m0*N2) + x_Bump;
    ROAD_x(m0*N2 + b : end) = ROAD_x(m0*N2 + b : end) - road_shift2;
end

%%
if mod(length(ROAD_x),2) == 1
    ROAD_x = ROAD_x(1:end-1);
    ROAD_z_LH = ROAD_z_LH(1:end-1);
    ROAD_z_RH = ROAD_z_RH(1:end-1);
end
%%
% ROAD_z_LH(m0*N1 : m0*N1 + b-1) = ROAD_z_LH(m0*N1 : m0*N1 + b-1) + z_Bump;
% ROAD_z_LH(m0*N2 : m0*N2 + b-1) = ROAD_z_LH(m0*N2 : m0*N2 + b-1) + z_Bump;
% 
% ROAD_z_RH(m0*N1 : m0*N1 + b-1) = ROAD_z_RH(m0*N1 : m0*N1 + b-1) + z_Bump;
% ROAD_z_RH(m0*N2 : m0*N2 + b-1) = ROAD_z_RH(m0*N2 : m0*N2 + b-1) + z_Bump;

time = 0 : 0.01 : 19.99;

x_from_vel = zeros(length(Time),1);

Velocity_mps = Velocity/3.6;

for i = 1 : length(Time) - 2
   x_from_vel(i+1) = x_from_vel(i) + (Velocity_mps(i)) * (Time(i+1) - Time(i));
end

road_x = interp1(Time,x_from_vel,time);


road_ZR = interp1(ROAD_x,ROAD_z_RH,road_x);
road_ZL = interp1(ROAD_x,ROAD_z_LH,road_x);
vel = interp1(ROAD_x,Velocity,road_x);
Road_index = interp1(ROAD_x,road_index,road_x);


%%
% assignin('base','Time',Time);
% assignin('base','Velocity',Velocity);
% assignin('base','ROAD_x',ROAD_x);
% assignin('base','ROAD_z_LH',ROAD_z_LH);
% assignin('base','ROAD_z_RH',ROAD_z_RH);
% assignin('base','road_index',road_index);
% y = 1;
% sim('Road_Simulink_fullcar.slx');
% t = a.time;
% Road = a.Road;
assignin('base','road_ZR',road_ZR);
assignin('base','road_ZL',road_ZL);
assignin('base','t',time);
assignin('base','Road_index',Road_index);
assignin('base','car_vel',vel);
end