WB = 2650*0.001;
TW = 1568*0.001;

Body_CG_to_FL = [-WB/2 -TW/2 0.2];
Body_CG_to_FR = [-WB/2 TW/2 0.2];
Body_CG_to_RL = [WB/2 -TW/2 0.2];
Body_CG_to_RR = [WB/2 TW/2 0.2];

World_to_Whl_FL = [-WB/2 -TW/2 0];
World_to_Whl_FR = [-WB/2 TW/2 0];
World_to_Whl_RL = [WB/2 -TW/2 0];
World_to_Whl_RR = [WB/2 TW/2 0];

%%
tire_k_Frt = 203464.1;
tire_c_Frt = 50;
tire_k_Rr = 203464.1;
tire_c_Rr = 50;

%%
Whl_m_Frt = 101.488/2;
Whl_m_Rr = 86.718/2;

%%
SABS_c_Frt = 50; % N/(m/s) %originally 1000
SABS_c_Rr = 50;

C_max = 4000;
C_min = 300;
sat_limit = 1000;

%%
Body_m = 1106.284;
Body_Ixx = 438.7;
Body_Iyy = 1447.5;
Body_Izz = 1671.2;
Body_Ixz = -22.0;