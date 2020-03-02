function y = Road_Generator_Fullcar_symmetric(index,index2)
L  = 300;  % Length of Road Profile (m)
B  = 0.1 ; % Sampling Interval (m)
dn = 1/L;  % Frequency Band (1/m)
n0 = 0.1; % Spatial Frequency (cycles/m)
%%%%%%%%%%%%% index : road type %%%%%%%%%%%%%%%%%%%%
%%%  1 : ISO 8608
%%%  2 : random sine
%%%  3 : sine with amplitude of ISO 8608 in frequency domain 
%%%%%%%%%%%%% function output %%%%%%%%%%%%%%%%%%%%%%
%%%  t : timeseries with 100Hz, 20 seconds
%%%  Road_z : road condition in z-direction of corresponding 't'
%%%  Kx1 : LQR Gain of Quarter car (basic version)
%%%  y : y = 1 ; dummy value to prevent possible errors
%%%  road_index : 1: bumper, 2: smooth_road, 3: rough_road
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
Road_index = ones(2000,1);
if (index == 1)
    if index2 == 0
        Choose_Bump = randi([1,4],1,1);
    if Choose_Bump == 1 % Speed Bump
        width = 1800+(3600-1800)*rand(1,1);
        hight = 50+(100-50)*rand(1,1);
        [x_Bump, z_Bump, v_Bump, t_Bump] = Fcn_Bump_Road(25,25,width,hight,0.01);

    elseif Choose_Bump == 2 % Impact Bar
        width = 80+(400-80)*rand(1,1);
        hight = 10+(60-10)*rand(1,1);
        [x_Bump, z_Bump, v_Bump, t_Bump] = Fcn_Bump_Road(25,25,width,hight,0.01);

    elseif Choose_Bump == 3 % Impact Bar Square
        width = 80+(400-80)*rand(1,1);
        hight = 5+(20-5)*rand(1,1);
        [x_Bump, z_Bump, v_Bump, t_Bump] = Fcn_Square_Road(25,25,width,hight,0.01);

    elseif Choose_Bump == 4 % Pot hole
        width = 800+(1500-800)*rand(1,1);
        hight = -20+(-5--20)*rand(1,1);
        [x_Bump, z_Bump, v_Bump, t_Bump] = Fcn_Square_Road(25,25,width,hight,0.01);
    end

    Road_Length = randi([200,300],1,1);
    [x_Rough, z_Rough_LH, z_Rough_RH, v_Rough, t_Rough] = Fcn_General_Road(80,80,3,Road_Length,B,dn,n0);

    Road_Length = randi([200,300],1,1);
    [x_Smooth, z_Smooth_LH, z_Smooth_RH, v_Smooth, t_Smooth] = Fcn_General_Road(110,110,1.5,Road_Length,B,dn,n0);


    % Road �ռ�
    Shuffle_Road = randi([1,6],1,1);
    if Shuffle_Road == 1
        ROAD_x = [x_Bump;
            x_Bump(end)+B+x_Rough;
            x_Bump(end)+x_Rough(end)+2*B+x_Smooth];

        ROAD_z_LH = [z_Bump; z_Rough_LH; z_Smooth_LH];
        ROAD_z_RH = [z_Bump; z_Rough_LH; z_Smooth_LH];
        road_index = [3 * ones(length(z_Bump),1); 1*ones(length(z_Rough_LH),1); 2*ones(length(z_Smooth_LH),1)];
        Velocity = [v_Bump; v_Rough; v_Smooth];
        Time = [t_Bump;
                t_Bump(end)+0.001+t_Rough;
                t_Bump(end)+t_Rough(end)+0.001*2+t_Smooth];

    elseif Shuffle_Road == 2
        ROAD_x = [x_Bump;
            x_Bump(end)+B+x_Smooth;
            x_Bump(end)+x_Smooth(end)+2*B+x_Rough];

        ROAD_z_LH = [z_Bump; z_Smooth_LH; z_Rough_LH];
        ROAD_z_RH = [z_Bump; z_Smooth_LH; z_Rough_LH];
        road_index = [3 * ones(length(z_Bump),1); 2*ones(length(z_Smooth_LH),1); 1*ones(length(z_Rough_LH),1)];
        Velocity = [v_Bump; v_Smooth; v_Rough];
        Time = [t_Bump;
                t_Bump(end)+0.001+t_Smooth;
                t_Bump(end)+t_Smooth(end)+0.001*2+t_Rough];

    elseif Shuffle_Road == 3
        ROAD_x = [x_Smooth;
            x_Smooth(end)+B+x_Bump;
            x_Smooth(end)+x_Bump(end)+2*B+x_Rough];

        ROAD_z_LH = [z_Smooth_LH; z_Bump; z_Rough_LH];
        ROAD_z_RH = [z_Smooth_LH; z_Bump; z_Rough_LH];
        road_index = [2 * ones(length(z_Smooth_LH),1); 3*ones(length(z_Bump),1); 1*ones(length(z_Rough_LH),1)];
        Velocity = [v_Smooth; v_Bump; v_Rough];
        Time = [t_Smooth;
                t_Smooth(end)+0.001+t_Bump;
                t_Smooth(end)+t_Bump(end)+0.001*2+t_Rough];

    elseif Shuffle_Road == 4
        ROAD_x = [x_Rough;
            x_Rough(end)+B+x_Bump;
            x_Rough(end)+x_Bump(end)+2*B+x_Smooth];

        ROAD_z_LH = [z_Rough_LH; z_Bump; z_Smooth_LH];
        ROAD_z_RH = [z_Rough_LH; z_Bump; z_Smooth_LH];
        road_index = [1 * ones(length(z_Rough_LH),1); 3*ones(length(z_Bump),1); 2*ones(length(z_Smooth_LH),1)];
        Velocity = [v_Rough; v_Bump; v_Smooth];
        Time = [t_Rough;
                t_Rough(end)+0.001+t_Bump;
                t_Rough(end)+t_Bump(end)+0.001*2+t_Smooth];

    elseif Shuffle_Road == 5
        ROAD_x = [x_Smooth;
            x_Smooth(end)+B+x_Rough;
            x_Smooth(end)+x_Rough(end)+2*B+x_Bump];

        ROAD_z_LH = [z_Smooth_LH; z_Rough_LH; z_Bump];
        ROAD_z_RH = [z_Smooth_LH; z_Rough_LH; z_Bump];
        road_index = [2 * ones(length(z_Smooth_LH),1); 1*ones(length(z_Rough_LH),1); 3*ones(length(z_Bump),1)];
        Velocity = [v_Smooth; v_Rough; v_Bump];
        Time = [t_Smooth;
                t_Smooth(end)+0.001+t_Rough;
                t_Smooth(end)+t_Rough(end)+0.001*2+t_Bump];

    elseif Shuffle_Road == 6
        ROAD_x = [x_Rough;
            x_Rough(end)+B+x_Smooth;
            x_Rough(end)+x_Smooth(end)+2*B+x_Bump];

        ROAD_z_LH = [z_Rough_LH; z_Smooth_LH; z_Bump];
        ROAD_z_RH = [z_Rough_LH; z_Smooth_LH; z_Bump];
        road_index = [1 * ones(length(z_Smooth_LH),1); 2*ones(length(z_Rough_LH),1); 3*ones(length(z_Bump),1)];
        Velocity = [v_Rough; v_Smooth; v_Bump];
        Time = [t_Rough; 
                t_Rough(end)+0.001+t_Smooth; 
                t_Rough(end)+t_Smooth(end)+0.001*2+t_Bump];
    end
    elseif index2 == 1 %only rough
        Road_Length = randi([600,800],1,1);
        [x_Rough, z_Rough_LH, z_Rough_LH, v_Rough, t_Rough] = Fcn_General_Road(70,90,3,Road_Length,B,dn,n0);
        ROAD_x = x_Rough;
        ROAD_z_LH = z_Rough_LH;
        ROAD_z_RH = z_Rough_LH;
        road_index = 2*ones(length(z_Rough_LH),1);
        Velocity = v_Rough;
        Time = t_Rough;
    elseif index2 == 2 %only soft
        Road_Length = randi([600,800],1,1);
        [x_Smooth, z_Smooth_LH, z_Smooth_LH, v_Smooth, t_Smooth] = Fcn_General_Road(100,120,1.5,Road_Length,B,dn,n0);
        ROAD_x = x_Smooth;
        ROAD_z_LH = z_Smooth_LH;
        ROAD_z_RH = z_Smooth_LH;
        road_index = 1*ones(length(z_Smooth_LH),1);
        Velocity = v_Smooth;
        Time = t_Smooth;
    elseif index2 == 3 %only bumper
        % Choose_Bump = randi([1,4],1,1);
        Choose_Bump = 1;
        if Choose_Bump == 1 % Speed Bump
            % width = 1800+(3600-1800)*rand(1,1);
            % hight = 50+(100-50)*rand(1,1);
            width = 2700;
            hight = 80;
            [x_Bump, z_Bump, v_Bump, t_Bump] = Fcn_Bump_Road(30,30,width,hight,0.01);

        elseif Choose_Bump == 2 % Impact Bar
            width = 80+(400-80)*rand(1,1);
            hight = 10+(60-10)*rand(1,1);
            [x_Bump, z_Bump, v_Bump, t_Bump] = Fcn_Bump_Road(30,30,width,hight,0.01);

        elseif Choose_Bump == 3 % Impact Bar Square
            width = 80+(400-80)*rand(1,1);
            hight = 5+(20-5)*rand(1,1);
            [x_Bump, z_Bump, v_Bump, t_Bump] = Fcn_Square_Road(30,30,width,hight,0.01);

        elseif Choose_Bump == 4 % Pot hole
            width = 800+(1500-800)*rand(1,1);
            hight = -20+(-5--20)*rand(1,1);
            [x_Bump, z_Bump, v_Bump, t_Bump] = Fcn_Square_Road(30,30,width,hight,0.01);
        end
        ROAD_z_LH = [z_Bump; z_Bump; z_Bump; z_Bump; z_Bump;];
        ROAD_z_RH = [z_Bump; z_Bump; z_Bump; z_Bump; z_Bump;];        
        road_index = [3 * ones(length(z_Bump),1); 3 * ones(length(z_Bump),1); 3 * ones(length(z_Bump),1); 3 * ones(length(z_Bump),1); 3 * ones(length(z_Bump),1)];
        Velocity = [v_Bump; v_Bump; v_Bump; v_Bump; v_Bump];
        Time = [t_Bump;
                t_Bump(end)+0.001+t_Bump;
                t_Bump(end)+t_Bump(end)+0.001*2+t_Bump;
                t_Bump(end)+t_Bump(end)+t_Bump(end)+0.001*3+t_Bump;
                t_Bump(end)+t_Bump(end)+t_Bump(end)+t_Bump(end)+0.001*4+t_Bump];
        ROAD_x = [x_Bump;
                  x_Bump(end)+B+x_Bump;
                  x_Bump(end)+x_Bump(end)+2*B+x_Bump;
                  x_Bump(end)+x_Bump(end)+x_Bump(end)+3*B+x_Bump;
                  x_Bump(end)+x_Bump(end)+x_Bump(end)+x_Bump(end)+4*B+x_Bump];
    end

%%
assignin('base','Time',Time);
assignin('base','Velocity',Velocity);
assignin('base','ROAD_x',ROAD_x);
assignin('base','ROAD_z_LH',ROAD_z_LH);
assignin('base','ROAD_z_RH',ROAD_z_RH);
assignin('base','road_index',road_index);
y = 1;
sim('Road_Simulink_fullcar.slx');
% t = a.time;
% Road = a.Road;
assignin('base','road_ZR',road_ZR);
assignin('base','road_ZL',road_ZL);
assignin('base','t',time);
assignin('base','Road_index',Road_index);
assignin('base','car_vel',vel);
elseif (index == 2)
 low_Frequency = rand(1);
 middle_Frequency = rand(1)*4 + 1;
 high_Frequency = rand(1)*10 + 5;
 vel = 25*ones(2000,1);
 t = 0 : 0.01 : 0.01 * (2000 - 1);
 Road_z_L = 0.005 * sin(low_Frequency*t) + 0.005 * sin(middle_Frequency*t) + 0.005 * sin(high_Frequency*t);
 Road_z_R = 0.005 * sin(low_Frequency*t) + 0.005 * sin(middle_Frequency*t) + 0.005 * sin(high_Frequency*t);
 t = reshape(t,[2000,1]);
 Road_z_L = reshape(Road_z_L,[2000,1]);
 assignin('base','t',t);
 assignin('base','road_ZL',Road_z_L);
 assignin('base','road_ZR',Road_z_L);
 assignin('base','Road_index',Road_index);
 assignin('base','car_vel',vel);
 y = 1;
elseif (index == 3)
amp_road = sqrt([6.33278440924684e-05,9.70995992576793e-05,4.79640560448303e-05,2.24867654052997e-05,1.39251747423065e-05,9.91294894325781e-06,7.48620370110958e-06,5.38517168769561e-06,3.66769781306591e-06,2.69369371292997e-06,2.02108223372533e-06,1.49473857759419e-06,1.19562314588102e-06,1.06648058005433e-06,1.00600799074230e-06,9.36776752394415e-07,8.79897732983858e-07,8.32060982870799e-07,7.41922212730803e-07,6.47786730234395e-07,5.71370007337514e-07,5.14595849261450e-07,4.82845545503211e-07,4.52664795131417e-07,4.24240649196159e-07,4.02541778748039e-07,3.62900253699966e-07,3.23202522298462e-07,2.93595229206512e-07,2.69272216228388e-07,2.50067852948105e-07,2.29754703198241e-07,2.06941007545595e-07,1.87802531406566e-07,1.77728682592581e-07,1.71130793183092e-07,1.62862998611997e-07,1.54156401650069e-07,1.40959978960720e-07,1.23068052354326e-07,1.14230086904071e-07,1.12640675574357e-07,1.08761282065116e-07,1.00902201425650e-07,9.37448426762502e-08,9.09715900688584e-08,9.09242643784871e-08,9.21137314428804e-08,8.84017915848447e-08,8.39617849812271e-08,8.00205854211787e-08,7.47929051306908e-08,7.26993143276238e-08,7.00786197723204e-08,6.49649466713072e-08,6.37968185513979e-08,6.28414150381530e-08,6.21726428631587e-08,6.07975113443325e-08,5.80399377968428e-08,5.39472270442668e-08,5.09707549071860e-08,4.95820049609014e-08,4.74757969129987e-08,4.38442710408661e-08,4.12625612097516e-08,3.99006339044716e-08,3.83516663148468e-08,3.77432722365749e-08,3.50875777369872e-08,3.19538567494831e-08,3.05151903263993e-08,2.87352649460021e-08,2.68141817682658e-08,2.51448773012700e-08,2.44762214328428e-08]);
 f = [0;0.200000000000000;0.400000000000000;0.600000000000000;0.800000000000000;1;1.20000000000000;1.40000000000000;1.60000000000000;1.80000000000000;2;2.20000000000000;2.40000000000000;2.60000000000000;2.80000000000000;3;3.20000000000000;3.40000000000000;3.60000000000000;3.80000000000000;4;4.20000000000000;4.40000000000000;4.60000000000000;4.80000000000000;5;5.20000000000000;5.40000000000000;5.60000000000000;5.80000000000000;6;6.20000000000000;6.40000000000000;6.60000000000000;6.80000000000000;7;7.20000000000000;7.40000000000000;7.60000000000000;7.80000000000000;8;8.20000000000000;8.40000000000000;8.60000000000000;8.80000000000000;9;9.20000000000000;9.40000000000000;9.60000000000000;9.80000000000000;10;10.2000000000000;10.4000000000000;10.6000000000000;10.8000000000000;11;11.2000000000000;11.4000000000000;11.6000000000000;11.8000000000000;12;12.2000000000000;12.4000000000000;12.6000000000000;12.8000000000000;13;13.2000000000000;13.4000000000000;13.6000000000000;13.8000000000000;14;14.2000000000000;14.4000000000000;14.6000000000000;14.8000000000000;15];
 t = 0 : 0.01 : 0.01 * (2000 - 1);
 t = reshape(t,[2000,1]);
 Road_z_L = zeros(length(t),1);
 Road_z_R = zeros(length(t),1);
 phi1 = 2*pi*rand(length(f));
 phi2 = 2*pi*rand(length(f));
 vel = 25*ones(2000,1);
 for k = 1: length(f)
     Road_z_L = Road_z_L + amp_road(k) * sin((k)*t + phi1(k));
     Road_z_R = Road_z_R + amp_road(k) * sin((k)*t + phi2(k));
 end
 t = reshape(t,[2000,1]);
 road_ZL = reshape(Road_z_L,[2000,1]);
 road_ZR = reshape(Road_z_R,[2000,1]);
 assignin('base','t',t);
 assignin('base','road_ZL',road_ZL);
 assignin('base','road_ZR',road_ZR);
 assignin('base','Road_index',Road_index);
 assignin('base','car_vel',vel);
 y = 1;
end
[A,B,~,~,~,~] = SSgenerator();
n_states = length(A);
n_control = 4;

C = eye(n_states); D = zeros(n_states,n_control);
sys = ss(A,B,C,D);
R = 0.01*eye(n_control);

Q11 = 40*10000*eye(6); Q12 = zeros(6,n_states-6);
Q21 = zeros(n_states-6,6); Q22 = zeros(n_states-6,n_states-6);
Q = [Q11,Q12; Q21,Q22];
Kx = lqr(sys,Q,R);

[T,D_] = bdschur(A);
assignin('base','transMat',inv(T))
assignin('base','Kx',Kx);
end