function y = autoSaver(dir_name,file_name)
sat = 0;
if isempty(dir_name)
    dir_name = '';
end    
csv_name = [dir_name,file_name,'.csv'];
dataSet = table2array(readtable(csv_name));
a = 1; b = 50;
for i = a : b
    dataset(i).data = dataSet(2000 * (i-1)+1 : 2000*i,4:11);
end

c_LQR = 50;
m1 = 315;
m2 = 37.5;
k1 = 29.5*10^3;
k2 = 210*10^3;
noise_level = 0.0005;
Q = 50 * 10000;
R = 0.01;
k_filter = 0.04; % smooth road : 0.004, rough road 0.016
tau_filter = 1/12;


if sat == 1
    Cmax = 4000;
    Cmin = 300;
elseif sat == 0
    Cmax = 10000;
    Cmin = 0.1;
end
sat_level = 1500;
T = 0.01;
A_ = [-c_LQR/m1 -k1/m1 c_LQR/m1 k1/m1;
      1    0      0    0 ;
      c_LQR/m2  k1/m2  -c_LQR/m2 -(k1+k2)/m2;
      0     0     1    0 ;];
Bu_ = [-1/m1 0 1/m2 0]'; Bw_ = [0 0 k2/m2 0]';
C_ = eye(4); D_ = zeros(4,1);

nx = 5; ny = 4;
A = [A_, Bw_;
    zeros(1,4), -k_filter/tau_filter];
Bu = [Bu_; 0]; Bw = [zeros(size(Bw_)); k_filter/tau_filter];
C = [C_, zeros(4,1)]; D = [D_];
sysLQG = c2d(ss(A,Bu,C,D),0.01);

Qn = 1*zeros(5,5); Qn(5,5) = 1;
Rn = noise_level*eye(ny);
Qmat = zeros(size(A)); Qmat(1,1) = 1; Qmat(2,2) = 1;
Qmat = Qmat * Q;

QXU = blkdiag(Qmat,R);
QWV = blkdiag(Qn,Rn);

KLQG = lqg(sysLQG,QXU,QWV);

stack.LQG = []; stack.SH= []; stack.Pas = [];
%% LQG
for P = a:b
Road_z = dataset(P).data(:,8);
N = length(Road_z);
t = 0 : 0.01 : 0.01*(N-1);
Y = zeros(4,N);
Xk = zeros(5,N);
for k = 1 : N
    initVal = Y(:,k);
    if k < 2
        Uin = KLQG.C * Xk(:,k);
    else
        Xk(:,k) = KLQG.A * Xk(:,k-1) + KLQG.B * (Y(:,k) + noise_level * randn(4,1));
        Uin = KLQG.C * Xk(:,k);
    end    
    vel = Y(1,k) - Y(3,k);
    if(vel>0)
        u1 = min(Uin,vel*Cmax);
        u2 = max(u1,vel*Cmin);
    else
        u1 = max(Uin,vel*Cmax);
        u2 = min(u1,vel*Cmin);
    end
    Uin = min(abs(u2),sat_level)*sign(u2);
    road = Road_z(k);
    [time,y] = ode45(@(time,y) suspension_active(time,y,Uin,road,c_LQR),[0.01*(k-1),0.01*k],initVal);
    
    Yout(k,2) = y(end,1); Y(1,k+1) = Yout(k,2);
    Yout(k,3) = y(end,2); Y(2,k+1) = Yout(k,3);
    Yout(k,5) = y(end,3); Y(3,k+1) = Yout(k,5);
    Yout(k,6) = y(end,4); Y(4,k+1) = Yout(k,6);
    Yout(k,1) = -c_LQR/m1*Yout(k,2) -k1/m1*Yout(k,3) + c_LQR/m1*Yout(k,5) + k1/m1*Yout(k,6) - Uin/m1;
    Yout(k,4) = c_LQR/m2*Yout(k,2) + k1/m2*Yout(k,3) - c_LQR/m2*Yout(k,5) - (k1+k2)/m2*Yout(k,6) + k2/m2*road + 1/m2*Uin;
    u_LQR(k) = Uin;
end
%% discrete version passive
Y_p_c = zeros(N,4);
C = 1000;
for k = 1 : N
    initVal = Y_p_c(k,:);
    Uin = (Y_p_c(k,1)-Y_p_c(k,3))*C;
    Uin = min(abs(Uin),sat_level).*sign(Uin);
    road = Road_z(k);    
    [time,y] = ode45(@(time,y) suspension_active(time,y,Uin,road,0),[0.01*(k-1),0.01*k],initVal);    
    Yout_pc(k,2) = y(end,1); Y_p_c(k+1,1) = Yout_pc(k,2);
    Yout_pc(k,3) = y(end,2); Y_p_c(k+1,2) = Yout_pc(k,3);
    Yout_pc(k,5) = y(end,3); Y_p_c(k+1,3) = Yout_pc(k,5);
    Yout_pc(k,6) = y(end,4); Y_p_c(k+1,4) = Yout_pc(k,6);
    Yout_pc(k,1) = -0/m1*Yout_pc(k,2) -k1/m1*Yout_pc(k,3) + 0/m1*Yout_pc(k,5) + k1/m1*Yout_pc(k,6) - Uin/m1;
    Yout_pc(k,4) = 0/m2*Yout_pc(k,2) + k1/m2*Yout_pc(k,3) - 0/m2*Yout_pc(k,5) - (k1+k2)/m2*Yout_pc(k,6) + k2/m2*road + 1/m2*Uin;
end
%% Skyhook
alpha = 0.5;
Y = zeros(N,4);
for k = 1 : N
    initVal = Y(k,:);
    obs_vel = (Y(k,1) + noise_level*randn(1)) - (Y(k,3) + noise_level*randn(1));
    if (obs_vel == 0 )
        obs_vel =.000001;
    end
    if(Y(k,1)*obs_vel>=0)
        C = (alpha*4000*obs_vel + (1-alpha)*300*Y(k,1))/obs_vel;
    else
        C = Cmin;
    end
    C = min(C,4000)*sign(C);
    road = Road_z(k);
    [time,y] = ode45(@(time,y) suspension_skyhook(time,y,C,road),[0.01*(k-1),0.01*k],initVal);
    
    Y_sh(k,2) = y(end,1); Y(k+1,1) = Y_sh(k,2);
    Y_sh(k,3) = y(end,2); Y(k+1,2) = Y_sh(k,3);
    Y_sh(k,5) = y(end,3); Y(k+1,3) = Y_sh(k,5);
    Y_sh(k,6) = y(end,4); Y(k+1,4) = Y_sh(k,6);
    Y_sh(k,1) = -C/m1*Y_sh(k,2) -k1/m1*Y_sh(k,3) + C/m1*Y_sh(k,5) + k1/m1*Y_sh(k,6);
    Y_sh(k,4) = C/m2*Y_sh(k,2) + k1/m2*Y_sh(k,3) - C/m2*Y_sh(k,5) - (k1+k2)/m2*Y_sh(k,6) + k2/m2*road;
    C_k(k) = C;
end
%%
RMS_LQG_Quarter(P) = sqrt(sum(Yout(:,1).^2)/length(Yout(:,1)));
RMS_Pas_Quarter(P) = sqrt(sum(Yout_pc(:,1).^2)/length(Yout_pc));
RMS_SH_Quarter(P) = sqrt(sum(Y_sh(:,1).^2)/length(Yout(:,1)));
RMS_RL_Quarter(P) = sqrt(sum(dataset(P).data(:,1).^2)/length(dataset(P).data(:,1)));

stack.LQG = [stack.LQG; Yout(:,1);];
stack.Pas = [stack.Pas; Yout_pc(:,1);];
stack.SH = [stack.SH; Y_sh(:,1);];
disp(P);
end
RMS_avg_LQG_Quarter = sum(RMS_LQG_Quarter)/length(RMS_LQG_Quarter);
RMS_avg_Pas_Quarter = sum(RMS_Pas_Quarter)/length(RMS_Pas_Quarter);
RMS_avg_SH_Quarter = sum(RMS_SH_Quarter)/length(RMS_SH_Quarter);
RMS_avg_RL_Quarter = sum(RMS_RL_Quarter)/length(RMS_RL_Quarter);

%% RMS data save

RMS_data = [RMS_avg_LQG_Quarter,RMS_avg_Pas_Quarter,RMS_avg_SH_Quarter,RMS_avg_RL_Quarter];
header = {'LQG','passive','SH','RL'};
rms_table = array2table(RMS_data);
rms_table.Properties.VariableNames = header;
writetable(rms_table,[file_name,'_rms.csv']);

%% PSD save
road_profile = dataSet(:,11);
m1_acc_rr = dataSet(:,4);
segmentlength = 300;
noverlap = 200;
nfft = 500;
[rr_acc,~] = pwelch(m1_acc_rr,segmentlength,noverlap,nfft,100);
[p_road,f] = pwelch(road_profile,segmentlength,noverlap,nfft,100);

m1_acc_LQG = stack.LQG;
m1_acc_pas = stack.Pas;
m1_acc_SH = stack.SH;
[LQG_acc,~] = pwelch(m1_acc_LQG,segmentlength,noverlap,nfft,100);
[pas_acc,~] = pwelch(m1_acc_pas,segmentlength,noverlap,nfft,100);
[SH_acc,~] = pwelch(m1_acc_SH,segmentlength,noverlap,nfft,100);

fig_PSD = figure('visible','off');
plot(f,10*log10(pas_acc./p_road)); hold on;
plot(f,10*log10(LQG_acc./p_road));
plot(f,10*log10(SH_acc./p_road));
plot(f,10*log10(rr_acc./p_road));
legend('passive','LQR','SH','RL');
xlabel('Frequency(Hz)'); ylabel('dB'); xlim([0,50]);
psd_name = [file_name,'_psd'];
saveas(fig_PSD,psd_name,'png');
close(fig_PSD);

%% acc plot save
fig_acc = figure('visible','off');
plot(Y_sh(:,1)); hold on;
plot(Yout(:,1));
plot(dataset(P).data(:,1));
legend('SH','LQR','RL')
xlabel('steps'); ylabel('acc (m/s^2)');
acc_name = [file_name,'_acc'];
saveas(fig_acc,acc_name,'png');
close(fig_acc);
y = 1;
end