function [x, z_LH, z_RH, v, t_end] = Fcn_General_Road(Vel_from,Vel_to,k1,L,B,dn,n0)
% k1: from ISO 8608, 3 is very rough road
% L: Length of Road Profile (m)
% B: Sampling Interval (m)
% dn: Frequency Band (1/m)
% n0: Spatial Frequency (cycles/m)
% Vel_from : Initial Velocity (kph)
% Vel_to : Final Velocity (kph)

% N: Number of data points
N = L/B;

% Spatial Frequency Band (1/m)
n  = [dn : dn : N*dn]';

% Abscissa Variable from 0 to L (x-Coordinate, m)
x = [0:B:L-B]';

% Amplitude for Road  Class
Amp1 = sqrt(dn)*(2^k1)*(1e-3)*(n0./n);

% Random Phase Angle
phi1 =  2*pi*rand(size(n));

% LH
% Road hight (m)
z_LH = zeros(size(x));
for i=1:length(x)
    z_LH(i) = sum(Amp1.*cos(2*pi*n*x(i)+ phi1));
end

% ���Ͼ��ϰ� �����ϴ� Ʈ���尡 ����� �����ذ�
z_LH = detrend(z_LH,'linear');

% ó���� ������ ���� 0���� �����ϵ��� ������ ����
z_LH = z_LH.*tukeywin(size(z_LH,1),0.1);

% RH
% Random Phase Angle
phi1 =  2*pi*rand(size(n));

% Road hight (m)
z_RH = zeros(size(x));
for i=1:length(x)
    z_RH(i) = sum(Amp1.*cos(2*pi*n*x(i)+ phi1));
end

% ���Ͼ��ϰ� �����ϴ� Ʈ���尡 ����� �����ذ�
z_RH = detrend(z_RH,'linear');

% ó���� ������ ���� 0���� �����ϵ��� ������ ����
z_RH = z_RH.*tukeywin(size(z_RH,1),0.1);

% Random Velocity
v = (Vel_from+(Vel_to-Vel_from)*rand(1,1))*ones(size(z_RH));

% kph to mps
v_mps = v / 3.6;

% End Time
t_end = x / v_mps(1);





