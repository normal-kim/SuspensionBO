function y = matrixGenerator()
m1 = 315;
m2 = 37.5;
k1 = 29.5*10^3; % spring above
k2 = 210*10^3;% you can fix this b1
c = 50;
T = 0.01;
A = [-c/m1 -k1/m1 c/m1 k1/m1;
1    0      0    0 ;
c/m2  k1/m2  -c/m2 -(k1+k2)/m2;
0     0     1    0 ;];
B = [-1/m1 0 1/m2 0]';
Bw = [0 0 k2/m2 0]';
C = eye(4); D = zeros(4,1);
sys = ss(A,[B,Bw],C,[D,D]);
sysD = c2d(sys,T);
sys_A = sysD.A; sys_B = sysD.B;
assignin('base','sys_A',sys_A);
assignin('base','sys_B',sys_B);
y = 1;