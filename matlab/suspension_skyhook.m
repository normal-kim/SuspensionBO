function dY = suspension_skyhook(t,Y,C,road)
 w = road;
 m1 = 315;
 m2 = 37.5;
 k1 = 29.5*10^3;
 k2 = 210*10^3;
 A = [-C/m1 -k1/m1 C/m1 k1/m1;
 1    0      0    0 ;
 C/m2  k1/m2  -C/m2 -(k1+k2)/m2;
 0     0     1    0 ;];
 Bw = [0 0 k2/m2 0]';
 B = [-1/m1 0 1/m2 0]';
 dY = A*Y + Bw*w;
end