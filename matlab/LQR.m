function [K, S, E] = LQR(A, B, a, b, c, w2)

w1 = 400000;
Q = zeros(14,14);
Q(1,1) = a;
Q(2,2) = a;

Q(3,3) = b;
Q(4,4) = b;

Q(5,5) = c;
Q(6,6) = c;

R = eye(4);

K = lqr(A,B,w1*Q,w2*R);
assignin('base','K',K);
end
