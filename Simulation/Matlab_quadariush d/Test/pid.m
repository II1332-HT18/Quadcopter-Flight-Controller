%/ PID /%

cler all
clc
num = 1;
denom = [1 3 1];

G2 = tf(num, denom);
H = 1;

M = feedback(G2, H);
step(M)
grid on

%%
Kp = 7;
Ki = 0.3;
Kd = 0.54;

G1 = pid(Kp, Ki, Kd);

E(s) = R(s) - G2*Y(s);
Y(s) = G1*E(s);

Mc = feedback(G1*G2, H);
step(Mc)
grid on









