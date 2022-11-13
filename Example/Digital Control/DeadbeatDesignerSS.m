%This script performs deadbeat design in state space, given both prediction
%and current estimator controllers
%Build the plant model
s = tf('s')
T = .1;  %Can easily change to see effects on time / control
P = 1/s/(s+1);
[A,B,C,D] = ssdata(c2d(P,T))
%Now we can simply place the poles
K = acker(A,B,[0;0])
Lp = acker(A',C',[0;0])'
Lc = inv(A)*Lp
%Now build the controllers
Cp = ss(A-B*K-Lp*C,Lp,-K,0,T)
Cc = ss(A-B*K-Lc*C*A+Lc*C*B*K,Lc,-K,0,T)*tf('z',T)
%Set initial states
x0 = [1;1];