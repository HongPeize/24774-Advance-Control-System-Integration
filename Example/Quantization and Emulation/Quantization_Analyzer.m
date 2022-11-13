%This looks at quantization effects in the Arduino implementation of a SISO
%deadbeat controller

%Quantization levels
%I need to make assumptions about the sensor ranges
%Assume:  y ranges plus/minus 2, u ranges plus/minus 5
q1 = 4/1024;  %10 bit ADC resolution
q2 = 10/256;  %8 bit DAC resolution

%Build the plant model
s = tf('s');
P = 1/s/(s+1);
Pz = c2d(P,1);
%This is the previously designed deadbeat controller
z = tf('z',1);
Cz = (1.5820*z-0.5820)/(z+0.4180);

%Worst-case analysis
T = feedback(Pz*Cz,1);
S = 1-T;
h1 = impulse(T);
h2 = impulse(Pz*S);
plot([h1,h2])
emax = sum(abs(h1))*q1/2+sum(abs(h2))*q2/2