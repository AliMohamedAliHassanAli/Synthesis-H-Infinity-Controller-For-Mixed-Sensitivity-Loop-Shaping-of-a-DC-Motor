%% Introduction
%This Code is Consdiering: 
% buidlong of the Extnded Plant Frame Work to understand the accurate
% dynamics between the load disurbance to the output 
close all
clear all
clc
s = tf("s");
%% Building the Augmeneted Plant 
% Motor Parameters
Ra = 4.36;            % Armature Resitance in Ohm
La = 134*10^-6;       % Armature inductance Henry 
Jm = 9.5*10^-10;     % Rotor INteria Kg*m^2 
Bm = 1.02*10^-9;     % Rotor Viscous Firction N*m/rpm
km = 0.00302;        % N*m/A
ke = 0.000378;       % V/rpm
Ts = 0.00011;        % N*m Static Friction Torque
ST = 0.00732;        % N*m Stall Torque
Tl = 0.00273;        % N*m Load Torque
D = tf(Tl);
Vm = 12;              % V Nominal Voltage
Tl1 = Tl               % Disturbances 
D1 = tf(Tl1); 
A = tf([km],[La Ra]);   % Armature Transfer Function
M = tf([1],[Jm Bm]);    % Motor Transfer Function 
KE = tf(ke);            % Ke Transfer Function
% AUGMENTED PLANT 
%INPUTS              OUTPUTS
% w(1) = r          z(1) = WS
% w(2) = Tl         z(2) = WU
% w(3) = n          z(3) = WT
% w(4) = u          z(4) = Measurment Error
A.u = 'v1';          % Armature input
A.y = 'Tm';          % Armature Output
D.u = 'w(2)';        % Distrubance Input
D.y = 'Tl';          % Distrubance Output
% Torque Input with Disturbances
sum1 = sumblk('T = Tm - Tl');   % T = Tm - Tl
M.u = 'T';   % Rotor Input
M.y = 'y';   % Motor Output
KE.u = 'y';  % Ke
KE.y = 'v2';
sum2 = sumblk('v1 = w(4) - v2');     % v1 = Vm - v2
sum3 = sumblk('ym = y + w(3)');    % ym = y + n
sum4 = sumblk('z(4) = w(1) - ym'); % em = r - ym
%% H infnity Mixed Senvititiy sythnesis 
%Senstivity Weight
A1 = 1/100;
wBS1 = 10000;
M1 = 0.5;
W1=((s/M1+wBS1)/(s+wBS1*A1))^2;
%complementary sensitivity function
A2 = 1/1000;
wBS2 = 150000;
M2 = 0.5;
W2=((s/M2+wBS1)/(s+wBS2*A2))^2;
%Control sensitivty 
W3=tf(0.01);
% Including Weights in the Interconnected Model
W1.u = 'z(4)';
W1.y = 'z(1)';
W2.u = 'w(4)';
W2.y = 'z(2)';
W3.u = 'y';
W3.y = 'z(3)'; 
P1 = connect(A,M,KE,D,W1,W2,W3,sum1,sum2,sum3,sum4,'w','z');  % Augmented plant with no control loop
size(P1)
% H-infinity Controller (1 measurement, 1 control input)
[K,CL,gamma] = hinfsyn(P1,1,1);
%% Closing the Loop between the Augmented plant with the Controller 
%INPUTS              OUTPUTS
% w(1) = r          z(1) = WS
% w(2) = Tl         z(2) = WU
% w(3) = n          z(3) = WT          
%                   z(4) = theta dot
%                   z(5) = control
% Fake Blocks
I = tf(1);
I1 = tf(1);
% Controller Block
K.u = 'em';
K.y = 'Vm';
%( Redefining Signals since  we must consider exogenous control and error output as internal signals)
sum2 = sumblk('v1 = Vm - v2');     % v1 = w(4) - v2
sum4 = sumblk('em = w(1) - ym');   % em = r - ym
W1.u = 'em';
W2.u = 'Vm';
% Adding Thetadot and Control Input as Output (just to see them)
I.u = 'y';
I.y = 'z(4)';
I1.u = 'Vm';
I1.y = 'z(5)';
% Creation of Closed Loop Augmented Plant
P2 = connect(A,M,KE,D,W1,W2,W3,K,I,I1,sum1,sum2,sum3,sum4,'w','z'); % Augmented plant with control loop
size(P2)
%% PLOTS
% Noise TimeSpace
tn = linspace(0,1,10000);
% Noise Signal
n = sin(100*tn);
% Reference Signal
opt_ref = stepDataOptions('StepAmplitude',19000);
% Nominal Plant with Nominal Controller
x = tf(P2(4,:));  % Transfer Functions from Inputs to Thetadot
y = tf(P2(5,:));  % % Transfer Functions from Inputs to Control
figure(1)
subplot(2,1,1)
step(x(1),opt_ref)   % r-y response
subplot(2,1,2)
step(x(2))           % d-y response
