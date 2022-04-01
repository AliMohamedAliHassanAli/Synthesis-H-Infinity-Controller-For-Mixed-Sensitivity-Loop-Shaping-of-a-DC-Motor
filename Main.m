%% Introduction
%This Code is Consdiering: 
%the Velocity Control Probelm with no load closed loop dynamics tuning with PI,PID AND LQR
%the Velocity Control Probelm with load using H infinty Mixed Sensisvity synthesis 
%the Postion Control Probelm with load using H infinty Mixed Sensisvity synthesis 
close all
clear all
clc
s = tf('s')
%% DC Motor Parameters 
R = 4.36;                                       %Armature Resistance in Ohm
La = 134*10^-6;                                 %Armature Inductance in Henri
Jm = 5.4 *10^-8;                                %Rotor in kg.m^2 
Km = 3.61*10^-3;                                %Motor Troque Constant in N.m/Amp
Bm = 5.33*10^-9/(2*pi);                         %Motor viscous friction constant in N.m/rpm
Ke=0.378*10^-4;                                 %Motor Back Emf in V/rpm
%% OpenLoop Dynamics of FREE LOAD Velcoity Control Problem 
s = tf("s");
P_Armature= tf([Km],[La R])                            %Armature Transfer Function
P_Rotor = tf([1],[Jm Bm])                              %Rotor Transfer Function
P_Motor= feedback(P_Armature*P_Rotor,Ke)               %Motor Transfer Function
k = dcgain(P_Motor)
LoopFunction = tf(P_Motor)                             %OPEN loop Function
SIZE = size(LoopFunction);                             %Size of OPen loop Function 
Stability = allmargin(LoopFunction)
P = pole(LoopFunction)
Z = zero(LoopFunction)
figure(1)
rlocus(LoopFunction) 
figure(2)
bodemag(LoopFunction)
figure(3)
nyquist(LoopFunction)
grid on
figure(4)
step(LoopFunction)
ylabel('Rotation Speed in RPM')
%% Closed Loop Dynamics of FREE LOAD Disturbance Velcoity Control Problem
%PID Controller 
C_PID = pidtune(LoopFunction,'Pid')
 sys_PID = feedback(LoopFunction*C_PID,1)
%PIDF Controller 
C_PIDF = pidtune(LoopFunction,'Pidf')
 sys_PIDF = feedback(LoopFunction*C_PIDF,1)
%LQR Controller 
b=[0.00361];
a=[7.236e-12 2.354e-07 1.402e-07]
[A,B,C,D] = tf2ss(b,a)
sys_t = ss(A,B,C,D)
Q = [10 0;0 0]
R = 10
K_LQR = lqr(A,B,Q,R)
G = -inv(C*inv(A-B*K_LQR)*B)
sys_LQR = ss(A-B*K_LQR,B*G,C,D);
figure(5)
opt = stepDataOptions('InputOffset',0,'StepAmplitude',26000);
  step(sys_PID,'b',sys_PIDF,'g',sys_LQR,'r',opt)
  xlabel('Time')
  ylabel('Angular velocity (S^-1)')
 legend('PID','PIDF','LQR','Location','SouthEast')
 figure(6)
 bodemag(LoopFunction*C_PID,LoopFunction*C_PIDF,LoopFunction)
 legend('PID','PIDF','Openloop','Location','Southwest')
%% Velocity control Problem having constant Load Troque Disturbance 
s = tf("s");
P_Armature= tf([Km],[La R])                             %Armature Transfer Function
P_Armature_C1=P_Armature*1/s                            %Armature Transer Function after C1
P_Rotor = tf([1],[Jm Bm])                               %Rotor Transfer Function
P_Motor= feedback(P_Armature_C1*P_Rotor,Ke)             %Motor Transfer Function
% Using H_infinty of Weight Sensitivity Function
%Senstivity Weight
A1 = 1/1000;
wBS1 =10;
M1 = 2;
wS1=((s/M1+wBS1)/(s+wBS1*A1))^2;
[C,CL,Gamma1]= mixsyn(P_Motor, wS1,[],[]);
C_Optimal=tf(C)
Tr_Closed_Optimal=tf(CL)
figure(8)
sys_Hinfinty_Ws = feedback(C_Optimal*P_Motor,1)
opt = stepDataOptions('InputOffset',0,'StepAmplitude',26000);
step(sys_Hinfinty_Ws,'k',sys_PID,'b',sys_PIDF,'g',sys_LQR,'y',opt)
ylabel('Angular velocity (S^-1)')
legend('H_infinty_Ws','PID','PIDF','LQR','Location','SouthEast')
 figure(9)
 bodemag(P_Motor*C_Optimal,P_Motor*C_PIDF)
 legend('H_infinty_Ws','PIDF','Location','SouthEast')
%% Using Stacked Senstivity Weigths
%Senstivity Weight
A1 = 1/100;
wBS1 = 10;
M1 = 0.5;
wS1=((s/M1+wBS1)/(s+wBS1*A1))^2;
%complementary sensitivity function
A2 = 1/1000;
wBS2 = 1000;
M2 = 0.5;
wS3=((s/M2+wBS1)/(s+wBS2*A2))^2;
%Control sensitivty 
wS2=1;
[C,CL,Gamma1]= mixsyn(P_Motor, wS1,wS2,wS3);
C_Optimal_Mixed=tf(C)
Tr_Closed_Optimal_Mixed=tf(CL)
figure(10)
sys_Hinfinty_Mixed = feedback(C_Optimal_Mixed*P_Motor,1)
opt = stepDataOptions('InputOffset',0,'StepAmplitude',26000)
step(sys_Hinfinty_Mixed,'c',sys_Hinfinty_Ws,'k',sys_PID,'b',sys_PIDF,'g',sys_LQR,'y',opt)
ylabel('Angular velocity (S^-1)')
legend('H Infinty Mixed','H Infinty Ws','PID','PIDF','LQR','Location','SouthEast')
figure(11)
 bodemag(P_Motor*C_Optimal_Mixed,P_Motor*C_Optimal,P_Motor*C_PIDF)
 legend('H Infinty Mixed','H Infinty Ws','PID','PIDF','LQR','Location','SouthEast')
 %% Postion Control Problem using H infinty Mixed Senstivity 
 close all 
 clear all
 clc
%%Postion Control Probmel using H infinty Mixed Senstivity
R = 4.36;                                       %Armature Resistance in Ohm
La = 134*10^-6;                                 %Armature Inductance in Henri
Jm = 5.4 *10^-8;                                %Rotor in kg.m^2 
Km = 3.61*10^-3;                                %Motor Troque Constant in N.m/Amp
Bm = 5.33*10^-9/(2*pi);                         %Motor viscous friction constant in N.m/rpm
Ke=0.378*10^-4;                                 %Motor Back Emf in V/rpm
s = tf("s");
P_Armature= tf([Km],[La R])                               %Armature Transfer Function
P_Armature_C1=P_Armature*1/s                              %Armature Transer Function after C1
P_Rotor = tf([1],[Jm Bm])*1/s                             %Rotor Transfer Function
P_Motor= feedback(P_Armature_C1*P_Rotor,Ke)               %Motor Transfer Function
%Senstivity Weight
A1 = 1/100;
wBS1 = 10;
M1 = 2;
wS1=((s/M1+wBS1)/(s+wBS1*A1))^2;
%complementary sensitivity function
A2 = 1/1000;
wBS2 = 1000;
M2 = 2;
wS2=((s/M2+wBS1)/(s+wBS2*A2))^2;
%Control sensitivty 
wS3=1;
[C,CL,Gamma1]= mixsyn(P_Motor, wS1,wS2,wS3);
C_Optimal_Mixed=tf(C)
Tr_Closed_Optimal_Mixed=tf(CL)
figure(10)
sys_Hinfinty_Mixed = feedback(C_Optimal_Mixed*P_Motor,1)
step(sys_Hinfinty_Mixed)
ylabel('Angular Postion in Rad')
legend('H Infinty Mixed')
figure(11)
 bodemag(P_Motor*C_Optimal_Mixed)
 legend('H Infinty Mixed')
