clc; clear all; close all;

%Params

ST = 0,018;%kg-m %STALL Torque
OP = 0.1; %OPerating speed /60 degrees
OV = 4.8; % Voltage
BV = 10*10^-6; % Dead band width
PC = 20*10^-3; % pulse cycle
minposf = 1/500*10^-6; %freq dla 0 degrees
maxposf = 1/2400*10^-6; %freq dla 180 degrees

%%% trasmitancja serva %%%
servo = tf(0.01,[0.005, 0.06, 0.1001, 0])
step(servo);