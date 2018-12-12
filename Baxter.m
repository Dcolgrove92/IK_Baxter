%Dallin and Bryan
%ME 537 Robotics Project
%Version 2
addpath C:\Users\stmab\Documents\MATLAB\MATLAB_Toolbox/RoboticToolbox/rvctools;
startup_rvc;
% clear
% clc

q = [0 0 0 0 0 0 0]; % zero angles, L shaped pose
%zero in the 45 x and -y
L1 = Link([0 .27035 .069 -pi/2]);
L2 = Link([0 0  0 pi/2]);
L3 = Link([0 .36435 .069 -pi/2]);
L4 = Link([0 0 0 pi/2]);
L5 = Link([0 .37429 .010 -pi/2]);
L6 = Link([0 0 0 pi/2]);
L7 = Link([0 .412925 0 0]); %.229525




robot = SerialLink(L1 + L2 + L3 + L4 + L5 + L6 + L7);
robot.offset= [-pi/4,pi/2, 0 0 0 0 0];
%% defining the robot now
%robot = SerialLink(L, 'name', 'HW 2', ...
%    'manufacturer', 'Killpack Inc.');

% some useful poses

% setting up the joint limits

robot.qlim = [-1.7016, 1.7016; -2.147, 1.047; -3.0541, 3.541; -.05, 2.618; -3.059, 3.059; -1.5707, 2.094;...
    -3.059, 3.059];

robot.plot(q)


