clear all; close all;
clc;

L1 = 82;
L2 = 104;
L3 = 98;
L4 = 28;
L5 = 50;

E(1) = Link([0 L1 0  pi/2], 'standard');
E(2) = Link([0 0  L2 0],    'standard');
E(3) = Link([0 0  L3 0],    'standard');
E(4) = Link([0 0  L4  -pi/2],'standard');
E(5) = Link([0 L5 0  0],    'standard');

E(1).offset = 210*pi/180;
E(4).offset = -pi/2;

E(1).qlim = [0 120*pi/180];
E(2).qlim = [0 120*pi/180];
E(3).qlim = [0 120*pi/180];
E(4).qlim = [0 120*pi/180];
E(5).qlim = [0 120*pi/180];


robot = SerialLink(E, 'name', 'robo 5dof');

figure(1)
robot.teach('callback', @(robot, q) get_teach_config(robot));