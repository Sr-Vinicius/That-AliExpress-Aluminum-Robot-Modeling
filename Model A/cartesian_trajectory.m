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

E(1).offset = pi/6;
E(2).offset = pi/6;
E(4).offset = -pi/2;

E(1).qlim = [0 120*pi/180];
E(2).qlim = [0 120*pi/180];
E(3).qlim = [0 120*pi/180];
E(4).qlim = [0 120*pi/180];
E(5).qlim = [0 120*pi/180];

robot = SerialLink(E, 'name', 'robo 5dof');

T1 = robot.fkine([pi/2 pi/3 pi/3 2*pi/3 0]);
T2 = robot.fkine([0 7*pi/18 pi/4 2*pi/3 0]);

t = 0: 0.03: 3; 
Ts = ctraj(T1,T2,length(t));

for i=1: 1: length(t)
    Q(i,:) = ikine5s_specific(Ts(i));
end

L = {'r', 'LineWidth', 1.5};

figure(1)
robot.plot(Q, 'trail', L);