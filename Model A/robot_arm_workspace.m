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
E(2).offset = pi/2;
E(4).offset = -pi/2;

E(1).qlim = [0 120*pi/180];
%E(2).qlim = [0 120*pi/180]; %[without offset on robot's shoulder]
E(2).qlim = [-60*pi/180 60*pi/180]; %[with offset on robot's shoulder]
E(3).qlim = [0 120*pi/180];
E(4).qlim = [0 120*pi/180];
E(5).qlim = [0 120*pi/180];


robot = SerialLink(E, 'name', 'robo 5dof');

i = 0;
conv = pi/180; % cte para converter rad para graus

for q1=0 : 2*conv : 120*conv           % limites da junta 1 = 0º à 185º
    for q2=-60*conv : 20*conv : 60*conv % limites da junta 2
        for q3=0: 20*conv: 120*conv      % ... da junta 3
            for q4=0*conv : 20*conv : 120*conv % ... da junta 4
                for q5=0 : 20*conv : 120*conv   % ... da junta 5
                    T01 = trotz(q1+210*pi/180)*transl(0,0,L1)*trotx(pi/2);% transformação homogenea
                                                                   % entre as juntas
                                                                   % 0 e 1 pela
                                                                   % notação de DH
                    %T12 = trotz(q2)*transl(L2,0,0); %[without offset]
                    T12 = trotz(q2+pi/2)*transl(L2,0,0); %[with offset]
                    T23 = trotz(q3)*transl(L3,0,0);
                    T34 = trotz(q4-pi/2)*transl(L4,0,0)*trotx(-pi/2);
                    T45 = trotz(q5)*transl(0,0,L5);
                    T05 = T01*T12*T23*T34*T45;
                    i = i+1;
                    p = T05(1:3,4); % pega apenas a translação da transformação
                    p1(i) = p(1);   % transl em x
                    p2(i) = p(2);   % ... em y
                    p3(i) = p(3);   % ... em z
                end
            end
        end
    end
end

figure(1)
scatter1 = scatter3(p1,p2,p3,'r','filled');
scatter1.MarkerFaceAlpha = .2;
scatter1.SizeData = 10;
hold on
robot.teach()