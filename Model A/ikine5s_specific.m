function [ikine5s] = ikine5s_specific(Kf)
%IKINE5S_SPECIFIC Summary of this function goes here
%   Detailed explanation goes here
    L1 = 82;
    L2 = 104;
    L3 = 98;
    L4 = 28;
    L5 = 50;
    theta1 = atan2(Kf.t(2),Kf.t(1));
    while theta1 < 0
        theta1 = theta1 + pi;
    end

    theta5 = -atan2(Kf.o(3), Kf.n(3)); %conversar com o eder sobre esse sinal negativo
    while theta5 < 0
        theta5 = theta5 + pi;
    end

    theta234 = atan2(-(Kf.a(1)*cos(theta1)+Kf.a(2)*sin(theta1)), Kf.a(3));
    
    C3 = ((Kf.t(1)*cos(theta1)+Kf.t(2)*sin(theta1)-L4*cos(theta234)+L5*sin(theta234))^2 + (Kf.t(3)-L1-L5*cos(theta234)-L4*sin(theta234))^2 - L2^2 - L3^2)/(2*L2*L3);
    S3 = sqrt(1-C3^2);
    theta3 = atan2(S3,C3);
    while theta3 < 0
        theta3 = theta3 + pi;
    end

    num2 = (Kf.t(3)-L1-L4*sin(theta234)-L5*cos(theta234))*(L3*cos(theta3)+L2) - (L3*sin(theta3))*(Kf.t(1)*cos(theta1)+Kf.t(2)*sin(theta1)-L4*cos(theta234)+L5*sin(theta234));
    den2 = (Kf.t(3)-L1-L4*sin(theta234)-L5*cos(theta234))*(L3*sin(theta3)) + (L3*cos(theta3)+L2)*(Kf.t(1)*cos(theta1)+Kf.t(2)*sin(theta1)-L4*cos(theta234)+L5*sin(theta234));
    theta2 = atan2(num2,den2);
    while theta2 < 0
        theta2 = theta2 + pi;
    end
    
    theta4 = theta234 - theta2 - theta3;
    while theta4 < 0
        theta4 = theta4 + pi;
    end

    theta1 = theta1 - pi/6;
    theta2 = theta2 - pi/6;
    theta4 = theta4 + pi/2;

    while theta1 > pi 
        theta1 = theta1 - pi;
    end
    while theta2 > pi 
        theta2 = theta2 - pi;
    end
    while theta3 > pi 
        theta3 = theta3 - pi;
    end
    while theta4 > pi 
        theta4 = theta4 - pi;
    end
    while theta5 > pi 
        theta5 = theta5 - pi;
    end

    ikine5s = [theta1, theta2, theta3, theta4, theta5];
end

