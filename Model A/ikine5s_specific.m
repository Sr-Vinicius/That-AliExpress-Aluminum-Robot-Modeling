function [ikine5s] = ikine5s_specific(Kf)
%IKINE5S_SPECIFIC Summary of this function goes here
%   Detailed explanation goes here
    L1 = 82;
    L2 = 104;
    L3 = 98;
    L4 = 28;
    L5 = 50;
    theta1 = atan2(Kf.t(2),Kf.t(1));
    theta5 = atan2(Kf.o(3), Kf.n(3));
    theta234 = atan2(-(Kf.a(1)*cos(theta1)+Kf.a(2)*sin(theta1)), Kf.a(3));

    C3 = ((Kf.t(1)*cos(theta1)+Kf.t(2)*sin(theta1)-L4*cos(theta234)+L5*sin(theta234))^2 + (Kf.t(3)-L1-L5*cos(theta234)-L4*sin(theta234))^2 - L2^2 - L3^2)/(2*L2*L3);
    S3 = sqrt(1-C3^2);
    theta3 = atan2(S3,C3);

    num2 = (Kf.t(3)-L1-L4*sin(theta234)-L5*cos(theta234))*(L3*cos(theta3)+L2) - (L3*sin(theta3))*(Kf.t(1)*cos(theta1)+Kf.t(2)*sin(theta1)-L4*cos(theta234)+L5*sin(theta234));
    den2 = (Kf.t(3)-L1-L4*sin(theta234)-L5*cos(theta234))*(L3*sin(theta3)) + (L3*cos(theta3)+L2)*(Kf.t(1)*cos(theta1)+Kf.t(2)*sin(theta1)-L4*cos(theta234)+L5*sin(theta234));
    theta2 = atan2(num2,den2);

    theta4 = theta234 - theta2 - theta3;
    
    ikine5s = [theta1, theta2, theta3, theta4, theta5];
end

