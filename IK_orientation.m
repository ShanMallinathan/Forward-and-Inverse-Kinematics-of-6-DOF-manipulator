function [theta567] = IK_orientation(zang, yang, xang, theta1, d3, theta3)

%Program to compute the orientation part of the inverse kinematics
syms theta4 theta5 theta6


rotx = [1 0 0; 
        0 cosd(xang) -sind(xang); 
        0 sind(xang) cosd(xang);];
roty = [cosd(yang) 0 sind(yang);
        0 1 0; 
        -sind(yang) 0 cosd(yang);];
rotz = [cosd(zang) -sind(zang) 0; 
        sind(zang) cosd(zang) 0; 
        0 0 1;];

Rot_overall = (rotz*roty*rotx);
trans = DH_param(d3, theta1, theta3, theta4, theta5, theta6, 0);
rot_03  = trans(1:3, 9:11);
rot_03Transpose = transpose(rot_03);
Lrot_36 = (rot_03Transpose*Rot_overall);


%Throws an error when the user inputs out of bounds values
try
    theta5 = eval(atan2d(sqrt((Lrot_36( 2,1))^2 + (Lrot_36(2,2))^2), -Lrot_36(2,3)));
catch
    warning("+++SINGULARITY CONDITION+++");
    error("Position Out of work envelope. Please check the value!!! Exiting the program...");
end
theta4 = eval(atan2d(-Lrot_36(3,3),  Lrot_36(1,3)));     
theta6 = eval(atan2d(-Lrot_36(2,2),  Lrot_36(2,1)));

theta5dash = -theta5;
theta4dash = theta4 + 180;
theta6dash = theta6 + 180;

%To induce mechanical constrain to joint 5 

if (theta5<-170 || theta5>170)
    warning("theta5 is out of joint design");
    disp("Setting theta5 to NAN");
    theta5 = nan;
end
if (theta5dash<-170 || theta5dash>170)
    warning("theta5dash is out of joint design");
    disp("Setting theta5dash to NAN");
    theta5dash = nan;
end

theta567 = [theta4 theta4dash theta5 theta5dash theta6 theta6dash];
end