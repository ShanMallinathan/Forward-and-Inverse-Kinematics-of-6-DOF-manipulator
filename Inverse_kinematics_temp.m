%Program to compute the inverse kinematics without joint restrictions

function [Jointvalues] = Inverse_kinematics_temp(Px, Py, Pz, xang, yang, zang, par_input)
%Predifined values for the link offsets and link lengths
d1 = 400;
d5 = 1800;
a2 = 400;
a4 = 300;

%initialize set values to nan
set1 = [nan nan nan nan nan nan];
set2 = [nan nan nan nan nan nan];
set3 = [nan nan nan nan nan nan];
set4 = [nan nan nan nan nan nan];
set5 = [nan nan nan nan nan nan];
set6 = [nan nan nan nan nan nan];
set7 = [nan nan nan nan nan nan];
set8 = [nan nan nan nan nan nan];

if par_input~=1
    disp("Enter the following values");
    prompt = "Px = ";
    Px = input(prompt);
    prompt = "Py = ";
    Py = input(prompt);
    prompt = "Pz = ";
    Pz = input(prompt);
    prompt = "alpha = ";
    xang = input(prompt);
    prompt = "beta = ";
    yang = input(prompt);
    prompt = "gamma = ";
    zang=input(prompt);
end

%Theta1
theta1 = -atan2d(Pz, Px);
theta11 = theta1+180;


%theta3
theta3p = asind(((Px/cosd(theta1))-a2 - a4)/d5);
theta3 = 90-theta3p; %To measure wrt x3 and x4
theta3ap = -theta3p; %a stands for shoulder down configuration
theta3a = -theta3;
%theta3 wrt theta1prime
theta33p = asind(((Px/cosd(theta11))-a2 - a4)/d5);
theta33 = 90-theta33p;
theta33ap = -theta33p;
theta33a = -theta33;

%d3
d3 = Py-d1-d5*cosd(theta3p);
d3a = Py-d1+d5*cosd(theta3ap);
%d3 wrt theta1prime
d33 = Py-d1-d5*cosd(theta33p);
d33a = Py-d1+d5*cosd(theta33ap);

%Last three angles
if(isreal(theta1) && isreal(d3) && isreal(theta3))
    theta456a = IK_orientation(zang, yang, xang, theta1, d3, theta3);
end
if(isreal(theta1) && isreal(d3a) && isreal(theta3a))
    theta456aa = IK_orientation(zang, yang, xang, theta1, d3a, theta3a);
end
if(isreal(theta11) && isreal(d33) && isreal(theta33))
    theta456b = IK_orientation(zang, yang, xang, theta11, d33, theta33);
end
if(isreal(theta11) && isreal(d33a) && isreal(theta33a))
    theta456bb = IK_orientation(zang, yang, xang, theta11, d33a, theta33a);
end

if(isreal(theta1) && isreal(d3) && isreal(theta3))
    joint_var_pos_S1 = [theta1 d3 theta3];
    joint_var_orient_S11 = [theta456a(1) theta456a(3) theta456a(5)];
    joint_var_orient_S12 = [theta456a(2) theta456a(4) theta456a(6)];
    disp("Set1");
    set1 = [joint_var_pos_S1 joint_var_orient_S11]
    disp("Set2");
    set2 = [joint_var_pos_S1 joint_var_orient_S12]
else
    disp("No Solution for the Set 1 and 2")
end

if(isreal(theta1) && isreal(d3a) && isreal(theta3a))
    joint_var_pos_S2 = [theta1 d3a theta3a];
    joint_var_orient_S21 = [theta456aa(1) theta456aa(3) theta456aa(5)];
    joint_var_orient_S22 = [theta456aa(2) theta456aa(4) theta456aa(6)];
    disp("Set3");
    set3 = [joint_var_pos_S2 joint_var_orient_S21]
    disp("Set4");
    set4 = [joint_var_pos_S2 joint_var_orient_S22]
else
    disp("No Solution for the Set 3 and 4")
end

if(isreal(theta11) && isreal(d33) && isreal(theta33))
    joint_var_pos_S3 = [theta11 d33 theta33];
    joint_var_orient_S31 = [theta456b(1) theta456b(3) theta456b(5)];
    joint_var_orient_S32 = [theta456b(2) theta456b(4) theta456b(6)];
    disp("Set5");
    set5 = [joint_var_pos_S3 joint_var_orient_S31]
    disp("Set6");
    set6 = [joint_var_pos_S3 joint_var_orient_S32]
else
    disp("No Solution for the Set 5 and 6")
end

if(isreal(theta11) && isreal(d33a) && isreal(theta33a))
    joint_var_pos_S4 = [theta11 d33a theta33a];
    joint_var_orient_S41 = [theta456bb(1) theta456bb(3) theta456bb(5)];
    joint_var_orient_S42 = [theta456bb(2) theta456bb(4) theta456bb(6)];
    disp("Set7");
    set7 = [joint_var_pos_S4 joint_var_orient_S41]
    disp("Set8");
    set8 = [joint_var_pos_S4 joint_var_orient_S42]
else
    disp("No Solution for the Set 7 and 8")
end

Jointvalues = [set1; set2; set3; set4; set5; set6; set7; set8;];
% 
% %TCP and wrist compensation
% 
% rotx = [1 0 0; 
%         0 cosd(xang) -sind(xang); 
%         0 sind(xang) cosd(xang);];
% roty = [cosd(yang) 0 sind(yang);
%         0 1 0; 
%         -sind(yang) 0 cosd(yang);];
% rotz = [cosd(zang) -sind(zang) 0; 
%         sind(zang) cosd(zang) 0; 
%         0 0 1;];
% 
% Rot_overall = (rotz*roty*rotx);
% pos = [Px; Py; Pz];
% TCP = [0;0;90];
% Rot_overall*TCP
% newpos = pos - TCP.*Rot_overall*[0;0;1]
% Px = newpos(1);
% Py = newpos(2);
% Pz = newpos(3);




