%Program to compute the inverse kinematics 

function [Jointvalues] = Inverse_kinematics(Px, Py, Pz, xang, yang, zang, par_input)
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
    disp("TCP Orientation variables in ZYX euler angle reprsentations")
    prompt = "alpha  (X angle)  = ";
    xang = input(prompt);
    prompt = "beta (Y angle) = ";
    yang = input(prompt);
    prompt = "gamma (Z angle) = ";
    zang=input(prompt);
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

%TCP and wrist compensation

rotx = [1 0 0; 
        0 cosd(xang) -sind(xang); 
        0 sind(xang) cosd(xang);];
roty = [cosd(yang) 0 sind(yang);
        0 1 0; 
        -sind(yang) 0 cosd(yang);];
rotz = [cosd(zang) -sind(zang) 0; 
        sind(zang) cosd(zang) 0; 
        0 0 1;];
%----------------------------------------
Rot_overall = (rotz*roty*rotx);

%To define a TCP of end effector
pos = [Px; Py; Pz];
TCP =[0;0;90;];
if par_input~=1
    prompt = ("Do you have an end effector attached? (1-Yes/0-No): ");
    answer2 = input(prompt);
    
    if answer2 == 1
        prompt = 'Press 1 to enter the coordinates with respect to base and 2 with respect to frame 6: ';
        answer3 = input(prompt);
        prompt = "TCP_X = ";
        TCP_X = input(prompt);
        prompt = "TCP_Y = ";
        TCP_Y = input(prompt);
        prompt = "TCP_Z = ";
        TCP_Z = input(prompt);
        
        %TCP WRT GLobal coordinate system
        if answer3 ==1
            TCP = [TCP_Y; TCP_Z; TCP_X; 0;];
            
        %TCP WRT frame 6
        elseif answer3 == 2
            TCP_Loc = [TCP_X; TCP_Y; TCP_Z;]
            TCP = TCP+TCP_Loc;
        end
    end
end


newpos = pos - Rot_overall*TCP
Px = newpos(1)
Py = newpos(2)
Pz = newpos(3)


%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%Theta1
theta1 = -atan2d(Pz, Px)
if (theta1<-90 || theta1>90)
    warning("Theta1 is out of joint design");
    disp("Setting theta1 to NAN");
    theta1 = nan;
end
theta11 = theta1+180;
if (theta11<-90 || theta11>90)
    warning("Theta11 is out of joint design");
    disp("Setting theta11 to NAN");
    theta11 = nan;
end

%theta3
theta3p = asind(((Px/cosd(theta1))-a2 - a4)/d5);
theta3 = 90-theta3p %To measure wrt x3 and x4
if (theta3<-90 || theta3>90)
    warning("Theta3 is out of joint design");
    disp("Setting theta3 to NAN");
    theta3 = nan;
end

theta3a = -theta3;
if (theta3a<-90 || theta3a>90)
    warning("Theta3a is out of joint design");
    disp("Setting theta3a to NAN");
    theta3a = nan;
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%theta3 wrt theta1prime
theta33p = asind(((Px/cosd(theta11))-a2 - a4)/d5);
theta33 = 90-theta33p;
if (theta33<-90 || theta33>90)
    warning("Theta33 is out of joint design");
    disp("Setting theta33 to NAN");
    theta33 = nan;
end

theta33a = -theta33;
if (theta33a<-90 || theta33a>90)
    warning("Theta33a is out of joint design");
    disp("Setting theta33a to NAN");
    theta33a = nan;
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%d3
d3 = Py-d1-d5*sind(theta3)
if (d3<0 || d3>2500)
    warning("d3 is out of joint design");
    disp("Setting d3 to NAN");
    d3 = nan;
end
d3a = Py-d1-d5*sind(theta3a);
if (d3a<0 || d3a>2500)
    warning("d3a is out of joint design");
    disp("Setting d3a to NAN");
    d3a = nan;
end
%d3 wrt theta1prime
d33 = Py-d1-d5*sind(theta33);
if (d33<0 || d33>2500)
    warning("d33 is out of joint design");
    disp("Setting d33 to NAN");
    d33 = nan;
end
d33a = Py-d1-d5*sind(theta33a);
if (d33a<0 || d33a>2500)
    warning("d33a is out of joint design");
    disp("Setting d33a to NAN");
    d33a = nan;
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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


% %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%To check if atleast one solution exists
flag = 0;
j = 1;
jointv = [];
Workspace_cond = isnan(Jointvalues); %converts the joint value matrix to logical matrix

%Loop to check each row for non NAN values
for i = 1:8
    if any(Workspace_cond(i, :))
        flag = flag + 1;
    else
        jointv(j, :)  = Jointvalues(i,:);
        j = j+1;
        
    end
end

%flag will be 8 if all solutions has atleast one NAN any() returns 1 for
%such solutions
if flag == 8
    warning("+++SINGULARITY CONDITION");
    error("The point is out of work envelope No solution exist). Exiting...")

end

Jointvalues =  jointv;

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




