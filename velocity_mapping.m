function velocity_mapping
%Function to compute the joint/global velocities using jacobian 

disp("______________________________________________________________");
disp("Joint values will be taken as from home (all 0)");
    disp("Do you wish to enter or compute joint values (Position of TCP)");
    disp("______________________________________________________________");
    prompt = "Enter 0 for default values; 1 for custom input and 2 to compute from position: ";
    answer2 = input(prompt);
    if answer2 == 0
        theta1 = 0;
        d3 = 0;
        theta3 = 0;
        theta4 = 0;
        theta5 = 0;
        theta6 = 0;
       
    elseif answer2 == 1
        disp("Enter the following robot variable values")
        prompt = ('theta1 =');
        theta1 = input(prompt);
        prompt = ('d3 =');
        d3 = input(prompt);
        prompt = ('theta3 =');
        theta3 = input(prompt);
        prompt = ('theta4 =');
        theta4 = input(prompt);
        prompt = ('theta5 =');
        theta5 = input(prompt);
        prompt = ('theta6 =');
        theta6 = input(prompt);

    elseif answer2 == 2
        disp("Calling Inverse kinematic function");
        jointvalues = Inverse_kinematics(0,0,0,0,0,0,0);
        prompt = "Enter the configureation (set) 1 to 8: ";
        set = input(prompt);
        theta1 = jointvalues(set, 1);
        d3 = jointvalues(set, 2);
        theta3 = jointvalues(set, 3);
        theta4 = jointvalues(set, 4);
        theta5 = jointvalues(set, 5);
        theta6 = jointvalues(set, 6);

        assert((~isnan(theta1) && ~isnan(d3) && ~isnan(theta3) && ~isnan(theta4) && ~isnan(theta5) && ~isnan(theta6)), "Solution doesnot exist, use a viable solution. Exiting!!!")

    else
        error("Wrong Choice! Exiting");
    end

jac = compute_jacobian(theta1, d3, theta3, theta4, theta5, theta6);
disp("______________________________________________________________");
prompt = "Enter 1 to compute the joint velocities and 2 to compute overall velocity: ";
answer = input(prompt);
disp("______________________________________________________________");

if answer == 1
    
    disp("Enter the components of the overall velocity");
    prompt = "X linear component: ";
    Xdot = input(prompt);
    prompt = "Y linear component: ";
    Ydot = input(prompt);
    prompt = "Z linear component: ";
    Zdot = input(prompt);
    prompt = "X angular component: ";
    wXdot = input(prompt);
    prompt = "Y angular component: ";
    wYdot = input(prompt);
    prompt = "Z angular component: ";
    wZdot = input(prompt);
    VelVect = [Xdot; Ydot; Zdot; wXdot; wYdot; wZdot];
    JointVel = (jac) \ VelVect

elseif answer == 2
    prompt = "1st link velocity: ";
    theta1dot = input(prompt);
    prompt = "2nd link velocity: ";
    d3dot = input(prompt);
    prompt = "3rd link velocity: ";
    theta3dot = input(prompt);
    prompt = "4th link velocity: ";
    theta4dot = input(prompt);
    prompt = "5th link velocity: ";
    theta5dot = input(prompt);
    prompt = "6th link velocity: ";
    theta6dot = input(prompt);
    JointVel = [theta1dot; d3dot; theta3dot; theta4dot; theta5dot; theta6dot];
    VelVect = jac * JointVel

else
    error("Wrong Choice!! Exiting...")
end