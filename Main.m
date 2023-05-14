function Main
disp("Main Menu");
disp("1. Get all transformation matrix (symbolic format)");
disp("2. Forward Kinematics");
disp("3. Inverse Kinematics");
disp("4. Get Jacobian Matrix (Symbolic format)");
disp("5. Find overall/joint velocity components");
disp("6. To Know the singularity condition of the manipulator")
disp("7. To find inverse kinematics without joint constrains")

prompt = "Enter your Choice: ";
answer = input(prompt);

if answer == 1
    Link_trans_matrix()
elseif answer == 2
    End_Effector_Loc()
elseif answer == 3
    Inverse_kinematics(0,0,0,0,0,0,0)
elseif answer == 4
    jacobian()
elseif answer == 5
    velocity_mapping()
elseif answer == 6
    warning("Function to throw an error and terminate");
    pause(1.2)
    singular()    
elseif answer == 7
    Inverse_kinematics_temp(0,0,0,0,0,0,0)
else
    error("Wrong Choice!! Exiting...")
end
end