%Function to calculate singularity 
function singular
syms theta1 d3 theta3 theta4 theta5 theta6
jac = jacobian();
jac_det = (det(jac));
eq1 = jac_det == 0;
sing1 = solve(eq1, theta1, d3, theta3, theta4, theta5, theta6)

%out of bound singularity -Function to throw an error and termniate

sing2 = Inverse_kinematics(10000, 10000, 10000, 0, 0, 0, 1) %Arbitary values given
