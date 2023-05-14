%Function to calculate the link transformation matrix
%Not an user input function but to calculate the designed 7 DOF
%manipulator's transformation matrix

function[trans] = Link_trans_matrix

% Declaring the joint angles, joint offsets as a variable
syms d1 d3 d5 a2 a4 theta1 theta3 theta4 theta5 theta6

%Function calls to calculate the transformation matrics for joint i wrt i-1
t01 = simplify(Trans_Matrix(0, -90, d1, theta1))
t12 = simplify(Trans_Matrix(a2, 0, d3, 0))
t23 = simplify(Trans_Matrix(a4, 90, 0, (theta3+90)))
t34 = simplify(Trans_Matrix(0, 90, d5, theta4))
t45 = simplify(Trans_Matrix(0, -90, 0, theta5))
t56 = simplify(Trans_Matrix(0, 90, 0, theta6))


t02 = simplify(t01*t12)
t03 = simplify(t01*t12*t23)
t04 = simplify(t01*t12*t23*t34)
t05 = simplify(t01*t12*t23*t34*t45)
t06 = simplify(t01*t12*t23*t34*t45*t56)
t36 = simplify(t34*t45*t56)


trans = [t01, t02, t03, t04, t05, t06];