# Forward-and-Inverse-Kinematics-of-6-DOF-manipulator

This matlab code is to compute forward and inverse kinematics, Jacobians, Singulariy conditions and transformation matrix of the 6DOF manipulator whose configuration is as shown below.

![image](https://github.com/ShanMallinathan/Forward-and-Inverse-Kinematics-of-6-DOF-manipulator/assets/115928550/2add025a-3769-420e-ba68-a1e7c5f35b63)

![image](https://github.com/ShanMallinathan/Forward-and-Inverse-Kinematics-of-6-DOF-manipulator/assets/115928550/7eac9d78-c079-4f37-ac2e-aca2daf4c3fe)

compile Main.m file for computing kinematics, jacobians, singularities, velocities, transformation matrix - Select relavant option from the menu

Home pose : with joint offset between link 5 and 6

[p; [alpha; beta; gamma;] = [2590; 400; 0; 90; 0; 0]

without joint offset


[p; [alpha; beta; gamma;] = [2500; 400; 0; 90; 0; 0]


Control : run control.slx
