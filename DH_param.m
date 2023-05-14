%Function to calculate the position of the end effector
%Similar to Link_Trans_Matrix function but takes all the variables as
%inputs and returns the overall transformation matrix

function  [trans]= DH_param (d3, theta1, theta3, theta4, theta5, theta6, reassign)

%Predifined values for the link offsets and link lengths
d1 = 400;
d5 = 1800;
a2 = 400;
a4 = 300;

%To change link offset and lengths
if reassign == 1
    prompt = "Would you like to specify values for d2, a3, a4 and d5? (1-Yes/0-No): ";
    answer = input(prompt);
    
    if answer == 1
        disp("Enter the following values") 
        prompt = 'd1 = ';
        d1 = input(prompt);
        prompt = 'a2 = ';
        a2 = input(prompt);
        prompt = 'a4 = ';
        a4 = input(prompt);
        prompt = 'd5 = ';
        d5 = input(prompt);
        
    end
end

%Function calls to calculate the transformation matrics for joint i wrt i-1
t01 = (Trans_Matrix(0, -90, d1, theta1));
t12 = (Trans_Matrix(a2, 0, d3, 0));
t23 = (Trans_Matrix(a4, 90, 0, (theta3+90)));
t34 = (Trans_Matrix(0, 90, d5, theta4));
t45 = (Trans_Matrix(0, -90, 0, theta5));
t56 = (Trans_Matrix(0, 90, 0, theta6));


%Calculation of overall transformation matrix from frame 0 to frame 7


t02 = (t01*t12);
t03 = (t01*t12*t23);
t04 = (t01*t12*t23*t34);
t05 = (t01*t12*t23*t34*t45);
t06 = (t01*t12*t23*t34*t45*t56);


trans = [t01, t02, t03, t04, t05, t06];

