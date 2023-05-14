% Function to define the transformation matrix

function [Trans] = Trans_Matrix(ai, alpha, d, tetha)
    Trans = [cosd(tetha),            -1*sind(tetha),          0,             ai;
             sind(tetha)*cosd(alpha), cosd(tetha)*cosd(alpha), -1*sind(alpha), -d*sind(alpha);
             sind(tetha)*sind(alpha), cosd(tetha)*sind(alpha), cosd(alpha) ,    d*cosd(alpha);
             0,                     0,                      0,              1;];
end
    

