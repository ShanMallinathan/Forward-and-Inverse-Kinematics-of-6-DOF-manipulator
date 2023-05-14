function Roboplotter(trans)

p0 = [0;0;0;1];
p1 = trans(:, 1:4)*p0;
p2dash = p1 + [trans(1:3, 1:3)*[400;0;0];0];
p2 = trans(:, 5:8)*p0;
p3 = trans(:, 9:12)*p0;
p4 = trans(:, 13:16)*p0;
p5 = trans(:, 17:20)*p0;
p6 = trans(:, 21:24)*p0;
coordinates = [p0 p1 p2dash p2 p3 p4 p5 p6];
plot3(coordinates(1,:),coordinates(2,:),coordinates(3,:), 'r' ); 