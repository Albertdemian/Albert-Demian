%This function calculates forward kinematics for any number of DOF
%manipulator

function fun = trans(param)
%param is the matrix that has the DH parameter with this arrangement (theta d a alpha)

[f g]= size(param); %returns the size f is number of rows which is the degrees of freedom of this robot
fun=eye(4);         %Identity matrix as initial value... This value will store the complete HT

for i = 1:f         %looping for all DOF 
theta  = deg2rad(param(i,1));       %Rotation around z axis
di     = param(i,2);                %Translation along Z axis
ai     = param(i,3);                %Translation along X axis
alpha  = deg2rad(param(i,4));       %Rotation around X axis

%Building HT matrices for all above variables
Rz = [cos(theta) -sin(theta) 0 0;sin(theta)  cos(theta) 0 0;0 0 1 0; 0 0 0 1];
Tz = [1 0 0 0;0 1 0 0;0 0 1 di;0 0 0 1];
Tx = [1 0 0 ai;0 1 0 0;0 0 1 0;0 0 0 1];
Rx = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0;0 sin(alpha) cos(alpha) 0;0 0 0 1];

%HT matrix for a single link with index(i)
fun_link = Rz*Tz*Tx*Rx;

%HT matrix from frame{0} to frame{i}
fun = fun*fun_link
end
end

