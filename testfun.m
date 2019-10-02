%to solve for forward kinematics
theta = [10 0-90 15 0 0 0];
param(1,:) = [theta(1) 400 25  -90];
param(2,:) = [theta(2) 000 560  00];
param(3,:) = [theta(3) 000 025 -90];
param(4,:) = [theta(4) 515 000  90];
param(5,:) = [theta(5) 000 000 -90];
param(6,:) = [theta(6) 090 000  00]; 

forwardkin = round(trans(param),3)

%inverse
pos_fin = forwardkin(1:3,4)
z_vec = param(6,2)* forwardkin(1:3,1:3)*[0;0;1]