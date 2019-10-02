%Albert Demian
%gauss method to solve for inverse kinematics
%
syms th1 th2 th3 a1 a2 a3 d1 d2 d3 alp1 alp2 alp3

alp1 = deg2rad(-90);
alp2 = deg2rad(0);
alp3 = deg2rad(-90)
a1 = 25;
a2 = 560;
a3 = 25;
d1 = 400;
d2=0;
d3=0;

r1r1 = [cos(th1) -sin(th1)*cos(alp1) sin(th1)*sin(alp1) a1*cos(th1)];
r1r2 = [sin(th1) cos(th1)*cos(alp1) -cos(th1)*sin(alp1) a1*sin(th1)];
r1r3 = [0        sin(alp1)           cos(alp1)          d1         ];
r1r4 = [0 0 0 1];
R1 = [r1r1; r1r2; r1r3; r1r4];

r2r1 = [cos(th2) -sin(th2)*cos(alp2) sin(th2)*sin(alp2) a2*cos(th2)];
r2r2 = [sin(th2) cos(th2)*cos(alp2) -cos(th2)*sin(alp2) a2*sin(th2)];
r2r3 = [0        sin(alp2)           cos(alp2)          d2         ];
r2r4 = [0 0 0 1];
R2 = [r2r1; r2r2; r2r3; r2r4];

r3r1 = [cos(th3) -sin(th3)*cos(alp3) sin(th3)*sin(alp3) a3*cos(th3)];
r3r2 = [sin(th3) cos(th3)*cos(alp3) -cos(th3)*sin(alp3) a3*sin(th3)];
r3r3 = [0        sin(alp3)           cos(alp3)          d3         ];
r3r4 = [0 0 0 1];
R3 = [r3r1; r3r2; r3r3; r3r4];

T = R1*R2*R3;

z_vec = [85.59 15.12 -23.31]';
T_pos = T(1:3,4);

F(th1,th2,th3) = T_pos - z_vec;
d_th1 = diff(F,th1);
d_th2 = diff(F,th2);
d_th3 = diff(F,th3);

J(th1,th2,th3) = [d_th1 d_th2 d_th3];

th = [0.1;0.1;0.1];

for i = 1:40
    x1 = round(F(th(1),th(2),th(3)));
    x2 = round(J(th(1),th(2),th(3)));
    Y = round(inv(x2)*x1);
    
    th = round(th+Y) 
    disp(i)
end

    

