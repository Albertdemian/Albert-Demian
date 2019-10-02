% Albert Demian
% KUKA KR 10 1100-2
% Robot kinematic model and DH Parameter

L(1) = Link([0 400 25 deg2rad(-90)]);
L(2) = Link([0 0 560 0 0 deg2rad(-90)]);
L(3) = Link([0 0 25 deg2rad(-90)]);
L(4) = Link([0 515 0 deg2rad(90)]);
L(5) = Link([0 0 0 deg2rad(-90)]);
L(6) = Link([0 90 0 0]);


H = SerialLink(L, 'name','KUKA KR 10 1100-2')

