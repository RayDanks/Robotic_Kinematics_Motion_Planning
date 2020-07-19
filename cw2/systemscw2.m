%% check

FirstVector = [0;0;1];
SecondVector = [2;3;4];
disp(SecondVector(1))

Cross = cross(FirstVector,SecondVector) %Does work!
%Multiply = FirstVector*SecondVector %Doesn't work (wrong dims)

%% syms trig test
syms x 
d = sin(x+pi);
disp(d)
disp(length(d))

%% Forward_kine
clc

syms theta1 theta2 theta3 theta4 theta5 a1 a2 a3 a4 d1 d5 pi
aM = [a1,a2,a3,a4,0];
alphaM = [pi/2,0,0,pi/2,pi];
dM = [d1,0,0,0,d5];
thetaM = [theta1+pi, theta2+(pi/2), theta3, theta4-(pi/2), theta5+pi];


% A = [theta2,theta3];
% B = [theta2,theta1];
% C = A.*B;
% disp(C)


    
    T = eye(4, 4);

    for i = 1:5
        
        A = dh_matrix_standard(aM(i), alphaM(i), dM(i), thetaM(i));

        T = simplify(T * A);
    end

T = simplify(T);
%disp(T)
disp(T)

%Now define the equations for each element and relate it to the desired
%transform from the base (ie where we want the end effector to be)
% 
% syms r11 r12 r13 r21 r22 r23 r31 r32 r33 pex pey pez
% 
% DesiredTransform = [r11,r12,r13,pex;r21,r22,r23,pey;r31,r32,r33,pez;0,0,0,1];
% 
% ThetaEquations = sym(zeros(5,1));




function A = dh_matrix_standard(a, alpha, d, theta)

    A = sym(zeros(4,4));
    A(4, 4) = 1.0;
    A(4, 3) = 0.0;
    A(4, 2) = 0.0;
    A(4, 1) = 0.0;

    A(1, 1) = cos(theta);
    A(1, 2) = -sin(theta)*cos(alpha);
    A(1, 3) = sin(theta)*sin(alpha);
    A(1, 4) = a * cos(theta);

    A(2, 1) = sin(theta);
    A(2, 2) = cos(theta)*cos(alpha);
    A(2, 3) = -cos(theta)*sin(alpha);
    A(2, 4) = a * sin(theta);

    A(3, 1) = 0.0;
    A(3, 2) = sin(alpha);
    A(3, 3) = cos(alpha);
    A(3, 4) = d;

end
