
function [new_t1,new_t2,T] = ik(r1,r2,t1,t2,x_d)
%forward kinematics for given theta values
T = [cos(t1) -sin(t1) 0 r1*cos(t1) + r2*cos(t1 + t2);
     sin(t1)  cos(t1) 0 r1*sin(t1) + r2*sin(t1 + t2);
     0        0       1 0;
     0        0       0 1];

%get position for end effector
x=T(1,4);
y=T(2,4);

%distance between desired and current position
delta_x=[x_d(1)-x;x_d(2)-y];

%compute jacobian
J=ik_jacobian(r1, r2, t1, t2);
%compute inverse jacobian
inv_J=pinv(J);
%compute change of joint angles

delta_q=inv_J * delta_x;
%compute the next joint angles
new_t1=t1+delta_q(1);
new_t2=t2+delta_q(2);
end


