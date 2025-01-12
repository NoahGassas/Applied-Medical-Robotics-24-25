function [T] = fk(r1, r2, t1, t2)
% TODO replace with the values of an homogenous transformation matrix
   %t1 = deg2rad(t1);
   %t2 = deg2rad(t2);
T1 = [cos(t1), -sin(t1), 0, r1 * cos(t1);
      sin(t1),  cos(t1), 0, r1 * sin(t1);
      0,           0,            1, 0;
      0,           0,            0, 1];
T2 = [cos(t2), -sin(t2), 0, r2 * cos(t2);
      sin(t2),  cos(t2), 0, r2 * sin(t2);
      0,           0,            1, 0;
      0,           0,            0, 1];
T = T1 * T2;
end