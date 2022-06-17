function P = matrix_to_quaternion(T)
    A = rotm2quat(T(1:3,1:3));
    P(4:6) = A(2:4);
    P(7) = A(1);
    P(1:3) = T(1:3,4);
end