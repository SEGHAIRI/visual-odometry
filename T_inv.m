function T1 = T_inv(T)
    T1(1:3,1:3) = T(1:3,1:3)';
    T1(1,4)=-T(1:3,1)'*T(1:3,4);
    T1(2,4)=-T(1:3,2)'*T(1:3,4);
    T1(3,4)=-T(1:3,3)'*T(1:3,4);
    T1(4,:)=[0 0 0 1];
end