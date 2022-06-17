function incre_pose = exprotmT(increment)
% calculate the increment pose
%
% INPUT:
%   increment: a vector as [wx, wy, wz, tx, ty, tz]
%
% OUTPUT:
%   incre_pose: a [4, 4] matrix about the rigid transformation

    if size(increment)~=[6 1]
        increment=increment';
    end
    omega   = increment(1:3,1); % se3
    omega_s = [0,-omega(3,1),omega(2,1);omega(3,1),0,-omega(1,1);-omega(2,1),omega(1,1),0];     
    v       = increment(4:6,1); % se3
    
    % Singularity case
    if norm(omega) == 0 
        V = eye(3);
        R = eye(3);
    else
        V = eye(3) + ((1-cos(norm(omega)))/(norm(omega)^2))*omega_s + ((norm(omega)-sin(norm(omega)))/(norm(omega)^3))*(omega_s*omega_s);
        R = eye(3) + (sin(norm(omega))/norm(omega))*omega_s + ((1-cos(norm(omega)))/(norm(omega)^2))*(omega_s*omega_s);
    end
    
    incre_pose = [R,V*v;0 0 0 1];
end