function Aout = A_mat(q)
phi = q(3);
Aout = [cos(phi) -sin(phi)
    sin(phi) cos(phi)];
end
