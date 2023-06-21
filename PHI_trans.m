function PHI_out = PHI_trans(qi,qj,sLPi,sLPj,vLi,vLj)
% this function calculates the constraint residuals of a planar revolute joint
ri = [qi(1);qi(2)];
rj = [qj(1);qj(2)];
R = [0 -1;1 0];
PHI_out = [vLi'*(B_mat(qi))'*(rj - ri)-vLi'*Bij_mat(qi,qj)*sLPj-vLi'*R'*sLPi
            -vLi'*Bij_mat(qi,qj)*vLj];
end

