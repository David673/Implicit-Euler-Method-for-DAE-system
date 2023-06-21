function [q_ip1,lambda_ip1] = dqSliderImplicit(t,xIN)
x_ip1 = fsolve(@fcnSliderImplicit,xIN,optimset('Display','off'));
q_ip1 = x_ip1(1:6);
lambda_ip1 = x_ip1(7:8);
end