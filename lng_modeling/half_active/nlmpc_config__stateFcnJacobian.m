function [Jx, Ju] = nlmpc_config__stateFcnJacobian(x, u, p)
% This function was generated by Model Predictive Control Toolbox (Version 24.1).
% 2024/05/07 19:06:36
%# codegen
persistent ADdata
if isempty(ADdata)
    ADdata = coder.load('nlmpc_config__stateFcnJacobianADdata','constants');
end
params.p = p;
[~,J] = nlmpc_config__stateFcnJacobianAD([x;u],ADdata.constants,params);
Jx = J(:,1:14);
Ju = J(:,[15 16 17 18]);
