function [Jx, Ju, Jdmv, Je] = nlmpc_config__costFcnJacobian(stage, x, u, dmv, e, p)
% This function was generated by Model Predictive Control Toolbox (Version 24.1).
% 2024/05/07 17:38:18
%# codegen
persistent ADdata
if isempty(ADdata)
    ADdata = coder.load('nlmpc_config__costFcnJacobianADdata','constants');
end
params.stage = stage;
params.p = p;
[~,J] = nlmpc_config__costFcnJacobianAD([x;u;dmv;e],ADdata.constants,params);
Jx = J(1:14,:);
Ju = J([15 16 17 18],:);
Jdmv = J(19:22,:);
Je = J(23:26,:);
