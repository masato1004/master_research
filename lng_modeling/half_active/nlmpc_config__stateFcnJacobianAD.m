function [obj, grad] = nlmpc_config__stateFcnJacobianAD(inputVariables, extraParams, params)
%nlmpc_config__stateFcnJacobianAD 目的関数の値と gradient を計算します
%
%   OBJ = nlmpc_config__stateFcnJacobianAD(INPUTVARIABLES, EXTRAPARAMS,
%   PARAMS) は、EXTRAPARAMS の追加パラメーターと PARAMS のパラメーターを使用し
%   て、点 INPUTVARIABLES における目的値 OBJ を計算します。
%
%   [OBJ, GRAD] = nlmpc_config__stateFcnJacobianAD(INPUTVARIABLES,
%   EXTRAPARAMS, PARAMS) はさらに、現在の点における目的関数の gradient の値 GRAD
%   を計算します。
%
%   2024/06/30 02:26:16 に prob2struct により自動生成

%#codegen
%#internal
%% 変数のインデックス。
uidx = 15:18;
xidx = 1:14;

%% ソルバーベースの変数を問題ベースにマップします。
u = inputVariables(uidx);
u = u(:);
x = inputVariables(xidx);
x = x(:);

%% Extract parameters.
p = params.p;

%% 目的関数を計算します。
arg1 = extraParams{1};
arg2 = extraParams{2};
arg4 = extraParams{3};
arg3 = p(1:8);
arg5 = arg3(:);
obj = (((arg1 * x) + (arg2 * u)) + (arg4 * arg5));

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg9 = zeros(18, 14);
    ujac = zeros(4, 14);
    xjac = zeros(14, 14);
    arg6 = eye(14);
    arg7 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg4, [8 1], arg6);
    arg8 = zeros(109, 14);
    arg8(1:8,:) = arg7;
    ujac = ujac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg2, [4 1], arg6);
    xjac = xjac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg1, [14 1], arg6);
    arg9(uidx,:) = ujac;
    arg9(xidx,:) = xjac;
    grad = (reshape(arg9, [18, 14]))';
end
end