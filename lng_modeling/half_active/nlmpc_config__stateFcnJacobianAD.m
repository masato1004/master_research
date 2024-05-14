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
%   2024/05/14 14:03:21 に prob2struct により自動生成

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
arg8 = extraParams{2};
arg16 = extraParams{3};
arg19 = extraParams{4};
arg20 = extraParams{5};
arg21 = extraParams{6};
arg23 = extraParams{7};
arg17 = zeros(14, 1);
arg3 = p(7);
arg4 = p(5);
arg5 = (arg3 ./ arg4);
arg6 = atan(arg5);
arg7 = (-sin(arg6));
arg11 = p(8);
arg12 = p(6);
arg13 = (arg11 ./ arg12);
arg14 = atan(arg13);
arg15 = (-sin(arg14));
arg17(1:12) = extraParams{1};
arg17(13) = (arg7 ./ arg8);
arg17(14) = (arg15 ./ arg16);
arg18 = arg17(:);
arg22 = p(1:8);
arg24 = arg22(:);
obj = ((((arg20 * x) + (arg21 * u)) + (arg23 * arg24)) + (arg18 .* arg19));

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg41 = zeros(18, 14);
    ujac = zeros(4, 14);
    xjac = zeros(14, 14);
    arg25 = eye(14);
    arg26 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg23, [8 1], arg25);
    arg27 = zeros(84, 14);
    arg27(1:8,:) = arg26;
    ujac = ujac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg21, [4 1], arg25);
    xjac = xjac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg20, [14 1], arg25);
    arg28 = (arg25.*arg19(:));
    arg29 = arg28(14,:);
    arg30 = (((-optim.coder.problemdef.gradients.divide.DivideLeftJacobian(arg16)*arg29).*cos(arg14(:)))./(1+arg13(:).^2));
    arg31 = optim.coder.problemdef.gradients.divide.DivideRightJacobian(arg11, arg12)*arg30;
    arg32 = zeros(84, 14);
    arg32(6,:) = arg31;
    arg33 = optim.coder.problemdef.gradients.divide.DivideLeftJacobian(arg12)*arg30;
    arg34 = zeros(84, 14);
    arg34(8,:) = arg33;
    arg35 = arg28(13,:);
    arg36 = (((-optim.coder.problemdef.gradients.divide.DivideLeftJacobian(arg8)*arg35).*cos(arg6(:)))./(1+arg5(:).^2));
    arg37 = optim.coder.problemdef.gradients.divide.DivideRightJacobian(arg3, arg4)*arg36;
    arg38 = zeros(84, 14);
    arg38(5,:) = arg37;
    arg39 = optim.coder.problemdef.gradients.divide.DivideLeftJacobian(arg4)*arg36;
    arg40 = zeros(84, 14);
    arg40(7,:) = arg39;
    arg41(uidx,:) = ujac;
    arg41(xidx,:) = xjac;
    grad = (reshape(arg41, [18, 14]))';
end
end