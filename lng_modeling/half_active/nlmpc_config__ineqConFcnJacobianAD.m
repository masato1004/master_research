function [obj, grad] = nlmpc_config__ineqConFcnJacobianAD(inputVariables, extraParams, params)
%nlmpc_config__ineqConFcnJacobianAD 目的関数の値と gradient を計算します
%
%   OBJ = nlmpc_config__ineqConFcnJacobianAD(INPUTVARIABLES, EXTRAPARAMS,
%   PARAMS) は、EXTRAPARAMS の追加パラメーターと PARAMS のパラメーターを使用し
%   て、点 INPUTVARIABLES における目的値 OBJ を計算します。
%
%   [OBJ, GRAD] = nlmpc_config__ineqConFcnJacobianAD(INPUTVARIABLES,
%   EXTRAPARAMS, PARAMS) はさらに、現在の点における目的関数の gradient の値 GRAD
%   を計算します。
%
%   2024/05/04 02:08:28 に prob2struct により自動生成

%#codegen
%#internal
%% 変数のインデックス。
uidx = 15:18;
xidx = 1:14;
eidx = 23;

%% ソルバーベースの変数を問題ベースにマップします。
u = inputVariables(uidx);
u = u(:);
x = inputVariables(xidx);
x = x(:);
e = inputVariables(eidx);

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
arg26 = ((((arg20 * x) + (arg21 * u)) + (arg23 * arg24)) + (arg18 .* arg19));
arg27 = arg26(8);
obj = ((arg27.^2 - extraParams{8}) - e);

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg46 = zeros(23, 1);
    ujac = zeros(4, 1);
    xjac = zeros(14, 1);
    ejac = 0;
    arg28 = 1;
    ejac = ejac + (-arg28);
    arg29 = (arg28.*2.*(arg27(:)));
    arg30 = zeros(14, 1);
    arg30(8,:) = arg29;
    arg31 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg23, [8 1], arg30);
    arg32 = zeros(198, 1);
    arg32(1:8,:) = arg31;
    ujac = ujac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg21, [4 1], arg30);
    xjac = xjac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg20, [14 1], arg30);
    arg33 = (arg30.*arg19(:));
    arg34 = arg33(14,:);
    arg35 = (((-optim.coder.problemdef.gradients.divide.DivideLeftJacobian(arg16)*arg34).*cos(arg14(:)))./(1+arg13(:).^2));
    arg36 = optim.coder.problemdef.gradients.divide.DivideRightJacobian(arg11, arg12)*arg35;
    arg37 = zeros(198, 1);
    arg37(6,:) = arg36;
    arg38 = optim.coder.problemdef.gradients.divide.DivideLeftJacobian(arg12)*arg35;
    arg39 = zeros(198, 1);
    arg39(8,:) = arg38;
    arg40 = arg33(13,:);
    arg41 = (((-optim.coder.problemdef.gradients.divide.DivideLeftJacobian(arg8)*arg40).*cos(arg6(:)))./(1+arg5(:).^2));
    arg42 = optim.coder.problemdef.gradients.divide.DivideRightJacobian(arg3, arg4)*arg41;
    arg43 = zeros(198, 1);
    arg43(5,:) = arg42;
    arg44 = optim.coder.problemdef.gradients.divide.DivideLeftJacobian(arg4)*arg41;
    arg45 = zeros(198, 1);
    arg45(7,:) = arg44;
    arg46(uidx,:) = ujac;
    arg46(xidx,:) = xjac;
    arg46(eidx,:) = ejac;
    grad = (arg46(:))';
end
end