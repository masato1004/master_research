function [obj, grad] = nlmpc_config__costFcnJacobianAD(inputVariables, extraParams, params)
%nlmpc_config__costFcnJacobianAD 目的関数の値と gradient を計算します
%
%   OBJ = nlmpc_config__costFcnJacobianAD(INPUTVARIABLES, EXTRAPARAMS,
%   PARAMS) は、EXTRAPARAMS の追加パラメーターと PARAMS のパラメーターを使用し
%   て、点 INPUTVARIABLES における目的値 OBJ を計算します。
%
%   [OBJ, GRAD] = nlmpc_config__costFcnJacobianAD(INPUTVARIABLES,
%   EXTRAPARAMS, PARAMS) はさらに、現在の点における目的関数の gradient の値 GRAD
%   を計算します。
%
%   2024/05/08 12:30:27 に prob2struct により自動生成

%#codegen
%#internal
%% 変数のインデックス。
xidx = 1:14;
uidx = 15:18;
eidx = 23:26;

%% ソルバーベースの変数を問題ベースにマップします。
x = inputVariables(xidx);
x = x(:);
u = inputVariables(uidx);
u = u(:);
e = inputVariables(eidx);
e = e(:);

%% Extract parameters.
p = params.p;

%% 目的関数を計算します。
arg3 = extraParams{1};
arg5 = extraParams{2};
arg10 = extraParams{3};
arg12 = extraParams{4};
arg15 = extraParams{5};
arg17 = extraParams{6};
arg1 = p(85:98);
arg2 = (x - arg1(:)).';
arg4 = (arg2 .* arg3);
arg6 = p(85:98);
arg7 = (arg4 * arg5);
arg8 = (x - arg6(:));
arg9 = u.';
arg11 = (arg9 .* arg10);
arg13 = (arg11 * arg12);
arg14 = e.';
arg16 = (arg14 .* arg15);
arg18 = (arg16 * arg17);
obj = (((arg7 * arg8) + (arg13 * u)) + (arg18 * e));

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg35 = zeros(26, 1);
    xjac = zeros(14, 1);
    ujac = zeros(4, 1);
    ejac = zeros(4, 1);
    arg19 = 1;
    ejac = ejac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg18, [4 1], arg19);
    arg20 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], e, arg19);
    arg21 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg17, arg20);
    arg22 = (arg21.*arg15(:));
    ejac = ejac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg22,[1 4]);
    ujac = ujac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg13, [4 1], arg19);
    arg23 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], u, arg19);
    arg24 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg12, arg23);
    arg25 = (arg24.*arg10(:));
    ujac = ujac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg25,[1 4]);
    arg26 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg7, [14 1], arg19);
    xjac = xjac + arg26;
    arg27 = (-arg26);
    arg28 = zeros(98, 1);
    arg28(85:98,:) = arg27;
    arg29 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 14], arg8, arg19);
    arg30 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 14], arg5, arg29);
    arg31 = (arg30.*arg3(:));
    arg32 = optim.problemdef.gradients.indexing.TransposeAdjoint(arg31,[1 14]);
    xjac = xjac + arg32;
    arg33 = (-arg32);
    arg34 = zeros(98, 1);
    arg34(85:98,:) = arg33;
    arg35(xidx,:) = xjac;
    arg35(uidx,:) = ujac;
    arg35(eidx,:) = ejac;
    grad = arg35(:);
end
end