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
%   2024/05/07 12:52:45 に prob2struct により自動生成

%#codegen
%#internal
%% 変数のインデックス。
xidx = 1:14;
uidx = 15:18;
eidx = 23;

%% ソルバーベースの変数を問題ベースにマップします。
x = inputVariables(xidx);
x = x(:);
u = inputVariables(uidx);
u = u(:);
e = inputVariables(eidx);

%% Extract parameters.
p = params.p;
stage = params.stage;

%% 目的関数を計算します。
arg3 = extraParams{1};
arg6 = extraParams{2};
arg11 = extraParams{3};
arg13 = extraParams{4};
arg16 = extraParams{5};
arg18 = 1000000;
arg1 = p(185:198);
arg2 = (x - arg1(:)).';
arg4 = (arg2 .* arg3);
arg5 = (arg4 .* stage);
arg7 = p(185:198);
arg8 = (arg5 * arg6);
arg9 = (x - arg7(:));
arg10 = u.';
arg12 = (arg10 .* arg11);
arg14 = (arg12 * arg13);
arg15 = e.';
arg17 = (arg15 .* arg16);
arg19 = (arg17 .* arg18);
obj = (((arg8 * arg9) + (arg14 * u)) + (arg19 .* e));

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg37 = zeros(23, 1);
    xjac = zeros(14, 1);
    ujac = zeros(4, 1);
    ejac = 0;
    arg20 = 1;
    ejac = ejac + (arg20.*arg19(:));
    arg21 = (arg20.*e(:));
    arg22 = (arg21.*arg18(:));
    arg23 = (arg22.*arg16(:));
    ejac = ejac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg23,[1 1]);
    ujac = ujac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg14, [4 1], arg20);
    arg24 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], u, arg20);
    arg25 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg13, arg24);
    arg26 = (arg25.*arg11(:));
    ujac = ujac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg26,[1 4]);
    arg27 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg8, [14 1], arg20);
    xjac = xjac + arg27;
    arg28 = (-arg27);
    arg29 = zeros(198, 1);
    arg29(185:198,:) = arg28;
    arg30 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 14], arg9, arg20);
    arg31 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 14], arg6, arg30);
    arg32 = (arg31.*stage(:));
    arg33 = (arg32.*arg3(:));
    arg34 = optim.problemdef.gradients.indexing.TransposeAdjoint(arg33,[1 14]);
    xjac = xjac + arg34;
    arg35 = (-arg34);
    arg36 = zeros(198, 1);
    arg36(185:198,:) = arg35;
    arg37(xidx,:) = xjac;
    arg37(uidx,:) = ujac;
    arg37(eidx,:) = ejac;
    grad = arg37(:);
end
end