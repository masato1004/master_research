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
%   2024/06/23 23:26:42 に prob2struct により自動生成

%#codegen
%#internal
%% 変数のインデックス。
xidx = 1:14;
uidx = 15:18;
dmvidx = 19:22;

%% ソルバーベースの変数を問題ベースにマップします。
x = inputVariables(xidx);
x = x(:);
u = inputVariables(uidx);
u = u(:);
dmv = inputVariables(dmvidx);
dmv = dmv(:);

%% Extract parameters.
p = params.p;
stage = params.stage;

%% 目的関数を計算します。
arg4 = extraParams{1};
arg6 = extraParams{2};
arg14 = extraParams{3};
arg16 = extraParams{4};
arg19 = extraParams{5};
arg21 = extraParams{6};
arg1 = x(13:14);
arg2 = p(122:123);
arg3 = (arg1(:) - arg2(:)).';
arg5 = (arg3 .* arg4);
arg7 = (arg5 * arg6);
arg8 = (11 - stage);
arg9 = x(13:14);
arg10 = p(122:123);
arg11 = (arg7 .* arg8);
arg12 = (arg9(:) - arg10(:));
arg13 = u.';
arg15 = (arg13 .* arg14);
arg17 = (arg15 * arg16);
arg18 = dmv.';
arg20 = (arg18 .* arg19);
arg22 = (arg20 * arg21);
obj = (((arg11 * arg12) + (arg17 * u)) + (arg22 * dmv));

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg43 = zeros(26, 1);
    xjac = zeros(14, 1);
    ujac = zeros(4, 1);
    dmvjac = zeros(4, 1);
    arg23 = 1;
    dmvjac = dmvjac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg22, [4 1], arg23);
    arg24 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], dmv, arg23);
    arg25 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg21, arg24);
    arg26 = (arg25.*arg19(:));
    dmvjac = dmvjac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg26,[1 4]);
    ujac = ujac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg17, [4 1], arg23);
    arg27 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], u, arg23);
    arg28 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg16, arg27);
    arg29 = (arg28.*arg14(:));
    ujac = ujac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg29,[1 4]);
    arg30 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg11, [2 1], arg23);
    arg31 = (-arg30);
    arg32 = zeros(123, 1);
    arg32(122:123,:) = arg31;
    arg33 = zeros(14, 1);
    arg33(13:14,:) = arg30;
    xjac = xjac + arg33;
    arg34 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 2], arg12, arg23);
    arg35 = sum((arg34.*arg7(:)),1);
    arg36 = (arg34.*arg8(:));
    arg37 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 2], arg6, arg36);
    arg38 = (arg37.*arg4(:));
    arg39 = optim.problemdef.gradients.indexing.TransposeAdjoint(arg38,[1 2]);
    arg40 = (-arg39);
    arg41 = zeros(123, 1);
    arg41(122:123,:) = arg40;
    arg42 = zeros(14, 1);
    arg42(13:14,:) = arg39;
    xjac = xjac + arg42;
    arg43(xidx,:) = xjac;
    arg43(uidx,:) = ujac;
    arg43(dmvidx,:) = dmvjac;
    grad = arg43(:);
end
end