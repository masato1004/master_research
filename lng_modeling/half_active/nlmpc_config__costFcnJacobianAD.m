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
%   2024/06/05 18:58:36 に prob2struct により自動生成

%#codegen
%#internal
%% 変数のインデックス。
xidx = 1:14;
uidx = 15:18;
eidx = 23:26;
dmvidx = 19:22;

%% ソルバーベースの変数を問題ベースにマップします。
x = inputVariables(xidx);
x = x(:);
u = inputVariables(uidx);
u = u(:);
e = inputVariables(eidx);
e = e(:);
dmv = inputVariables(dmvidx);
dmv = dmv(:);

%% Extract parameters.
p = params.p;

%% 目的関数を計算します。
arg3 = extraParams{1};
arg5 = extraParams{2};
arg10 = extraParams{3};
arg12 = extraParams{4};
arg15 = extraParams{5};
arg17 = extraParams{6};
arg20 = extraParams{7};
arg22 = extraParams{8};
arg1 = p(110:123);
arg2 = (x - arg1(:)).';
arg4 = (arg2 .* arg3);
arg6 = p(110:123);
arg7 = (arg4 * arg5);
arg8 = (x - arg6(:));
arg9 = u.';
arg11 = (arg9 .* arg10);
arg13 = (arg11 * arg12);
arg14 = e.';
arg16 = (arg14 .* arg15);
arg18 = (arg16 * arg17);
arg19 = dmv.';
arg21 = (arg19 .* arg20);
arg23 = (arg21 * arg22);
obj = ((((arg7 * arg8) + (arg13 * u)) + (arg18 * e)) + (arg23 * dmv));

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg43 = zeros(26, 1);
    xjac = zeros(14, 1);
    ujac = zeros(4, 1);
    ejac = zeros(4, 1);
    dmvjac = zeros(4, 1);
    arg24 = 1;
    dmvjac = dmvjac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg23, [4 1], arg24);
    arg25 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], dmv, arg24);
    arg26 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg22, arg25);
    arg27 = (arg26.*arg20(:));
    dmvjac = dmvjac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg27,[1 4]);
    ejac = ejac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg18, [4 1], arg24);
    arg28 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], e, arg24);
    arg29 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg17, arg28);
    arg30 = (arg29.*arg15(:));
    ejac = ejac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg30,[1 4]);
    ujac = ujac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg13, [4 1], arg24);
    arg31 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], u, arg24);
    arg32 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg12, arg31);
    arg33 = (arg32.*arg10(:));
    ujac = ujac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg33,[1 4]);
    arg34 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg7, [14 1], arg24);
    xjac = xjac + arg34;
    arg35 = (-arg34);
    arg36 = zeros(123, 1);
    arg36(110:123,:) = arg35;
    arg37 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 14], arg8, arg24);
    arg38 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 14], arg5, arg37);
    arg39 = (arg38.*arg3(:));
    arg40 = optim.problemdef.gradients.indexing.TransposeAdjoint(arg39,[1 14]);
    xjac = xjac + arg40;
    arg41 = (-arg40);
    arg42 = zeros(123, 1);
    arg42(110:123,:) = arg41;
    arg43(xidx,:) = xjac;
    arg43(uidx,:) = ujac;
    arg43(eidx,:) = ejac;
    arg43(dmvidx,:) = dmvjac;
    grad = arg43(:);
end
end