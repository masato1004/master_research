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
%   2024/06/30 02:26:16 に prob2struct により自動生成

%#codegen
%#internal
%% 変数のインデックス。
xidx = 1:14;
uidx = 15:18;
dmvidx = 19:22;
eidx = 23:24;

%% ソルバーベースの変数を問題ベースにマップします。
x = inputVariables(xidx);
x = x(:);
u = inputVariables(uidx);
u = u(:);
dmv = inputVariables(dmvidx);
dmv = dmv(:);
e = inputVariables(eidx);
e = e(:);

%% Extract parameters.
p = params.p;

%% 目的関数を計算します。
arg4 = extraParams{1};
arg6 = extraParams{2};
arg12 = extraParams{3};
arg14 = extraParams{4};
arg17 = extraParams{5};
arg19 = extraParams{6};
arg22 = extraParams{7};
arg24 = extraParams{8};
arg1 = x(13:14);
arg2 = p(122:123);
arg3 = (arg1(:) - arg2(:)).';
arg5 = (arg3 .* arg4);
arg7 = x(13:14);
arg8 = p(122:123);
arg9 = (arg5 * arg6);
arg10 = (arg7(:) - arg8(:));
arg11 = u.';
arg13 = (arg11 .* arg12);
arg15 = (arg13 * arg14);
arg16 = dmv.';
arg18 = (arg16 .* arg17);
arg20 = (arg18 * arg19);
arg21 = e.';
arg23 = (arg21 .* arg22);
arg25 = (arg23 * arg24);
obj = ((((arg9 * arg10) + (arg15 * u)) + (arg20 * dmv)) + (arg25 * e));

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg47 = zeros(24, 1);
    xjac = zeros(14, 1);
    ujac = zeros(4, 1);
    dmvjac = zeros(4, 1);
    ejac = zeros(2, 1);
    arg26 = 1;
    ejac = ejac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg25, [2 1], arg26);
    arg27 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 2], e, arg26);
    arg28 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 2], arg24, arg27);
    arg29 = (arg28.*arg22(:));
    ejac = ejac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg29,[1 2]);
    dmvjac = dmvjac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg20, [4 1], arg26);
    arg30 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], dmv, arg26);
    arg31 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg19, arg30);
    arg32 = (arg31.*arg17(:));
    dmvjac = dmvjac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg32,[1 4]);
    ujac = ujac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg15, [4 1], arg26);
    arg33 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], u, arg26);
    arg34 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg14, arg33);
    arg35 = (arg34.*arg12(:));
    ujac = ujac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg35,[1 4]);
    arg36 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg9, [2 1], arg26);
    arg37 = (-arg36);
    arg38 = zeros(123, 1);
    arg38(122:123,:) = arg37;
    arg39 = zeros(14, 1);
    arg39(13:14,:) = arg36;
    xjac = xjac + arg39;
    arg40 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 2], arg10, arg26);
    arg41 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 2], arg6, arg40);
    arg42 = (arg41.*arg4(:));
    arg43 = optim.problemdef.gradients.indexing.TransposeAdjoint(arg42,[1 2]);
    arg44 = (-arg43);
    arg45 = zeros(123, 1);
    arg45(122:123,:) = arg44;
    arg46 = zeros(14, 1);
    arg46(13:14,:) = arg43;
    xjac = xjac + arg46;
    arg47(xidx,:) = xjac;
    arg47(uidx,:) = ujac;
    arg47(dmvidx,:) = dmvjac;
    arg47(eidx,:) = ejac;
    grad = arg47(:);
end
end