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
%   2024/05/06 03:37:37 に prob2struct により自動生成

%#codegen
%#internal
%% 変数のインデックス。
xidx = 1:14;
eidx = 23:30;
uidx = 15:18;

%% ソルバーベースの変数を問題ベースにマップします。
x = inputVariables(xidx);
x = x(:);
e = inputVariables(eidx);
e = e(:);
u = inputVariables(uidx);
u = u(:);

%% Extract parameters.
p = params.p;

%% 目的関数を計算します。
arg4 = extraParams{1};
arg10 = extraParams{2};
arg13 = extraParams{3};
arg1 = x([2 5 8 9 12 13 14]);
arg2 = p([136 139 142 143 146 147 148]);
arg3 = (arg1(:) - arg2(:)).';
arg5 = x([2 5 8 9 12 13 14]);
arg6 = p([136 139 142 143 146 147 148]);
arg7 = (arg3 * arg4);
arg8 = (arg5(:) - arg6(:));
arg9 = e.';
arg11 = (arg9 * arg10);
arg12 = u.';
arg14 = (arg12 * arg13);
obj = (((arg7 * arg8) + (arg11 * e)) + (arg14 * u));

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg30 = zeros(30, 1);
    xjac = zeros(14, 1);
    ejac = zeros(8, 1);
    ujac = zeros(4, 1);
    arg15 = 1;
    ujac = ujac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg14, [4 1], arg15);
    arg16 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], u, arg15);
    arg17 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 4], arg13, arg16);
    ujac = ujac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg17,[1 4]);
    ejac = ejac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg11, [8 1], arg15);
    arg18 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 8], e, arg15);
    arg19 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 8], arg10, arg18);
    ejac = ejac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg19,[1 8]);
    arg20 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg7, [7 1], arg15);
    arg21 = (-arg20);
    arg22 = zeros(148, 1);
    arg22([136 139 142 143 146 147 148],:) = arg21;
    arg23 = zeros(14, 1);
    arg23([2 5 8 9 12 13 14],:) = arg20;
    xjac = xjac + arg23;
    arg24 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 7], arg8, arg15);
    arg25 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 7], arg4, arg24);
    arg26 = optim.problemdef.gradients.indexing.TransposeAdjoint(arg25,[1 7]);
    arg27 = (-arg26);
    arg28 = zeros(148, 1);
    arg28([136 139 142 143 146 147 148],:) = arg27;
    arg29 = zeros(14, 1);
    arg29([2 5 8 9 12 13 14],:) = arg26;
    xjac = xjac + arg29;
    arg30(xidx,:) = xjac;
    arg30(eidx,:) = ejac;
    arg30(uidx,:) = ujac;
    grad = arg30(:);
end
end