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
%   2024/05/05 00:11:15 に prob2struct により自動生成

%#codegen
%#internal
%% 変数のインデックス。
xidx = 1:14;
eidx = 23:24;

%% ソルバーベースの変数を問題ベースにマップします。
x = inputVariables(xidx);
x = x(:);
e = inputVariables(eidx);
e = e(:);

%% Extract parameters.
p = params.p;
stage = params.stage;

%% 目的関数を計算します。
arg4 = extraParams{1};
arg11 = extraParams{2};
arg1 = x([2 5 8 9 12 13 14]);
arg2 = p([161 164 167 168 171 172 173]);
arg3 = (arg1(:) - arg2(:)).';
arg5 = x([2 5 8 9 12 13 14]);
arg6 = p([161 164 167 168 171 172 173]);
arg7 = (arg3 * arg4);
arg8 = (arg5(:) - arg6(:));
arg9 = e.';
arg10 = (arg9 .* stage);
arg12 = (arg10 * arg11);
obj = ((arg7 * arg8) + (arg12 * e));

if nargout > 1
    %% 目的関数の gradient を計算します。
    % gradient コードを呼び出すには、SpecifyObjectiveGradient オプションを true に設定
    % してソルバーに通知します。
    arg27 = zeros(24, 1);
    xjac = zeros(14, 1);
    ejac = zeros(2, 1);
    arg13 = 1;
    ejac = ejac + optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg12, [2 1], arg13);
    arg14 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 2], e, arg13);
    arg15 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 2], arg11, arg14);
    arg16 = (arg15.*stage(:));
    ejac = ejac + optim.problemdef.gradients.indexing.TransposeAdjoint(arg16,[1 2]);
    arg17 = optim.coder.problemdef.gradients.mtimes.MtimesRightAdjoint(arg7, [7 1], arg13);
    arg18 = (-arg17);
    arg19 = zeros(173, 1);
    arg19([161 164 167 168 171 172 173],:) = arg18;
    arg20 = zeros(14, 1);
    arg20([2 5 8 9 12 13 14],:) = arg17;
    xjac = xjac + arg20;
    arg21 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 7], arg8, arg13);
    arg22 = optim.coder.problemdef.gradients.mtimes.MtimesLeftAdjoint([1 7], arg4, arg21);
    arg23 = optim.problemdef.gradients.indexing.TransposeAdjoint(arg22,[1 7]);
    arg24 = (-arg23);
    arg25 = zeros(173, 1);
    arg25([161 164 167 168 171 172 173],:) = arg24;
    arg26 = zeros(14, 1);
    arg26([2 5 8 9 12 13 14],:) = arg23;
    xjac = xjac + arg26;
    arg27(xidx,:) = xjac;
    arg27(eidx,:) = ejac;
    grad = arg27(:);
end
end