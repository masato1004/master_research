%% 学籍番号：82317323
%% 氏名：井上将利

% mutate.m

function d=mutate(offs,mutprop)
global  bound rng
[pops,numvar]=size(offs);
mut=round(mutprop*pops*numvar); % 突然変異を行う回数

a = -50.0;
b = 50.0;
for i=1:mut
    x=ceil(rand*pops);      % どの個体
    y=ceil(rand*numvar);    % どの遺伝子

    offs(x,y) = a + (b-a).*rand(1, 1);
%     offs(x,y)=randi(1000);
%     offs(x,y)=bound(y,1)+rand*rng(y);
end
d=offs;
