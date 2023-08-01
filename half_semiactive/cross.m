%% 学籍番号：82317323
%% 氏名：井上将利

function off=cross(parent,croprop)
[pops,numvar]=size(parent);

for i=1:2:pops
    if croprop > rand
        r = randi([1 numvar],1,1);
        offs(i,1:r)         = parent(i,1:r);
        offs(i,r+1:numvar)  = parent(i+1,r+1:numvar);
        offs(i+1,1:r)       = parent(i+1,1:r);
        offs(i+1,r+1:numvar)= parent(i,r+1:numvar);
    else
        offs(i,:)  = parent(i,:);
        offs(i+1,:)= parent(i+1,:);
    end
end
off=offs;