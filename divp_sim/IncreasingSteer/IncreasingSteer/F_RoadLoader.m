function [zfl,zfr,zrl,zrr] = F_RoadLoader(FL,FR,RL,RR)
    global pcdmap
    [k,~] = dsearchn(pcdmap(:,1:2),[FL;FR;RL;RR]);
    zfl = pcdmap(k(1),3);
    zfr = pcdmap(k(2),3);
    zrl = pcdmap(k(3),3);
    zrr = pcdmap(k(4),3);
end