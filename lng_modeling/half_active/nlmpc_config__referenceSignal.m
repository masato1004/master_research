function ref = nlmpc_config__referenceSignal(x,u,init,Ts)
    ref = [
        x(1) + x(8)*Ts;
        init(2);
        init(3);
        init(4);
        init(5);
        x(6) + x(13)*Ts;
        x(7) + x(14)*Ts;
        init(8);
        0;
        0;
        0;
        0;
        x(13);
        x(14)
    ];
end