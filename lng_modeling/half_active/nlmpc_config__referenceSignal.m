function ref = nlmpc_config__referenceSignal(x,u,dx_init,Ts)
    ref = [
        x(1) + x(8)*Ts;
        0;
        0;
        0;
        0;
        x(6) + x(13)*Ts;
        x(7) + x(14)*Ts;
        dx_init;
        0;
        0;
        0;
        0;
        x(13);
        x(14)
    ];
end