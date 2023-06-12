% use prev_profile
V = 50*1000/3600; % m/s

% trend = (prev_profile(2,end)-prev_profile(2,1))/((prev_profile(1,end) - prev_profile(1,1))/V);

ts = 0.001;
fs = 1/ts;
T = (prev_profile(1,end) - prev_profile(1,1))/V;
t = 0:ts:T;
L = width(t);
X = prev_profile(2,:);
X = interp1((prev_profile(1,:)-prev_profile(1,1))./V,X,t,"linear");
Y = fft(X);


P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = fs*(0:(L/2))/L;
plot(f,P1) 
title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")