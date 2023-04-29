%% low passed road profile
T = 10;
fs = 1/tc;
X = double(wf_grad(2,1:end-width(wf_grad(2,isnan(wf_grad(2,:))))));
% X = wf_grad(2,wf_grad(1,:)+10-ts>0.3665 & wf_grad(1,:)+10-ts<0.825);
L = width(X);
t = [0:(L-1)]*tc;
r_fig = figure('Position', [600 200 600 190]);
plot(TL,r_p_f,"LineWidth",2,"Color","#0000ff");
ylabel("Displacement [m]");
xlabel("Time [s]");
xlim([0,3]);
% ylim([-0.01,0.1])
% ylim([-0.03,0.04])
fontname(r_fig,"Times New Roman");
fontsize(r_fig,10.5,"points");
grid on;
hold on;
% plot(wf_global(1,1:end-width(wf_global(2,isnan(wf_global(2,:)))))./V+T,wf_global(2,1:end-width(wf_global(2,isnan(wf_global(2,:))))),"LineWidth",1,"Color","#ff0000");
plot(wf_grad(1,1:end-width(wf_grad(2,isnan(wf_grad(2,:)))))+T,X,"LineWidth",2,"Color","#ff0000");
legend("True Value", "Previewed Displacement")

% y = fft(X);
% f = (0:length(y)-1)*fs/length(y);
% plot(f,abs(y))
% xlabel('Frequency (Hz)')
% ylabel('Magnitude')
% title('Magnitude')

Y = fft(X);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = fs*(0:(L/2))/L;
fft_fig = figure('Position', [600 200 600 190]);
plot(f,P1,"LineWidth",2,"Color","#0000ff") 
% title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f [Hz]")
ylabel("|P(f)|")
fontname(fft_fig,"Times New Roman");
fontsize(fft_fig,10.5,"points");
grid on;


filtered = lowpass(X,0.1,fs);
figure;
plot(t,filtered)



% wf_grad(2,1:end-width(wf_grad(2,isnan(wf_grad(2,:))))+1)
%% zero-phase filtering
d1 = designfilt("lowpassiir",FilterOrder=3, ...
    HalfPowerFrequency=0.004,DesignMethod="butter",SampleRate=1);
y = filtfilt(d1,X);

f_fig = figure('name',"Road-profile: Frequency "+frequency+" Hz",'Position', [600 200 600 190]);
plot(wf_grad(1,1:end-width(wf_grad(2,isnan(wf_grad(2,:)))))+10-ts,X,Color="#0000ff")
% plot(wf_grad(1,wf_grad(1,:)+10-ts>0.3665 & wf_grad(1,:)+10-ts<0.825)+10-ts,X,Color="#0000ff")
hold on
plot(wf_grad(1,1:end-width(wf_grad(2,isnan(wf_grad(2,:)))))+10-ts,y,LineWidth=2,Color="#ff0000")
% plot(wf_grad(1,wf_grad(1,:)+10-ts>0.3665 & wf_grad(1,:)+10-ts<0.825)+10-ts,y,LineWidth=2,Color="#ff0000")
xlim([0.2,1]);
legend("Previewed Displacement","Zero-Phase Filtering")
xlabel("Time [s]");
grid on;
ylabel("Displacement [m]")
fontname(f_fig,"Times New Roman");
fontsize(f_fig,10.5,"points");