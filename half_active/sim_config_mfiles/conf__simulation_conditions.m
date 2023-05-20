%% Dfine simulation condition with boolean
% draw an animation or not
animation = false;
prev_anim = false;

% add noise or not
high_freq_noise = false;
low_freq_noise  = false;

% control method
passive = false;
LQR     = false;
rprev   = false;
LQR_rprev       = false;
fprev_rprev     = false;
LQR_fprev_rprev = true;

% road profile
sensing = true;
paper   = false;
sin_wave = false;
step    = false;
manhole = false;
jari    = false;

%% simulation parameter
T   = 10;          % [s]       total simulation time
dt  = 0.0001;     % [s]       delta time
tc  = 0.001;      % control cycle
fs  = 50;         % sampling frequence
ts  = 1/fs;       % sampling cycle
cc  = 1;          % control cycle counter
sc  = 1;          % sampling cycle counter
sd  = 0.025/3     % standard deviation of additional noise


%% conbine settings
noise_logi = [high_freq_noise low_freq_noise];
noise_name = ["_high_freq_noise_" "_low_freq_noise_"];
if width(noise_name(noise_logi)) == 2
    added_noise = "_both_"
elseif width(noise_name(noise_logi)) == 0
    added_noise = "";
else
    added_noise= noise_name(noise_logi)
end

ctrl_names = ["_passive_","_LQR_","_rprev_","_LQR_rprev_","_fprev_rprev_","_LQR_fprev_rprev_"];
logi_ctrl = [passive, LQR, rprev, LQR_rprev, fprev_rprev, LQR_fprev_rprev];
control = ctrl_names(logi_ctrl)

shape_names = ["_sensing2_","_paper_","_sin_","_step_","_manhole_","_jari_"];
logi_shape = [sensing, paper, sin_wave, step, manhole, jari];
shape = shape_names(logi_shape)