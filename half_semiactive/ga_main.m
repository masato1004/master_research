% ga_main.m
clc
clear
close all
global nowf_p nowf f_line inds
global num_in num_hid num_out num_w1 num_w2 num_b num_nn num_hid1 num_hid2
%% Initializing parameters
pops=500;                      % population size
maxgen=500;                   % maximum generation
crossp=0.8;                   % crossover probability
mutatep=0.35;                  % mutation probability

%% initializing pop

% network1
num_in = 12;    % input layer
num_hid1 = 40;
num_hid2 = 40;
num_hid = num_hid1 + num_hid2;  % hidden layer
num_out = 2;   % output layer
num_w1 = num_in*num_hid1 + num_hid1*num_hid2;  % weight
num_w2 = num_in*num_hid1 + num_hid1*num_hid2 + num_hid2*num_out;  % weight
num_b = num_hid*0;  % bias
num_nn = num_w2 + num_b;


a = -250.0;
b = 250.0;
pop = a + (b-a).*rand(pops, num_nn);

numvar = size(pops,2);

maxf = ones(1,maxgen)*nan; meanf= ones(1,maxgen)*nan;
nowf = ones(1,maxgen*pops)*nan; inds = nan;

% pop=zeros(pops,numvar);      
% pop(:,1:numvar)=(ones(pops,1)*rng).*(rand(pops,numvar))+(ones(pops,1)*bound(:,1)');
fig_ga = figure('Name',"GA processer",'Position', [100 400 1800 600]);
maxf_p = plot(maxf,'-o',Color="#ff0000",LineWidth=2); hold on; box on;
nowf_p = plot(nowf,Color="#ccccff",LineWidth=0.7); grid on;
meanf_p = plot(meanf,Color="#0000ff",LineWidth=2); grid on;
f_line = yline(nan,'-',"Current Fitness",DisplayName="");

xlim([0,maxgen]);
xlabel("Generation");
ylabel("Fitness Function");
fontname(fig_ga,"Times New Roman");
fontsize(fig_ga,16,"points");

drawnow
%% start generation
for it=1:maxgen
    disp("Iteration: "+it);
    % evaluate fitness for each of pop
    fpop = half_model_calculation(pop,maxgen,it);    % fpop: fitness

    % choose pops for next generation
    [cs,inds]=max(fpop);    % pop which make fitness maximum
    max_fpop = max(fpop);
    disp("Max fitness: "+max_fpop+newline);

    bchrom=pop(inds,:);     % elite
 
    % cross pair
    toursize=pops/2;
    players=ceil(pops*rand(pops,toursize));

    scores=fpop(players);

    [a,m]=max(scores');

    pind=zeros(1,pops);

    for i=1:pops
        pind(i)=players(i,m(i));
        parent(i,:)=pop(pind(i),:);
    end
    
    % Crossover
    child=cross(parent,crossp); 

    % Mutation
    pop=mutate(child,mutatep);
  
    % parameters for drawing
    % mm=multipeak(pop);
    maxf(it)=max_fpop; 
    meanf(it)=mean(fpop);
    
    [bfit,bind]=max(fpop);
    bsol=pop(bind,:);

    set(maxf_p,"YData",maxf);
    set(meanf_p,"YData",meanf);
    % delete(meanf_p);
    % delete(maxf_p)
    % maxf_p = plot(maxf,'-o',Color="#ff0000",LineWidth=2); hold on; box on;
    % meanf_p = plot(meanf,Color="#0000ff",LineWidth=2); grid on;
    drawnow

    % save elite
    pop(inds,:)=bchrom; % generate next pop with elite

end

% draw results

figure, plot(maxf), hold on, plot(meanf,'r-');
xlabel('generation')
ylabel('fitness')
title('fitness preogress')
legend("maximum fintness","meanftness")

save("ga_controller_b.mat", "num_in", "num_hid", "num_out", "num_w1", "num_w2", "num_b", "num_nn", "num_hid1", "num_hid2", "bchrom");

% Apply controller
% run("half_model_calculation_verify.m")