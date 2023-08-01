% ga_main.m
clc
clear
close all
global  bound rng
global num_in num_hid num_out num_w num_b num_nn
%% Initializing parameters
pops=50;                      % population size
maxgen=200;                   % maximum generation
crossp=0.8;                   % crossover probability
mutatep=0.5;                  % mutation probability

%% initializing pop

% network1
num_in = 10;    % input layer
num_hid = 40;  % hidden layer
num_out = 2;   % output layer
num_w = num_in*num_hid + num_hid*num_out;  % weight
num_b = num_hid+num_out;  % bias
num_nn = num_w + num_b;


a = -100.0;
b = 100.0;
pop = a + (b-a).*rand(pops, num_nn);

numvar = size(pops,2);

maxf = [0]; meanf= [0];

% pop=zeros(pops,numvar);      
% pop(:,1:numvar)=(ones(pops,1)*rng).*(rand(pops,numvar))+(ones(pops,1)*bound(:,1)');
fig_ga = figure('Name',"GA processer",'Position', [100 400 1800 480]);
maxf_p = plot(maxf,'-o',Color="#ff0000",LineWidth=2); hold on; box on;
meanf_p = plot(meanf,Color="#0000ff",LineWidth=2); grid on;
xlim([1,maxgen]);
xlabel("Generation");
ylabel("Fitness Function");
fontname(fig_ga,"Times New Roman");
fontsize(fig_ga,16,"points");

drawnow
%% start generation
for it=1:maxgen
    disp("Iteration: "+it);
    % evaluate fitness for each of pop
    fpop = half_model_calculation(pop);    % fpop: fitness

    % choose pops for next generation
    [cs,inds]=max(fpop);    % pop which make fitness maximum
    max_fpop = max(fpop);
    disp("Max fitness: "+max_fpop+newline);

    bchrom=pop(inds,:);     % elite
 
    % cross pair
    toursize=5;
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

    % set(maxf_p,'XData',1:it,"YData",maxf);
    % set(meanf_p,'XData',1:it,"YData",meanf_p);
    delete(meanf_p);
    delete(maxf_p)
    maxf_p = plot(maxf,'-o',Color="#ff0000",LineWidth=2); hold on; box on;
    meanf_p = plot(meanf,Color="#0000ff",LineWidth=2); grid on;
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

save("ga_controller.mat");

% Apply controller
run("half_model_calculation_verify.m")