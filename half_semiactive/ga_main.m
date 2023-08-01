% ga_main.m
clc
clear
close all
global  bound rng
global num_in num_hid num_out num_w num_b num_nn
%% Initializing parameters
pops=20;                      % population size
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


a = -10.0;
b = 10.0;
pop = a + (b-a).*rand(pops, num_nn);

numvar = size(pops,2);

maxf = []; meanf= [];

% pop=zeros(pops,numvar);      
% pop(:,1:numvar)=(ones(pops,1)*rng).*(rand(pops,numvar))+(ones(pops,1)*bound(:,1)');
figure(1);
xlim([1,maxgen]);
xlabel("Generation");
ylabel("Fitness Function")

maxf_p = plot(maxf,Color="#ff0000",LineWidth=2);
meanf_p = plot(meanf_p,Color="#0000ff",LineWidth=2);
%% start generation
for it=1:maxgen
    it
    % evaluate fitness for each of pop
    fpop = half_model_calculation(pop);    % fpop: fitness

    % choose pops for next generation
    [cs,inds]=max(fpop);    % pop which make fitness maximum
    max_fpop = max(fpop)

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
    maxf(it)=max(fpop); 
    meanf(it)=mean(fpop);
    
    [bfit,bind]=max(fpop);
    bsol=pop(bind,:);

    set(maxf_p,'XData',1:it,"YData",maxf);
    set(meanf_p,'XData',1:it,"YData",meanf_p);

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