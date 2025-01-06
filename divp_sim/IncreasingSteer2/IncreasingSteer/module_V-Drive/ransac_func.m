%% RANSAC func
function [a,b,c,d] = ransac_func(data,k,t,r)
arguments
    data = [0 0 0];
    k = 100; % max loop num
    t = 0.075; % threshold error val for inlier
    r = 800; % requrired inlier sample num to be correnct param
end
    good_models = [];
    good_model_errors = [];
    iterations = 0;
    while iterations < k
        % random sampling
        [len, ~] = size(data);
        sample = data(randsample(len,3), :);
        [a_,b_,c_,d_] = getParamWithSamples(sample);
        param = [a_,b_,c_,d_];
        inliers = [];
        for s = 1:len
            p = data(s,:);
            % checking not to sample the other samples or through
            if ~logical(sum(p == sample))
                if getError(param, p) > t
                    continue
                else
                    inliers = [inliers; p];
                end
            end
        end
        % add model to good_model if the it fit well enough
        if length(inliers) > r
            errorlist = [];
            for s = 1:len
                p = data(s,:);
                errorlist = [errorlist; getError(param, p)];
                current_error = mean(errorlist, "all");
            end
            good_models = [good_models; param];
            good_model_errors = [good_model_errors; current_error];
        end
        iterations = iterations + 1;
    end
        
    [~, best_index] = min(good_model_errors);
    a = good_models(best_index,1);
    b = good_models(best_index,2);
    c = good_models(best_index,3);
    d = good_models(best_index,4);
end

%% functions in RANSAC func

%% detect params from 3 points
% sample: 3-3 matrix
function [a,b,c,d] =  getParamWithSamples(samples)
    p1 = samples(1,:);
    p2 = samples(2,:);
    p3 = samples(3,:);
    x1 = p1(1);
    x2 = p2(1);
    x3 = p3(1);
    y1 = p1(2);
    y2 = p2(2);
    y3 = p3(2);
    z1 = p1(3);
    z2 = p2(3);
    z3 = p3(3);
    a = (y1*z2 - y1*z3 - y2*z1 + y2*z3 + y3*z1 - y3*z2);
    b = (-x1*z2 + x1*z3 + x2*z1 - x2*z3 - x3*z1 + x3*z2);
    c = (x1*y2 - x1*y3 - x2*y1 + x2*y3 + x3*y1 - x3*y2);
    d = (-x1*y2*z3 + x1*y3*z2 + x2*y1*z3 - x2*y3*z1 - x3*y1*z2 + x2*y2*z1);
end

%% get error-average between pre and cor
function error = getError(model, p)
    x = p(1);
    y = p(2);
    z = p(3);
    error = abs(applyModel(model, x, y) - z);
end

%% calculate z with predicted model
function z = applyModel(model, x, y)
    a = model(1);
    b = model(2);
    c = model(3);
    d = model(4);
    z = -a*x/c - b*y/c - d/c;
end