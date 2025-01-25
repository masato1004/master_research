function y = F_pdf(x, mu, sigma, sigmoid_bool)
    % F_pdf computes the probability density function of a normal distribution
    % x     - the input value(s)
    % mu    - the mean of the distribution
    % sigma - the standard deviation of the distribution
    
    % Ensure sigma is positive
    if sigma <= 0
        error('Standard deviation must be positive');
    end
    
    % Compute the PDF
    y = (1 / (sigma * sqrt(2 * pi))) * exp(-0.5 * ((x-mu) / sigma).^2);

    % Apply a sigmoid function to the output
    if sigmoid_bool
        y = y.*mysigmoid(x-mu);
    end
end

function mysigmoid = mysigmoid(x)
    mysigmoid = 1 - 1 ./ (1 + exp(-30*(x)));
end