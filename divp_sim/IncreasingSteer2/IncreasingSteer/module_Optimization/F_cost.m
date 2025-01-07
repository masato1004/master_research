function cost = F_cost(position, left_wheel, right_wheel)
    % F_cost - Calculate the cost based on position and wheel velocities
    %
    % Syntax: cost = F_cost(position, left_wheel, right_wheel)
    %
    % Inputs:
    %    position - [x, y] position of the robot
    %    left_wheel - velocity of the left wheel
    %    right_wheel - velocity of the right wheel
    %
    % Outputs:
    %    cost - calculated cost

    % Example cost function: sum of squared errors
    % You can modify this to suit your specific needs

    % Desired position (for example, the target position)
    desired_position = [0, 0];

    % Calculate the position error
    position_error = norm(position - desired_position);

    % Calculate the wheel velocity error (assuming desired velocity is 0)
    wheel_velocity_error = left_wheel^2 + right_wheel^2;

    % Combine the errors to form the cost
    cost = position_error + wheel_velocity_error;
end