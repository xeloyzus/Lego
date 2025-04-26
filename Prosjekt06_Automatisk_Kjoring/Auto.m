% Prosjekt05_LinjeFølging - Enhanced Version
%
% Improved PID controller for line following with better corner handling
%--------------------------------------------------------------------------

clear; close all
online = true;
plotting = true;
filename = 'P05_Automatisk_Kjoring.mat';
calibrated = false;

if online
    mylego = legoev3('USB');
    motorA = motor(mylego, 'A');
    motorB = motor(mylego,'D');
    myColorSensor = colorSensor(mylego);
    joystick = vrjoystick(1);
    motorA.resetRotation;
    motorB.resetRotation;
else
    load(filename)
end

fig1 = figure;
set(gcf, 'Position', [100, 100, 800, 600]);
drawnow;

% Kalibreringsvariabler
white = 24;
black = 4;
midpoint = 0;
MainSwitch = 0;
k = 0;

% Base PID parameters - will adjust dynamically
base_Kp = 1.5;      % Increased from original
base_Ki = 0.02;     % Reduced from original to reduce overshoot
base_Kd = 0.5;      % Increased from original for better cornering
base_speed = 15;

% Dynamic tuning parameters
max_error = 70;     % Maximum expected error value
aggressive_factor = 2.5; % How much to boost P in corners
corner_threshold = 45;  % Error value that indicates a corner

% Filter and limiting parameters
filter_coeff = 0.5;  % Smoother filtering

% Reduced integrator windup
I_max = 45;        
I_min = -45;
max_accel = 8;      % Maximum speed change per cycle (smoother motion)

% State variables
I_prev = 0;
e_f_prev = 0;
prev_uA = 0;
prev_uB = 0;
prev_speed = base_speed;
corner_flag = false;
corner_counter = 0;

% Kalibreringsrutine
if ~calibrated
    midpoint = max(0, min(100, (white + black) / 2));
    calibrated = true;
end

while ~MainSwitch
    k = k + 1;

    if online
        if k == 1
            tic
            Tid(k) = 0;
        else
            Tid(k) = toc;
        end

        Lys(k) = double(readLightIntensity(myColorSensor, 'reflected'));
        VinkelPosMotorA(k) = double(motorA.readRotation);
        VinkelPosMotorB(k) = double(motorB.readRotation);
       
        [~, JoyButtons] = HentJoystickVerdier(joystick);
        MainSwitch = JoyButtons(1);
    else
        if k >= length(Tid)
            MainSwitch = 1;
        end
        if plotting
            pause(0.03)
        end
    end

    if k == 1
        T_s(k) = 0.05;
        r(k) = Lys(1);
        y(k) = Lys(k);
        e(k) = y(k)-r(k);
        e_f(k) = e(k);
        IAE(k) = abs(e(k)) * T_s(k);
        MAE(k) = abs(e(k));
        TVA(k) = 0;
        TVB(k) = 0;
        I(k) = 0;
        P(k) = 0;
        D(k) = 0;
        u_pid(k) = 0;
    else
        % Sample time calculation
        T_s(k) = Tid(k) - Tid(k-1);
        if T_s(k) <= 0
            T_s(k) = 0.05; % Default if time calculation fails
        end
        
        % Reference and error
        r(k) = Lys(1);
        y(k) = Lys(k);
        e(k) = y(k)-r(k);
        
        % Dynamic PID tuning based on error
        error_ratio = min(abs(e(k))/max_error, 1);
        
        % Boost P term in corners
        if abs(e(k)) > corner_threshold
            if ~corner_flag
                corner_flag = true;
                corner_counter = 0;
            end
            corner_counter = corner_counter + 1;
            Kp = base_Kp * aggressive_factor;
            Ki = base_Ki * 0.5;  % Reduce I during corners
            Kd = base_Kd * 1.2;   % Increase D during corners
        else
            if corner_flag && corner_counter > 0
                corner_counter = corner_counter - 1;
                if corner_counter <= 0
                    corner_flag = false;
                end
            end
            Kp = base_Kp * (1 + 0.5*error_ratio);  % Gradual increase
            Ki = base_Ki;
            Kd = base_Kd;
        end
        
        % PID calculation
        para = [Kp, Ki, Kd, I_max, I_min, filter_coeff];
        [P(k), I(k), D(k), e_f(k)] = MinPID(I_prev, e_f_prev, [e(k-1), e(k)], T_s(k), para);
        IAE(k) = IAE(k-1) + abs(e(k)) * T_s(k);
        MAE(k) = (MAE(k-1)*(k-1) + abs(e(k))) / k;
    end

    u_pid(k) = P(k) + I(k) + D(k);
    
    % Dynamic speed adjustment
    speed_factor = max(0.4, 1 - (abs(e(k))/max_error)^1.5);
    target_speed = base_speed * speed_factor;
    
    % Rate limit speed changes
    speed_change = target_speed - prev_speed;
    if abs(speed_change) > max_accel
        target_speed = prev_speed + sign(speed_change)*max_accel;
    end
    prev_speed = target_speed;
    
    % Motor control with anti-windup
    u_A(k) = target_speed + u_pid(k);
    u_B(k) = target_speed - u_pid(k);
    
    % Clamp motor speeds
    u_A(k) = max(min(u_A(k), 100), -100);
    u_B(k) = max(min(u_B(k), 100), -100);
    
    % Store previous values
    if k > 1
        TVA(k) = TVA(k-1) + abs(u_A(k) - prev_uA);
        TVB(k) = TVB(k-1) + abs(u_B(k) - prev_uB);
    else
        TVA(k) = 0;
        TVB(k) = 0;
    end
    prev_uA = u_A(k);
    prev_uB = u_B(k);
    I_prev = I(k);
    e_f_prev = e_f(k);

    if online
        motorA.Speed = u_A(k);
        motorB.Speed = u_B(k);
        start(motorA);
        start(motorB);
    end

    % Plotting (same as before)
    if plotting || MainSwitch
               figure(fig1)

        subplot(3, 2, 1)
        plot(Tid(1:k), r(1:k), 'r-', 'DisplayName', '$r_k$');
        hold on
        plot(Tid(1:k), y(1:k), 'b-', 'DisplayName', '$y_k$');
        hold off
        grid on
        title('Lysmåling og Referanse', 'Interpreter', 'latex')

        subplot(3, 2, 2)
        plot(Tid(1:k), e(1:k), 'b-', 'DisplayName', '$e_k$');
        grid on
        title('Reguleringsavvik','Interpreter', 'latex')

        subplot(3, 2, 3)
        plot(Tid(1:k), u_A(1:k), 'b-', 'DisplayName', '$u_{A,k}$');
        hold on
        plot(Tid(1:k), u_B(1:k), 'r-', 'DisplayName', '$u_{B,k}$');
        hold off
        grid on
        title('Pådrag motor A og B', 'Interpreter', 'latex')

        subplot(3, 2, 4)
        plot(Tid(1:k), IAE(1:k), 'b-', 'DisplayName', '$IAE_k$');
        grid on
        title('Integral of Absolute Error (IAE)', 'Interpreter', 'latex')

        subplot(3, 2, 5)
        plot(Tid(1:k), TVA(1:k), 'b-', 'DisplayName', '$TV_{A,k}$');
        hold on
        plot(Tid(1:k), TVB(1:k), 'r-', 'DisplayName', '$TV_{B,k}$');
        hold off
        grid on
        title('Total Variation', 'Interpreter', 'latex')

        subplot(3, 2, 6)
        plot(Tid(1:k), MAE(1:k), 'b-', 'DisplayName', '$MAE_k$');
        grid on
        title('Mean Absolute Error (MAE)', 'Interpreter', 'latex')

        drawnow
    end
end

figure(fig1)

subplot(3, 2, 1)
legend('$r_k$', '$y_k$', 'Interpreter', 'latex', 'Orientation', 'horizontal', ...
    'Location', 'northeast')

subplot(3, 2, 2)
legend('$e_k$', 'Interpreter', 'latex', 'Orientation', 'horizontal', ...
    'Location', 'northeast')

subplot(3, 2, 3)
legend('$u_{A,k}$', '$u_{B,k}$', 'Interpreter', 'latex', 'Orientation', 'horizontal', ...
    'Location', 'northeast')

subplot(3, 2, 4)
legend('$IAE_k$', 'Interpreter', 'latex', 'Orientation', 'horizontal', ...
    'Location', 'northeast')

subplot(3, 2, 5)
legend('$TV_{A,k}$', '$TV_{B,k}$', 'Interpreter', 'latex', 'Orientation', 'horizontal', ...
    'Location', 'northeast')

subplot(3, 2, 6)
legend('$MAE_k$', 'Interpreter', 'latex', 'Orientation', 'horizontal', ...
    'Location', 'northeast')

% Cleanup (same as before)
if online
    stop(motorA);
    stop(motorB);
    save(filename, 'Tid', 'Lys', 'VinkelPosMotorA', 'VinkelPosMotorB', ...
        'r', 'y', 'e', 'u_A', 'u_B', 'IAE', 'MAE', 'TVA', 'TVB');
end

% Calculate quality metrics
y_mean = mean(y);        % Mean of light measurements
y_std = std(y);          % Standard deviation
y_median = median(y);    % Additional useful metric

% Display the results in command window
fprintf('\n=== Quality Metrics ===\n');
fprintf('Mean light value (ȳ): %.2f\n', y_mean);
fprintf('Standard deviation (σ): %.2f\n', y_std);
fprintf('Median light value: %.2f\n', y_median);
fprintf('Number of samples: %d\n', length(y));
fprintf('Reference value (rk): %.2f\n', r(1));

% Create histogram figure
fig2 = figure;
set(gcf, 'Position', [100, 100, 800, 600]);
histogram(y, 'BinWidth', 2, 'FaceColor', [0.2 0.4 0.8], 'EdgeColor', 'none');
hold on;

% Add mean and std deviation lines
xline(y_mean, 'r-', 'LineWidth', 2, 'DisplayName', sprintf('Mean (ȳ = %.1f)', y_mean));
xline([y_mean-y_std, y_mean+y_std], 'g--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('±1σ (%.1f to %.1f)', y_mean-y_std, y_mean+y_std));

% Add reference line if it exists
if exist('r', 'var') && ~isempty(r)
    xline(r(1), 'k:', 'LineWidth', 2, 'DisplayName', sprintf('Reference (r = %.1f)', r(1)));
end

% Format the plot
grid on;
xlabel('Light Sensor Value');
ylabel('Frequency');
title('Distribution of Light Measurements {yk}');
legend('show', 'Location', 'northwest');

% Add text box with statistics
stats_text = {
    sprintf('Mean (ȳ) = %.2f', y_mean),
    sprintf('Std Dev (σ) = %.2f', y_std),
    sprintf('Median = %.2f', y_median),
    sprintf('Samples = %d', length(y))
    };
annotation('textbox', [0.15, 0.7, 0.2, 0.15], 'String', stats_text, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'none');