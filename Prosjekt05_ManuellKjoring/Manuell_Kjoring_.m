%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Prosjekt05_ManuellKjoring
%
% Hensikten med programmet er å tune er regulator
% for styring av hastigheten til en motor
%
% Følgende motorer brukes:
%  - motor A og motor B
%--------------------------------------------------------------------------

clear; close all
online = true;
plotting = true;
filename = 'P05_Manuell_Kjoring.mat';

if online
    mylego = legoev3('USB');
    joystick = vrjoystick(1);

    motorA = motor(mylego, 'A');
    motorB = motor(mylego,'D');

    myColorSensor = colorSensor(mylego);
  
    motorA.resetRotation;
    motorB.resetRotation;
else
    load(filename)
end

fig1 = figure;
set(gcf, 'Position', [100, 100, 1200, 900]);
drawnow;

% Initialiser alle arrayer som skal vokse
Tid = []; Lys = []; VinkelPosMotorA = []; VinkelPosMotorB = []; GyroAngle = [];
u_A = []; u_B = []; r = []; y = []; e = []; P = []; I = []; D = []; u_pid = [];
IAE = []; MAE = []; TVA = []; TVB = [];

JoyMainSwitch = 0;
k = 0;
fc = 1.4;
tau = 1/(2*pi*fc);

prev_uA = 0;
prev_uB = 0;

% Kontrollparametre
base_speed = 40;    % Grunnhastighet (fra joystick)
max_speed = 50;     % Maks hastighet
steering_gain = 30; % Følsomhet for svinging
deadzone = 0.1;     % Dødsone for styrestikkeaksene

while ~JoyMainSwitch
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
    

        % Hent styrestikkeverdier
        joyY = -100 * axis(joystick, 2);  % Fram/bak (akse 2)
        joyX = 100 * axis(joystick, 3);   % Sving (akse 3)

        [~, JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1);

       

    else
        if k == length(Tid)
            JoyMainSwitch = 1;
        end
        if plotting
            pause(0.03)
        end
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %       PID REGULATOR OG BEREGNINGER

    u0 = 0;
    Kp = 0.1;
    Ki = 0.05;
    Kd = 0.01;
    I_max = 100;
    I_min = -100;

    % Lysmåling og referanse
    if k == 1
        T_s(k) = 0.05;
        r(k) = Lys(1);  % Initial lysverdi som referanse
        y(k) = Lys(k);  % Bruk lysverdi direkte
        e(k) = r(k) - y(k);
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
        T_s(k) = Tid(k) - Tid(k-1);
        r(k) = r(1);  % Behold samme referanse
        y(k) = Lys(k);  % Bruk lysverdi direkte
        e(k) = r(k) - y(k);


        % Bruk din MinPID-funksjon
        para = [Kp, Ki, Kd, I_max, I_min, 0.3]; % 0.3 er filterverdi
        [P(k), I(k), D(k), e_f(k)] = MinPID(I(k-1), e_f(k-1), [e(k-1), e(k)], T_s(k), para);

        IAE(k) = IAE(k-1) + abs(e(k)) * T_s(k);
        MAE(k) = (MAE(k-1)*(k-1) + abs(e(k))) / k;
        % Lavpassfiltrering
        
    end
   
    u_pid(k) = u0 + P(k) + I(k) + D(k); % Lagre PID-utgang i array

    
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % KOMBINER PID OG JOYSTICK INNPUT
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    % Anvend dødsone på styrestikkeaksene
    if abs(joyY) < deadzone
        joyY = 0;
    end
    if abs(joyX) < deadzone
        joyX = 0;
    end

    % Beregn hastighet og sving
    speed_cmd = joyY/100 * base_speed;
    turn_cmd = joyX/100 * steering_gain;

    % Beregn motorpådrag
    u_A(k) = speed_cmd + turn_cmd - u_pid(k);
    u_B(k) = speed_cmd - turn_cmd + u_pid(k);

    % Begrens hastigheter
    u_A(k) = max(min(u_A(k), 100), -100);
    u_B(k) = max(min(u_B(k), 100), -100);

    % Beregn total variasjon
    if k > 1
        TVA(k) = TVA(k-1) + abs(u_A(k) - prev_uA);
        TVB(k) = TVB(k-1) + abs(u_B(k) - prev_uB);
        

    else
        TVA(k) = 0;
        TVB(k) = 0;
    end
    prev_uA = u_A(k);
    prev_uB = u_B(k);

    if online
        motorA.Speed = u_A(k);
        motorB.Speed = u_B(k);
        start(motorA);
        start(motorB);
    end

    if plotting || JoyMainSwitch
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
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                       LEGG TIL LEGENDER ETTER PLOTTING

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



% Stopp motorer ved avslutning
if online
    stop(motorA);
    stop(motorB);
    %save(filename, 'Tid', 'Lys', 'VinkelPosMotorA', 'VinkelPosMotorB', ...
       % 'r', 'y', 'e', 'u_A', 'u_B', 'IAE', 'MAE', 'TVA', 'TVB');
end



% At the end of the file, after the main while loop and motor stop commands

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