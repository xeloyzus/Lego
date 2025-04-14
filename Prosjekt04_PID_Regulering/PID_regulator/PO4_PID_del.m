%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Prosjekt04_PID_Regulering
%
% Hensikten med programmet er å tune er regulator
% for styring av hastigheten til en motor
%
% Følgende motorer brukes:
%  - motor A
%--------------------------------------------------------------------------

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%         EXPERIMENT SETUP, FILENAME AND FIGURE

clear; close all                        % Alltid lurt å rydde workspace opp først
online = true;                         % Online mot EV3 eller mot lagrede data?
plotting = true;                        % Skal det plottes mens forsøket kjøres
filename = 'P04_PID_regulator.mat';    % Navnet på datafilen når online=0

if online
    mylego = legoev3('USB');
    joystick = vrjoystick(1);
    [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);

    % motorer
    motorA = motor(mylego, 'A');
    motorA.resetRotation;
else
    % Dersom online=false lastes datafil.
    load(filename)
end

fig1 = figure;
set(gcf, 'Position', [100, 100, 800, 600]); % Sett figurstørrelse
drawnow;

% Initialiserer
JoyMainSwitch = 0;
k = 0;
fc = 1.4;
tau = 1/(2*pi*fc);
%--------------------------------------------------------------------------

% Starter stoppeklokke for automatisk stopp
duration = tic;

while ~JoyMainSwitch
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                       GET TIME AND MEASUREMENT

    k = k + 1;

    if online
        if k == 1
            tic
            Tid(1) = 0;
        else
            Tid(k) = toc;
        end

        % motorer
        VinkelPosMotorA(k) = double(motorA.readRotation);

        % Joystick input
        [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);
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
    %         CONDITIONS, CALCULATIONS AND SET MOTOR POWER

    if toc(duration) > 29
        JoyMainSwitch = 1;
    end

    % PID-parametre
    u0 = 0;
    Kp = 0.04;
    Ki = 0.2;
    Kd = 0.002;
    I_max = 100;
    I_min = -100;
    alfa = 1;

    if k == 1
        T_s(1) = 0.05;

        x1(1) = VinkelPosMotorA(1);
        x2(1) = 0;

        x2_f(1) = 0;
        y(1) = x2_f(1);
        r(1) = 0;
        e(1) = r(1) - y(1);
        e_f(1) = e(1);

        P(1) = 0;
        I(1) = 0;
        D(1) = 0;
    else
        T_s(k) = Tid(k) - Tid(k - 1);

        x1(k) = VinkelPosMotorA(k);
        x2(k) = (x1(k) - x1(k - 1)) / T_s(k);

        alfa(k) = 1 - exp(-T_s(k) / tau);
        x2_f(k) = (1 - alfa(k)) * x2_f(k - 1) + alfa(k) * x2(k);
        y(k) = x2_f(k);

        tidspunkt = [0, 2, 5, 8, 11, 14, 21, 30];
        RefVerdier = [0, 200, 500, 700, 1000, 600, 600];
        for i = 1:length(tidspunkt) - 1
            if Tid(k) >= tidspunkt(i) && Tid(k) < tidspunkt(i + 1)
                r(k) = RefVerdier(i);
            end
        end

        e(k) = r(k) - y(k);

        para = [Kp, Ki, Kd, I_max, I_min, alfa(k)];
        [P(k), I(k), D(k), e_f(k)] = MinPID(I(k - 1), e_f(k - 1), [e(k - 1), e(k)], T_s(k), para);
    end

    u_A(k) = u0 + P(k) + I(k) + D(k);

    if online
        motorA.Speed = u_A(k);
        start(motorA);
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                  PLOT DATA

    if plotting || JoyMainSwitch
        figure(fig1)

        % Vinkelhastighet og referanse
        subplot(3, 1, 1)
        plot(Tid(1:k), r(1:k), 'r-', 'DisplayName', '$r_k$');
        hold on
        plot(Tid(1:k), y(1:k), 'b-', 'DisplayName', '$y_k$');
        hold off
        grid on
        ylabel('$[^{\circ}/s]$', 'Interpreter', 'latex')
        text(Tid(k), r(k), ['$' sprintf('%1.0f', r(k)) '^{\circ}/s$'], 'Interpreter', 'latex');
        text(Tid(k), y(k), ['$' sprintf('%1.0f', y(k)) '^{\circ}/s$'], 'Interpreter', 'latex');
        title('Filtrert vinkelhastighet $y(t)$ og referanse $r(t)$', 'Interpreter', 'latex')

        % Reguleringsavvik
        subplot(3, 1, 2)
        plot(Tid(1:k), e(1:k), 'b-', 'DisplayName', '$e_k$');
        hold on
        plot(Tid(1:k), e_f(1:k), 'r--', 'DisplayName', '$e_{f,k}$');
        hold off
        grid on
        title('Reguleringsavvik $e(t)$', 'Interpreter', 'latex')
        ylabel('$[^{\circ}/s]$', 'Interpreter', 'latex')

        % PID bidrag
        subplot(3, 1, 3)
        plot(Tid(1:k), P(1:k), 'b-', 'DisplayName', ['P-del, $K_p=' sprintf('%1.2f', Kp) '$']);
        hold on
        plot(Tid(1:k), I(1:k), 'r-', 'DisplayName', ['I-del, $K_i=' sprintf('%1.2f', Ki) '$']);
        plot(Tid(1:k), D(1:k), 'g-', 'DisplayName', ['D-del, $K_d=' sprintf('%1.2f', Kd) '$']);
        plot(Tid(1:k), u_A(1:k), 'k-', 'DisplayName', '$u(t)$');
        hold off
        grid on
        title('Bidragene P, I, og D og totalp{\aa}drag', 'Interpreter', 'latex')
        xlabel('Tid [sek]', 'Interpreter', 'latex')
        drawnow
    end
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                       STOP MOTORS OG LEGG TIL LEGENDE

subplot(3, 1, 1);
legend('Location', 'northeast', 'Interpreter', 'latex');

subplot(3, 1, 2);
legend('Location', 'northeast', 'Interpreter', 'latex');

subplot(3, 1, 3);
legend('Location', 'northeast', 'Interpreter', 'latex');

if online
    stop(motorA);
end
