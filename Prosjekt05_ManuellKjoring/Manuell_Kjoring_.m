%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Prosjekt04_PID_Regulering
%
% Hensikten med programmet er å tune er regulator
% for styring av hastigheten til en motor
%
% Følgende motorer brukes:
%  - motor A og motor B
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

    motorA = motor(mylego, 'A');
    motorB = motor(mylego,'D');
    % Oppsett av sensorer
    myColorSensor = colorSensor(mylego);

    motorA.resetRotation;
    motorB.resetRotation;
else
    % Dersom online=false lastes datafil.
    load(filename)
end

fig1 = figure;
set(gcf, 'Position', [100, 100, 1200, 900]); % Sett figurstørrelse for 6 subplots
drawnow;

% Initialiserer
JoyMainSwitch = 0;
k = 0;
fc = 1.4;
tau = 1/(2*pi*fc);

% Initialiser kvalitetsmål
IAE = 0;
MAE = 0;
TVA = 0;
TVB = 0;
prev_uA = 0;
prev_uB = 0;

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Løkke for å hente sensorverdier og utføre kjøring

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

        % Hent data fra sensorer 
        Lys(k) = double(readLightIntensity(myColorSensor, 'reflected'));
        VinkelPosMotorA(k) = double(motorA.readRotation);
        VinkelPosMotorB(k) = double(motorB.readRotation);

        % Les joystick-aksene og skaler til -100 til 100
        axes(2,k) = -100 * axis(joystick, 2);  % Frem/bak (invertert)
        axes(3,k) = 100 * axis(joystick, 3);   % Sideveis styring

        % Lagre joystickknapper
        [~, JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1);

        % Definer kontrollvariabler
        fwd_bwd = axes(2,k) / 100;   % Normaliser til [-1, 1]
        turn    = axes(3,k) / 100;   % Normaliser til [-1, 1]
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

    % PID-parametre
    u0 = 0;
    Kp = 0.04;
    Ki = 0.2;
    Kd = 0.002;
    I_max = 100;
    I_min = -100;
    alfa = 1;
    
    % Lysintensitet som referanse
    y(k) = Lys(k);
    
    if k == 1
        T_s(1) = 0.05;

        x1(1) = VinkelPosMotorA(1);
        x2(1) = 0;

        x2_f(1) = 0;
        y(1) = x2_f(1);
        r(1) = Lys(1); % startverdi (den første lysmålingen blir referanse)
        e(1) = r(1) - y(1);
        e_f(1) = e(1);

        P(1) = 0;
        I(1) = 0;
        D(1) = 0;
        
        % Initialiser kvalitetsmål for første tidsskritt
        IAE(k) = abs(e(1)) * T_s(1); % Numerisk integrasjon (rektangelmetode)
        MAE(k) = abs(e(1));          % Gjennomsnittlig absolutt feil
        TVA(k) = 0;                  % Ingen endring i pådrag ennå
        TVB(k) = 0;                  % Ingen endring i pådrag ennå
        prev_uA = 0;
        prev_uB = 0;
    else
        T_s(k) = Tid(k) - Tid(k - 1);

        x1(k) = VinkelPosMotorA(k);
        x2(k) = (x1(k) - x1(k - 1)) / T_s(k);

        alfa(k) = 1 - exp(-T_s(k) / tau);
        x2_f(k) = (1 - alfa(k)) * x2_f(k - 1) + alfa(k) * x2(k);
        y(k) = x2_f(k);

        % Bruk lysintensiteten som en dynamisk referanse
        r(k) = Lys(k);

        e(k) = r(k) - y(k);  % Beregn feilen (avviket fra ønsket gråskalaverdien)

        para = [Kp, Ki, Kd, I_max, I_min, alfa(k)];
        [P(k), I(k), D(k), e_f(k)] = MinPID(I(k - 1), e_f(k - 1), [e(k - 1), e(k)], T_s(k), para);
        
        % Beregn kvalitetsmål
        IAE(k) = IAE(k-1) + abs(e(k)) * T_s(k); % Numerisk integrasjon (rektangelmetode)
        MAE(k) = (MAE(k-1)*(k-1) + abs(e(k))) / k; % Glidende gjennomsnitt
    end

    u = u0 + P(k) + I(k) + D(k);
    
    % Apply the forward/backward speed to both motors
    u_A(k) = fwd_bwd * u;  % Apply forward/backward speed to motor A
    u_B(k) = fwd_bwd * u;  % Apply forward/backward speed to motor B
    
    % Apply turning only when there is a sideways (turn) input
    if abs(turn) > 0.1  % Only adjust if there's noticeable sideways movement
        u_A(k) = u_A(k) - turn * 30;  % Adjust motor A speed for turning
        u_B(k) = u_B(k) + turn * 30;  % Adjust motor B speed for turning
    end
    
    % Beregn Total Variation (TV) for motor A og B
    if k > 1
        TVA(k) = TVA(k-1) + abs(u_A(k) - prev_uA);
        TVB(k) = TVB(k-1) + abs(u_B(k) - prev_uB);
    else
        TVA(k) = 0;
        TVB(k) = 0;
    end
    prev_uA = u_A(k);
    prev_uB = u_B(k);
    
    % Set motor speeds
    if online
        motorA.Speed = u_A(k);
        motorB.Speed = u_B(k);
        start(motorA);
        start(motorB);
    end

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                  PLOT DATA

    if plotting || JoyMainSwitch
        figure(fig1)

        % Lysmålinger og feil (subplot 1)
        subplot(3, 2, 1)
        plot(Tid(1:k), r(1:k), 'r-', 'DisplayName', '$r_k$');
        hold on
        plot(Tid(1:k), y(1:k), 'b-', 'DisplayName', '$y_k$');
        hold off
        grid on
        ylabel('$[Reflektert lys]$', 'Interpreter', 'latex')
        title('Gråskalaverdier: Referanse og målt verdi', 'Interpreter', 'latex')
        legend('Location', 'best', 'Interpreter', 'latex')

        % Reguleringsavvik (subplot 2)
        subplot(3, 2, 2)
        plot(Tid(1:k), e(1:k), 'b-', 'DisplayName', '$e_k$');
        grid on
        title('Reguleringsavvik (feil mellom referanse og måling)', 'Interpreter', 'latex')
        legend('Location', 'best', 'Interpreter', 'latex')

        % Pådrag (subplot 3)
        subplot(3, 2, 3)
        plot(Tid(1:k), u_A(1:k), 'b-', 'DisplayName', '$u_{A,k}$');
        hold on
        plot(Tid(1:k), u_B(1:k), 'r-', 'DisplayName', '$u_{B,k}$');
        hold off
        grid on
        title('Pådrag for motor A og B', 'Interpreter', 'latex')
        legend('Location', 'best', 'Interpreter', 'latex')

        % Integral of Absolute Error (subplot 4)
        subplot(3, 2, 4)
        plot(Tid(1:k), IAE(1:k), 'b-', 'DisplayName', '$IAE_k$');
        grid on
        title('Integral of Absolute Error (IAE)', 'Interpreter', 'latex')
        legend('Location', 'best', 'Interpreter', 'latex')

        % Total Variation (subplot 5)
        subplot(3, 2, 5)
        plot(Tid(1:k), TVA(1:k), 'b-', 'DisplayName', '$TV_{A,k}$');
        hold on
        plot(Tid(1:k), TVB(1:k), 'r-', 'DisplayName', '$TV_{B,k}$');
        hold off
        grid on
        title('Total Variation for motor A og B', 'Interpreter', 'latex')
        legend('Location', 'best', 'Interpreter', 'latex')

        % Mean Absolute Error (subplot 6)
        subplot(3, 2, 6)
        plot(Tid(1:k), MAE(1:k), 'b-', 'DisplayName', '$MAE_k$');
        grid on
        title('Mean Absolute Error (MAE)', 'Interpreter', 'latex')
        legend('Location', 'best', 'Interpreter', 'latex')
        
        drawnow
    end
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                       STOP MOTORS OG LAGRE DATA

if online
    stop(motorA);
    stop(motorB);
    save(filename, 'Tid', 'Lys', 'VinkelPosMotorA', 'VinkelPosMotorB', ...
         'r', 'y', 'e', 'u_A', 'u_B', 'IAE', 'MAE', 'TVA', 'TVB');
end