%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Prosjekt04_PID_Regulering
%
% Hensikten med programmet er å tune er regulator
% for styring av hastigheten til en motor
%
% Følgende  motorer brukes:
%  - motor A
%
%--------------------------------------------------------------------------

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%         EXPERIMENT SETUP, FILENAME AND FIGURE

clear; close all   % Alltid lurt å rydde workspace opp først
online = false;     % Online mot EV3 eller mot lagrede data?
plotting = false;  % Skal det plottes mens forsøket kjøres
filename = 'P04_D_del.mat'; % Navnet på datafilen når online=0.

if online
    mylego = legoev3('USB');
    joystick = vrjoystick(1);
    [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);

    % motorer
    motorA = motor(mylego,'A');
    motorA.resetRotation;
else
    % Dersom online=false lastes datafil.
    load(filename)
end

fig1 = figure;
set(gcf, 'Position', [100, 100, 800, 600]); % Sett figurstørrelse
drawnow;

% setter skyteknapp til 0, og initialiserer tellevariabel k
JoyMainSwitch=0;
k=0;

%----------------------------------------------------------------------

% Starter stoppeklokke for å stoppe
% eksperiment automatisk når t>29 sekund.
% Du kan også stoppe med skyteknappen som før.
duration = tic;

while ~JoyMainSwitch

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                       GET TIME AND MEASUREMENT
    % Få tid og målinger fra sensorer, motorer og joystick

    % oppdater tellevariabel
    k=k+1;

    if online
        if k==1
            tic
            Tid(1) = 0;
        else
            Tid(k) = toc;
        end

        % motorer
        VinkelPosMotorA(k) = double(motorA.readRotation);

        % Data fra styrestikke. Utvid selv med andre knapper og akser
        [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1);
    else
        % online=false
        % Når k er like stor som antall elementer i datavektpren Tid,
        % simuleres det at bryter på styrestikke trykkes inn.
        if k==length(Tid)
            JoyMainSwitch=1;
        end

        if plotting
            % Simulerer tiden som EV3-Matlab bruker på kommunikasjon
            % når du har valgt "plotting=true" i offline
            pause(0.03)
        end
    end
    %--------------------------------------------------------------



    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER

    % Stopper automatisk når t>29 sekund
    if toc(duration) > 29
        JoyMainSwitch = 1;
    end

    % parametre
    u0 = 0;
    Kp = 0;    % start med lave verdier, typisk 0.005 Best-value 0.1
    Ki = 0;      % start med lave verdier, typisk 0.005
    Kd = 0.01;      % start med lave verdier, typisk 0.001
    I_max = 100;
    I_min = -100;
    alfa = 1;

    if k==1
        % Initialverdier
        T_s(1) = 0.05;      % nominell verdi

        % Motorens tilstander
        x1(1) = VinkelPosMotorA(1);  % vinkelposisjon motor
        x2(1) = 0;                   % vinkelhastighet motor

        % Måling, referansen, reguleringsavvik
        x2_f(1) = 0;         % filtrert vinkelhastighet motor
        y(1) = x2_f(1);      % måling filtrert vinkelhastighet
        r(1) = 0;            % referanse
        e(1) = r(1)-y(1);    % reguleringsavvik
        e_f(1) = e(1);       % filtrert reg.avvik for D-ledd

        % Initialverdi PID-regulatorens deler
        P(1) = 0;       % P-del
        I(1) = 0;       % I-del
        D(1) = 0;       % D-del
    else
        % Beregninger av tidsskritt
        T_s(k) = Tid(k)-Tid(k-1);

        % Motorens tilstander
        % x1: vinkelposisjon og
        % x2: vinkelhastighet (derivert av posisjon)
        x1(k) = VinkelPosMotorA(k);
        x2(k) = (x1(k)-x1(k-1))/T_s(k);

        % Målingen y er lavpassfiltrert vinkelhastighet
        tau = 0.2;      % tidskonstant til filteret
        alfa(k)  = 1-exp(-T_s(k)/tau);  % tidsavhengig alfa
        x2_f(k) = (1-alfa(k))*x2_f(k-1) + alfa(k)*x2(k);
        y(k) = x2_f(k);

        % Referanse r(k), forhåndsdefinert
        tidspunkt =  [0, 2,  5,  8,  11,  14, 21, 30];  % sekund
        RefVerdier = [0 200 500,700, 1000,600,600];  % grader/s
        for i = 1:length(tidspunkt)-1
            if Tid(k) >= tidspunkt(i) && Tid(k) < tidspunkt(i+1)
                r(k) = RefVerdier(i);
            end
        end

        % Lag kode for bidragene P(k), I(k) og D(k)

        % Reguleringssavvik
        e(k) = r(k)-y(k);
        % Filtrer e(t) før beregning av D
        e_f(k) = (1 - alfa(k)) * e_f(k-1) + alfa(k) * e(k);

        % Beregn PID-komponenter
        P(k) = Kp * e(k);  % P-ledd
        I(k) = I(k-1) + Ki * e(k) * T_s(k);  % I-ledd (integrasjon)
        D(k) = Kd * (e_f(k) - e_f(k-1)) / T_s(k);  % D-ledd (derivert)


    end

    % Integratorbegrensing
    % if ...

    % Anti-windup: Begrens integrator
    if I(k) > I_max
        I(k) = I_max;
    elseif I(k) < I_min
        I(k) = I_min;
    end


    % Total pådrag

    u_A(k) = u0 + P(k) + I(k) + D(k);

    if online
        motorA.Speed = u_A(k);
        start(motorA)
    end
    %--------------------------------------------------------------



    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                  PLOT DATA
    % Husk at syntaksen plot(Tid(1:k),data(1:k))
    % for gir samme opplevelse i online=0 og online=1 siden
    % hele datasettet (1:end) eksisterer i den lagrede .mat fila

    if plotting || JoyMainSwitch
        figure(fig1)

        % Første subplot: Vinkelhastighet og referanse
        subplot(3,1,1)
        plot(Tid(1:k), r(1:k), 'r-', 'DisplayName', '${r_k}$');
        hold on
        plot(Tid(1:k), y(1:k), 'b-', 'DisplayName', '${y_k}$');
        hold off
        grid on
        ylabel('$[^{\circ}/s]$', 'Interpreter', 'latex')
        text(Tid(k), r(k), ['$' sprintf('%1.0f', r(k)) '^{\circ}/s$'], 'Interpreter', 'latex');
        text(Tid(k), y(k), ['$' sprintf('%1.0f', y(k)) '^{\circ}/s$'], 'Interpreter', 'latex');
        title('Filtrert vinkelhastighet $y(t)$ og referanse $r(t)$', 'Interpreter', 'latex')
        legend('Location', 'best', 'Interpreter', 'latex')

        % Andre subplot: Reguleringsavvik
        subplot(3,1,2)
        plot(Tid(1:k), e(1:k), 'b-', 'DisplayName', '${e_k}$');
        hold on
        plot(Tid(1:k), e_f(1:k), 'r--', 'DisplayName', '${e_f,_k}$');
        hold off
        grid on
        title('Reguleringsavvik $e(t)$', 'Interpreter', 'latex')
        ylabel('$[^{\circ}/s]$', 'Interpreter', 'latex')
        legend('Location', 'best', 'Interpreter', 'latex')

        % Tredje subplot: PID bidrag
        subplot(3,1,3)
        plot(Tid(1:k), P(1:k), 'b-', 'DisplayName', ['P-del, $K_p$=' sprintf('%1.2f', Kp)]);
        hold on
        plot(Tid(1:k), I(1:k), 'r-', 'DisplayName', ['I-del, $K_i$=' sprintf('%1.2f', Ki)]);
        plot(Tid(1:k), D(1:k), 'g-', 'DisplayName', ['D-del, $K_d$=' sprintf('%1.2f', Kd)]);
        plot(Tid(1:k), u_A(1:k), 'k-', 'DisplayName', '$u(t)$');
        hold off
        grid on
        title('Bidragene P, I, og D og totalp{\aa}drag', 'Interpreter', 'latex')
        xlabel('Tid [sek]', 'Interpreter', 'latex')
        legend('Location', 'best', 'Interpreter', 'latex')

        % Oppdater plott
        drawnow
    end

    %--------------------------------------------------------------

    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %           STOP MOTORS

    if online
        stop(motorA);
    end


end
%------------------------------------------------------------------





