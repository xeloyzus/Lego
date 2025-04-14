%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% P01_NumeriskIntegrasjonKonstant
%
% Hensikten med programmet er å numerisk integrere målesignal u_k som
% representerer strømning [cl/s] for å beregne y_k som volum [cl].
%
% Følgende sensorer brukes:
% - Lyssensor
%--------------------------------------------------------------------------

clear; close all;   % Clear workspace and close all figures
online = false;     % Online mode (EV3) or offline mode (load data)?
plotting = true;    % Plot data in real-time?
filename = 'P01_LysTid_1.mat'; % Filename for saving/loading data

if online
    % LEGO EV3 and joystick setup
    mylego = legoev3('USB');
    joystick = vrjoystick(1);
    [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);

    % Sensors
    myColorSensor = colorSensor(mylego);
else
    % Load data if offline
    if exist(filename, 'file')
        load(filename);
    else
        error('Fil %s ikke funnet. Kør online eller sjekk filbanen.', filename);
    end
end

fig1 = figure;
set(gcf, 'Position', [100, 100, 800, 600]); % Set figure size
drawnow;

% Initialize variables
% Setter skyteknapp til 0, og initialiserer tellevariabel k

JoyMainSwitch = 1; % Joystick button state (Endret til 1 for å starte)
k = 0;             % Counter for iterations

y = zeros(size(Tid));
u = zeros(size(Tid));
T_s = zeros(size(Tid));

%--------------------------------------------------------------------------

tic; % Start timing

while JoyMainSwitch
    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                       GET TIME AND MEASUREMENT
    % Få tid og målinger fra sensorer, motorer og joystick

    k = k + 1;

    if online
        % Get time
        if k == 1
            Tid(1) = 0;
        else
            Tid(k) = toc;
        end

        % Get sensor data
        Lys(k) = double(readLightIntensity(myColorSensor, 'reflected'));

        % Get joystick data
        [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1); % Check if main button is pressed
    else
        % Offline mode: Simuler data
        if k > length(Tid)
            JoyMainSwitch = 1; % Simuler knappetrykk når data er ferdig
        end
        if plotting
            pause(0.03); % Simuler kommunikasjonsforsinkelse
        end
    end

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER
    % Gjør matematiske beregninger og motorkraftberegninger.

    % Tilordne målinger til variabler
    if k == 1 % Spesifisering av initialverdier og parametere
        LysInit = Lys(k); % Initial light intensity
        T_s(1) = 0.05; % Nominal time step
        y(1) = 20;     % Initial volume (adjust as needed)
    else
        T_s(k) = Tid(k) - Tid(k-1); % Time step

        if k <= length(Tid) && k <= length(u)
            % Apply during processing:
            if k > 1

                u(k) = Lys(k) - LysInit;
                y(k) = y(k-1) + (T_s(k) / 2) * (u(k) + u(k-1)); % Trapezoidal integration

            end

        end
    end

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                              PLOT DATA

    if plotting || JoyMainSwitch
        subplot(2, 1, 1);
        plot(Tid(1:k), u(1:k), 'b-', 'LineWidth', 1.5);
        title('Påfylling og drikking av Vannglass','Interpreter','latex');
        ylabel('Lysintensitet [cl/s]');
        grid on;
        if k > 1
            xlim([min(Tid(1:k)), max(Tid(1:k))]);
        else
            xlim([0, 1]); % Unngå feil med xlim
        end
        ylim([min(u(1:k)) - 1, max(u(1:k)) + 1]);

        subplot(2, 1, 2);
        plot(Tid(1:k), y(1:k), 'r-', 'LineWidth', 1.5);
        title('Vannvolum i glasset','Interpreter','latex');
        xlabel('Tid [sek]');
        ylabel('Volum [cl]');
        grid on;
        if k > 1
            xlim([min(Tid(1:k)), max(Tid(1:k))]);
        else
            xlim([0, 1]);
        end
        ylim([min(y(1:k)) - 1, max(y(1:k)) + 1]);
        drawnow;
    end
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% FINALIZE PLOTS AND SAVE DATA

subplot(2, 1, 1);
legend('$\{u_k\}$', 'Interpreter', 'latex');

subplot(2, 1, 2);
legend('$\{y_k\}$', 'Interpreter', 'latex');

if online
    % Save data
    %save(filename, 'Tid', 'Lys', 'y', 'u');
end
