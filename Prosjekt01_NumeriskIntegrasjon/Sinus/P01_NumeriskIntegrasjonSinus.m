% P01_NumeriskIntegrasjonSinus
%
% Hensikten med programmet er å numerisk integrere målesignal u_k som
% representerer strømning [cl/s] for å beregne y_k som volum [cl].
%
% Følgende sensorer brukes:
% - Lyssensor
%--------------------------------------------------------------------------

clear; close all;   % Tøm arbeidsområdet og lukk alle figurer
online = false;      % Online modus (EV3) eller offline modus (last data)?
plotting = true;    % Skal data plottes i sanntid?
filename = 'Sinus/P01_sinus.mat'; % Filnavn for lagring/lasting av data


if online
    % Koble til LEGO EV3 og joystick
    mylego = legoev3('USB');

    joystick = vrjoystick(1);
    [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);


    % Oppsett av sensorer
    myColorSensor = colorSensor(mylego);

    % Oppsett av motor
    mymotor = motor(mylego, 'A');
else
    % Last inn lagrede data hvis offline
    load(filename);

end

% Opprett figur for plotting
fig1 = figure;
set(gcf, 'Position', [100, 100, 800, 600]); % Sett størrelse på figur
drawnow;

% Initialiser variabler
JoyMainSwitch = 0; % Knappestatus for joystick
k = 0;             % Tellevariabel for iterasjoner
y = []; % Initialize y with the same size as Tid
u = []; % Initialize u with the same size as Tid
T_s = []; % Initialize T_s with the same size as Tid
fc= 2.82;
tau = 1/2*pi*fc;
u_filtrert = zeros(size(Tid));
%--------------------------------------------------------------------------
% HOVEDLØKKE - kjører til brukeren trykker på hovedknappen på joysticken

while ~JoyMainSwitch
    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % HENT TID OG MÅLINGER FRA SENSORER, MOTORER OG JOYSTICK

    k = k + 1; % Øk tellevariabel

    if online
        % Hent tid
        if k == 1
            tic; % Start tidtaker ved første iterasjon
        end
        Tid(k) = toc;  % RETTET: Tid blir alltid oppdatert

        % Hent data fra lyssensor
        Lys(k) = double(readLightIntensity(myColorSensor, 'reflected'));

        % Hent data fra joystick
        [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);

        % Sjekk at vi ikke går ut av array-grenser
        if length(JoyButtons) >= 1
            JoyMainSwitch = JoyButtons(1); % Avslutt program hvis hovedknapp trykkes
        end
        if length(JoyAxes) >= 4
            JoyPot(k) = JoyAxes(4); % Bruk joystick-akse 4 for motorstyring
        else
            JoyPot(k) = 0; % Sett standardverdi til 0 hvis ingen data
        end

        % MOTORSTYRING BASERT PÅ JOYSTICK-VERDIER
        if k <= length(JoyPot)
            motor_speed = round(JoyPot(k)); % Skalér joystick-verdi til motorhastighet (-100 til 100)
            display(motor_speed);
            % Sett motorkraft
            mymotor.Speed = motor_speed;
            start(mymotor);  % Start motor med oppdatert hastighet
        end
    else
        % Offline mode: Simuler data
        if k > length(Tid)
            JoyMainSwitch = 1; % Simuler knappetrykk når data er ferdig
        end
        if plotting
            pause(0.03); % Simuler kommunikasjonsforsinkelse
        end
    end

    % BEREGNINGER OG MOTORSTYRING
    if k <= length(Lys)
        LysInit = Lys(1); % Startverdi for lysintensitet
        u(k) = Lys(k) - LysInit;
    end

    if k == 1
        % Initialiser verdier
        T_s(1) = 0.05; % Standard tidstrinn
        y(k) = 0;     % Startverdi for volum (kan justeres)
    else
        % Beregn tidstrinn
        if k <= length(Tid) && k <= length(u)
            T_s(k) = Tid(k) - Tid(k-1); % Tidstrinn

            if k > 1

                % Low-pass filter u(k)
                alpha = 1 - exp(-T_s(k)/tau);

                % Trapezoidal integrasjon

                u_filtrert(k) = (1 - alpha) * u(k-1) + alpha * u(k)
                y(k) = (1 - alpha) * y(k-1) + alpha * u(k)
            end


        end
    end


    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % FILTERING AV SIGNALER

    % Estimerte verdier for sammenligning
    % Plassert helt nederst etter seksjonen STOP MOTORS
    U = 7.9/2; % amplitude av estimering signal
    w = 2*pi*(1/2.16); %vinkle rad/s av motoren
    u_est = U*sin(w*Tid);
    Y = U/w;

    y_A = U/w + y(1);
    phi = - pi/2;
    y_est = Y*sin(w*Tid + phi) + y_A;

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    % PLOTTING AV DATA
    if plotting || JoyMainSwitch
        if k <= length(Tid) && k <= length(u) && k <= length(y)
            % First subplot (u and u_est)
            subplot(2, 1, 1);
            hold off; % Clear previous plot
            plot(Tid(1:k), u_filtrert(1:k), 'b-', 'LineWidth', 1.5); % Measured signal
            hold on;
            plot(Tid, u_est, 'r--', 'LineWidth', 1.5); % Estimated signal
            title('Vannstr{\aa}m inn og ut av Ballong','Interpreter','latex');
            ylabel('(Liter/s)');
            legend({'$\{u_k\}$', ...
                ['$u(t) = ' num2str(U) '\sin(' num2str(w) 't)$']}, ...
                'Interpreter', 'latex');
            grid on;
            if k > 1
                xlim([min(Tid(1:k)), max(Tid(1:k))]);
            end
            ylim([min(u(1:k)) - 1, max(u(1:k)) + 1]);

            % Second subplot (y and y_est)
            subplot(2, 1, 2);
            hold off; % Clear previous plot
            plot(Tid(1:k), y(1:k), 'b-', 'LineWidth', 1.5); % Measured volume
            hold on;
            plot(Tid, y_est, 'r--', 'LineWidth', 1.5); % Estimated volume
            title('Ballong Volum','Interpreter','latex');
            xlabel('Tid [sek]');
            ylabel('(Liter)');
            legend({'$\{y_k\}$', ...
                ['$y(t) = ' num2str(Y) '\sin(' num2str(w) 't - \pi/2) + ' num2str(y_A) '$']}, ...
                'Interpreter', 'latex');
            grid on;
            if k > 1
                xlim([min(Tid(1:k)), max(Tid(1:k))]);
            end
            ylim([min(y(1:k)) - 1, max(y(1:k)) + 1]);
            drawnow;
        end
    end
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% AVSLUTNING: STOPP MOTOR OG LAGRE DATA


% Stopp motor før avslutning

% AVSLUTNING: STOPP MOTOR
if online
    stop(mymotor); % Stopp motor før avslutning

    % Lagre data
    %save('P01_sinus.mat', 'Tid', 'Lys', 'JoyPot'); % RETTET: Lagre data
end
