
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
filename = 'Prosjekt03_NumeriskDerivasjon/Chirp/P01_chirp_justert.mat'; % Filnavn for lagring/lasting av data


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
fc = 0.5;
tau = 1/(2*pi*fc);  % Time constant for LPF
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
        Tid(k) = toc;  %Tid blir alltid oppdatert

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
        if Tid(k) < 2
            motor_speed = 0; % Ingen pådrag de første 2 sekundene
        elseif Tid(k) >= 2 && Tid(k) <= 17  % Fra 2 sek til 17 sek (2 + 15)
            motor_speed = round((Tid(k) - 2) / 13 * 100); % Lineær økning til 100%
        else
            motor_speed = 100; % Maks pådrag etter 17 sek
        end

        % Sett motorkraft
        mymotor.Speed = motor_speed;
       
        start(mymotor);  % Start motor med oppdatert hastighet

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
        y(k) = 20;     % Startverdi for volum (kan justeres)
        y_filtered = zeros(1, length(Tid));

    else
        if k <= length(Tid) && k <= length(u)

            T_s(k) = Tid(k) - Tid(k-1); % Tidstrinn

            % Apply during processing:
            if k > 1

                % Low-pass filter u(k)
                alpha = 1 - exp(-T_s(k)/tau);

                % Trapezoidal integrasjon

                u_filtrert(k) = (1 - alpha) * u(k-1) + alpha * u(k)
                 
                y(k) = ((u_filtrert(k) - u_filtrert(k-1)) / T_s(k))



            end


        end
    end

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % PLOTTING AV DATA

    if plotting || JoyMainSwitch

        if k <= length(Tid) && k <= length(u) && k <= length(y)

            subplot(2, 1, 1);
            plot(Tid(1:k), u_filtrert(1:k), 'b-', 'LineWidth', 1.5);
            title('Vannstr{\aa}m inn og ut av Ballong','Interpreter','latex');
            ylabel('(Liter/s)');
            grid on;
            if k > 1
                xlim([min(Tid(1:k)), max(Tid(1:k))]);
            end
            ylim([min(u_filtrert(1:k)) - 1, max(u_filtrert(1:k)) + 1]);

            subplot(2, 1, 2);
            plot(Tid(1:k), y(1:k), 'r-', 'LineWidth', 1.5);
            title('Ballong Volum','Interpreter','latex');
            xlabel('Tid [sek]');
            ylabel('(Liter)');
            grid on;
            if k > 1
                xlim([min(Tid(1:k)), max(Tid(1:k))]);
            end
            ylim([min(y(1:k)) - 1, max(y(1:k)) + 1]);
            drawnow;
        end
    end
end
FrekvensSpekterSignal(u(1:k), Tid(1:k))
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% AVSLUTNING: STOPP MOTOR OG LAGRE DATA

% Legg til legend
subplot(2, 1, 1);
legend('$\{u_k\}$', 'Interpreter', 'latex');

subplot(2, 1, 2);
legend('$\{y_k\}$', 'Interpreter', 'latex');
%PlotAltOmFilter(u, y, Tid, b, a, 10, 'str')
% Stopp motor før avslutning

% AVSLUTNING: STOPP MOTOR
if online
    stop(mymotor); % Stopp motor før avslutning
end

if online
    % Lagre data
    %save('P01_chirp.mat', 'Tid', 'Lys', 'JoyPot'); % RETTET: Lagre data
end