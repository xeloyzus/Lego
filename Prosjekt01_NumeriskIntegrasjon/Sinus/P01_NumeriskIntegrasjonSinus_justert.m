%--------------------------------------------------------------------------
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
filename = 'Prosjekt01_NumeriskIntegrasjon/Sinus/P01_sinus_justert.mat'; % Filnavn for lagring/lasting av data

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
    load(filename); % Laster inn Tid, Lys, osv.
end

% Opprett figur for plotting
fig1 = figure;
set(gcf, 'Position', [100, 100, 800, 600]);
drawnow;

% Initialiser variabler
JoyMainSwitch = 0; % Knappestatus for joystick - skal være 0 for å starte
k = 0;             % Tellevariabel for iterasjoner
y = [];
u = [];
T_s = [];

fc = 1.5;                      % Lavpassfilter grensefrekvens
tau = 1/(2*pi*fc);              % Tidskonstant for LPF
u_filtrert = zeros(size(Lys));  % Rettet: korrekt initiering
y_filtrert = zeros(size(Lys));  % Rettet: hvis du trenger det senere
Tid = Tid(:)';                  % Sørg for riktig dimensjon (radvektor)
Lys = Lys(:)';                  % Sørg for riktig dimensjon (radvektor)

%--------------------------------------------------------------------------
% HOVEDLØKKE
while ~JoyMainSwitch
    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % HENT TID OG MÅLINGER FRA SENSORER, MOTORER OG JOYSTICK

    k = k + 1;

    if online
        if k == 1
            tic; % Start tidtaker
        end
        Tid(k) = toc;

        Lys(k) = double(readLightIntensity(myColorSensor, 'reflected'));

        [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);

        if length(JoyButtons) >= 1
            JoyMainSwitch = JoyButtons(1);
        end
        if length(JoyAxes) >= 4
            JoyPot(k) = JoyAxes(4);
        else
            JoyPot(k) = 0;
        end

        if k <= length(JoyPot)
            motor_speed = round(JoyPot(k));
            display(motor_speed);
            mymotor.Speed = motor_speed;
            start(mymotor);
        end
    else
        % Offline modus
        if k > length(Tid)
            JoyMainSwitch = 1;
        end
        if plotting
            pause(0.03);
        end
    end

    %=== BEREGNINGER ===
    if k <= length(Lys)
        % Dynamisk baseline-korreksjon med glidende gjennomsnitt
        if k == 1
            LysInit = Lys(1);
        else
            LysInit = mean(Lys(max(1,k-100):k));  % Bruker siste 100 målinger
        end
        u(k) = Lys(k) - LysInit; % Fjerner DC-komponent
    end

    if k == 1
        T_s(1) = 0.05;
        y(k) = 0;  % Startverdi for volum
        u_filtrert(k) = u(k);
    else
        if k <= length(Tid) && k <= length(u)
            T_s(k) = Tid(k) - Tid(k-1);
            
            % Forbedret filtrering med lavere alfa for bedre støyfjerning
            alfa = 1-exp(-T_s(1)/tau);  % Redusert fra 0.01 for raskere respons
            
            % Lavpassfilter for strømsignalet
            u_filtrert(k) = (1 - alfa)*u(k) + alfa*u_filtrert(k-1);
            
            % Numerisk integrasjon med trapesmetoden
            % Legger til høypassfilter-effekt for å fjerne integreringsdrift
            y(k) = y(k-1) + (T_s(k)/2)*(u_filtrert(k-1) + u_filtrert(k));
            
         
        end
    end

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % FILTERING AV SIGNALER
    U = 10;
    w = 2*pi*(1/5);   % ≈1.256 rad/s (5 sekunders periode)
    u_est = U*sin(w*Tid);
    Y = U/w;          % ≈7.96 (teoretisk amplitudeforhold ved integrasjon)
    y_A = -10;        % Justert offset for bedre passing med data
    phi = -pi/2;      % Negativ faseforskyvning for integrasjon
    y_est = Y*sin(w*Tid + phi) + y_A;

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    % PLOTTING
    if plotting || JoyMainSwitch
        if k <= length(Tid) && k <= length(u) && k <= length(y)
            subplot(2, 1, 1);
            hold off;
            plot(Tid(1:k), u(1:k), 'b-', 'LineWidth', 1.5);
            hold on;
            plot(Tid, u_est, 'r--', 'LineWidth', 1.5);
            title('Vannstr{\aa}m inn og ut av Ballong','Interpreter','latex');
            ylabel('(Liter/s)');
            grid on;
            if k > 1
                xlim([min(Tid(1:k)), max(Tid(1:k))]);
            end
            ylim([min(u(1:k)) - 1, max(u(1:k)) + 1]);

            subplot(2, 1, 2);
            hold off;
            plot(Tid(1:k), y(1:k), 'b-', 'LineWidth', 1.5);
            hold on;
            plot(Tid, y_est, 'r--', 'LineWidth', 1.5);
            title('Ballong Volum','Interpreter','latex');
            xlabel('Tid [sek]');
            ylabel('(Liter)');
            grid on;
            if k > 1
                xlim([min(Tid(1:k)), max(Tid(1:k))]);
            end
            % Dynamisk y-aksejustering basert på data
            current_ylim = [min(y(1:k))-2, max(y(1:k))+2];
            if diff(current_ylim) > 0  % Unngår feil hvis alle y-verdier er like
                ylim(current_ylim);
            end
            drawnow;
        end
    end
end

% Frekvensspekter
FrekvensSpekterSignal(u, Tid)

subplot(2, 1, 1);
legend({'$\{u_k\}$', ['$u(t) = ' num2str(U) '\sin(' num2str(w) 't)$']}, 'Interpreter', 'latex');
subplot(2, 1, 2);
legend({'$\{y_k\}$', ['$y(t) = ' num2str(Y) '\sin(' num2str(w) 't + ' num2str(phi) ') + ' num2str(y_A) '$']}, 'Interpreter', 'latex');

% AVSLUTNING
if online
    stop(mymotor);
    % save('P01_sinus.mat', 'Tid', 'Lys', 'JoyPot'); % Avkommenter hvis ønsket
end