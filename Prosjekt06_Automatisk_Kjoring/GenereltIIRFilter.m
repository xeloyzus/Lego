function y_k = GenereltIIRFilter(u,y,B,A)
%GenereltIIRFilter  Generelt rekursivt IIR-filter
%  y(k) = - [y(k-1) y(k-2) ...]*[a1 a2  ..]' ...  
%         + [u(k) u(k-1) u(k-2) ...]*[b0 b1 b2 ..]'
%
% Syntaks:  
% y(k) = GenereltIIRFilter(u(1:k),y(1:k-1),B,A)
%
% Innganger:
%     u(1:k)   - signal som skal filtreres
%     y(1:k-1) - gamle filtrerte verdier
%     B        - filterparametere, [b0 b1 b2 ..]
%     A        - filterparametere, [ 1 a1 a2 ..]
%
% Utganger:
%     y(k)     - filtrert verdi
%
% Eksempel lavpassfilter:
%     t = 0:0.01:5;
%     u = sin(2*pi*0.5*t);
%     y(1) = u(1);
%     for k = 2:length(t)
%        y(k) = GenereltIIRFilter(u(1:k), y(1:k-1), 0.05, [1 -0.95]);
%     end
%     plot(t,u,t,y)

% Author: Drengstig Tormod
% Work address: UiS
% Last revision: 12-May-2024


%------------- BEGIN CODE --------------
M = numel(B);
N = numel(A)-1;        % teller ikke med a0

% Sørger for at y og u er liggende vektorer.
if size(y,1) > size(y,2)
    y = y';
end
if size(u,1) > size(u,2)
    u = u';
end

if numel(u) < M || numel(y) < N  
    y_k = u(1);   % foretar ikke filtrering før nok u eller y
else
    U = u(end:-1:end-M+1);      % snur hele u-vektoren
    Y = y(end:-1:end-N+1);      % snur hele y-vektoren
    y_k = - Y*A(2:end)' + U*B';
end
%------------- END OF CODE --------------
