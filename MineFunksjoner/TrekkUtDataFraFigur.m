% Trekke ut informasjon fra .fig

fig=gcf;

% delfigur = subplot nr, 1,2,3,4,.. 
% Teller nedover første kolonne, deretter neste kolonne
delfigur = 1;     

axObjs = fig.Children(delfigur)  
dataObjs = axObjs.Children

t = dataObjs(1).XData
y = dataObjs(1).YData

