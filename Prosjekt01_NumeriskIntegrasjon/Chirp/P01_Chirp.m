% Load the file
data = load('P01_chirp_justert.mat');

% Delete the items at index 2 and 9 from both Lys and Tid
data.Tid([1, 7]) = [];
data.Lys([1, 7]) = [];

% Normalize both vectors to start at zero
data.Tid = data.Tid - data.Tid(1);
data.Lys = data.Lys - data.Lys(1);

% Save it back (overwrite the original file)
save('P01_chirp_justert.mat', '-struct', 'data');
