% Load your original file
data = load('P01_sinus_justert.mat');

% Extract Lys and Tid
Lys = data.Lys_clean;
Tid = data.Tid_clean;

% Create a mask where time >= 2 seconds
mask = Tid >= 2;

% Apply the mask
Lys = Lys(mask);
Tid = Tid(mask);

% Save the cleaned variables back to a new file
save('P01_sinus_justert_cleaned.mat', 'Lys', 'Tid');
