% main.m

% Parámetros de la función
f = 3;    % tamaño del parche
r = 5;    % radio de búsqueda
k = 0.2;  % parámetro de suavizado
file_name = '../dof';

Nldenoise(file_name, f, r, k);

file_name = '../indirect_dof';

indirect_denoised = Nldenoise_2(file_name, f, r, k);
direct = exrread('../direct_dof.exr');

final = indirect_denoised + direct;

exrwrite(final, '../resultado_final.exr');
disp('Proceso de denoising completado.');
