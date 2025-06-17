% main.m

% Parámetros de la función
f = 2;    % tamaño del parche
r = 3;    % radio de búsqueda
k = 1.0;  % parámetro de suavizado
file_name = '../dof';
% Llamar a la función de denoising
Nldenoise(file_name, f, r, k);

exit;
file_name = '../indirect_cbox_shield';

indirect_denoised = Nldenoise_2(file_name, f, r, k);
direct = exrread('../direct_cbox_shield.exr');

final = indirect_denoised + direct;

exrwrite(final, 'resultado_final.exr');

disp('Proceso de denoising completado.');
