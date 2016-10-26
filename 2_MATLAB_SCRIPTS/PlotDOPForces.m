clc; clear all; close all;

% import root of the experiments
%path = uigetdir('C:\Users\Istin\Documents\CHAI3D\modules\MEDROB2_VREP\0_MATLAB_DATASTREAMS', 'Select data file');
addpath('C:\Users\Istin\Documents\CHAI3D\modules\MEDROB2_VREP\0_MATLAB_DATASTREAMS');

fid=fopen('f.txt');
tline = fgetl(fid);
file_list = {};
while ischar(tline)
    file_list = vertcat(file_list,tline);
    tline=fgetl(fid);
end
fclose(fid);

for i = 1:length(file_list)
    path = file_list{i};
    [parent,controller,~] = fileparts(path);
    [parent,aid,~] = fileparts(parent);
    [~,user,~] = fileparts(parent);
    T = readtable(fullfile(path,'00_GEOMAGIC_file_DOP_force.txt'));
    
    if(strcmp('Vision_aided', aid))
        aid = 'VisionAid';
    else
        aid = 'NoVisionAid';
    end
    
    if(strcmp(controller, 'Cont_1'))
        controller = 'Controller1';
    elseif (strcmp(controller, 'Cont_2'))
        controller = 'Controller2';
    else
        controller = 'Controller3';
    end
    st = [user,'\_', aid, '\_', controller];
    
    figure(1)
    title(st);
    [~,idx]=max(T.DOP);
    
    hold on
    grid on
    xlabel('DOP [mm]');
    ylabel('Ext. Force [N]');

    plot(T.DOP(1:idx), T.Force(1:idx),'DisplayName','forward movement');
    plot(T.DOP(idx+1:end), T.Force(idx+1:end),'r--','DisplayName','backward movement');
    yl = ylim;
    ylim([yl(1)-0.1 yl(2)]);
    legend('Location', 'northwest');
    hold off

    name = fullfile(path, ['DOP_FORCE_', user, '_', aid, '_', controller, '.png']);
    print(name, '-dpng');
    close all
end