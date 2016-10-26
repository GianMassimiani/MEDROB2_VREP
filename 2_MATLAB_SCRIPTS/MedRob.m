clc; clear all; close all;

% import table
path = uigetdir('C:\Users\Istin\Documents\CHAI3D\modules\MEDROB2_VREP\0_MATLAB_DATASTREAMS', 'Select data file');
addpath(path);
[parent,controller,~] = fileparts(path);
[parent,aid,~] = fileparts(parent);
[~,user,~] = fileparts(parent);
T = readtable('01_GEOMAGIC_file_time_forces.txt');
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

figure(1);
title(st);
xlabel('Time [s]');
ylabel('Ext. Force [N]');
hold on;
grid on;
p1 = plot(T.Time, T.F_x, 'Color', 'red', 'DisplayName', 'F_x');
p2 = plot(T.Time, T.F_y, 'Color', 'green', 'DisplayName', 'F_y');
p3 = plot(T.Time, T.F_z, 'Color', 'blue', 'DisplayName', 'F_z');

plot_handlers = [p1,p2,p3];
yl = ylim;


keys = cell2mat(T.Key);
p4=0;p5=0;
% first A
user_feedback_A = find(keys == 'A', 1, 'first');
if(~isempty(user_feedback_A))
    first_user_feedback_A_time = T.Time(user_feedback_A);
    p4 = plot([first_user_feedback_A_time first_user_feedback_A_time],yl, '--', 'DisplayName', 'Touch', 'Color', 'black');
    plot_handlers = [plot_handlers, p4];
else
    disp('NO TOUCH PERCIEVED');
end


% first S
user_feedback_S = find(keys == 'S', 1, 'first');
if(~isempty(user_feedback_S))
    first_user_feedback_S_time = T.Time(user_feedback_S);
    p5 = plot([first_user_feedback_S_time first_user_feedback_S_time],yl, '-.', 'DisplayName', 'Perforation', 'Color', [1,.5,0]);
    plot_handlers = [plot_handlers, p5];
else
    disp('NO PERFORATIONS PERCIEVED');
end



while(~isempty(user_feedback_A))
    user_feedback_X = find(keys(user_feedback_A:end) == 'X', 1, 'first');
    
    tmp = user_feedback_A+user_feedback_X-1;
    user_feedback_A = find(keys(tmp:end) == 'A', 1, 'first');
    user_feedback_A = user_feedback_A + tmp - 1;
    user_feedback_A_time = T.Time(user_feedback_A);
    if(~isempty(user_feedback_A))
        plot([user_feedback_A_time user_feedback_A_time],yl, '--', 'Color', 'black');
    end
    
end

while(~isempty(user_feedback_S))
    % for next touch
    user_feedback_X = find(keys(user_feedback_S:end) == 'X', 1, 'first');
    % second S
    tmp = user_feedback_S+user_feedback_X;
    user_feedback_S = find(keys(tmp:end) == 'S', 1, 'first');
    user_feedback_S = tmp + user_feedback_S - 1;
    user_feedback_S_time = T.Time(user_feedback_S);
    
    if(~isempty(user_feedback_S))
        plot([user_feedback_S_time user_feedback_S_time],yl, '-.', 'Color', [1,.5,0]);
    end
end

legend(plot_handlers, 'Location', 'northwest');

hold off;

name = [path, '\', user, '_', aid, '_', controller, '.png'];

print(name, '-dpng');

