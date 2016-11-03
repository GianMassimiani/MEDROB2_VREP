clc; clear all; close all;


addpath(fullfile('..','0_MATLAB_DATASTREAMS'));

fid=fopen('find_mirco.txt');
tline = fgetl(fid);
file_list = {};
while ischar(tline)
    file_list = vertcat(file_list,tline);
    tline=fgetl(fid);
end
fclose(fid);

for i = 1:length(file_list)
    path = file_list{i};
    % import table
    [parent,controller,~] = fileparts(path);
    [parent,aid,~] = fileparts(parent);
    [~,user,~] = fileparts(parent);
    T = readtable(fullfile(path,'01_GEOMAGIC_file_time_forces.txt'));
    R = readtable(fullfile(path,'03_GEOMAGIC_file_contacts_error.txt'));
    Tis = table(cell(height(R),1),'VariableNames', {'Tissue'});
    R = [R Tis];
    S = readtable(fullfile(path,'02_GEOMAGIC_file_perforation_error.txt'));
    Tis = table(cell(height(S),1),'VariableNames', {'Tissue'});
    S = [S Tis];
        
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
    
    
    
    
    for i = 1:length(R.Time)
        R.Time(i)
        plot(R.Time(i), sum(yl)/2, 'x', 'Color', [0,0,1]);
        str = input('Tissue? ','s');
        R.Tissue{i} = str;
    end
        
    for i = 1:length(S.Time)
        S.Time(i)
        plot(S.Time(i), sum(yl)/2, 'o', 'Color', [1,0,0]);
        str = input('Tissue? ','s');
        S.Tissue{i} = str;
    end
    writetable(R,fullfile(path,'03_GEOMAGIC_file_contacts_error.txt'));
    writetable(S,fullfile(path,'02_GEOMAGIC_file_perforation_error.txt'));
    hold off;
    close all
end