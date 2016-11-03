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

m = containers.Map({'s','f','m','b'},{1,2,3,4});

vision_aided_graph = zeros(3,2,4); %controller, user, layer
no_vision_aided_graph = zeros(3,2,4); %controller, user, layer

for i = 1:length(file_list)
    path = file_list{i};
    
    [parent,controller,~] = fileparts(path);
    [parent,aid,~] = fileparts(parent);
    [parent,user,~] = fileparts(parent);
    R = readtable(fullfile(path,'03_GEOMAGIC_file_contacts_error.txt'));
    S = readtable(fullfile(path,'02_GEOMAGIC_file_perforation_error.txt'));
    
    if(~isempty(R))
        if (strcmp(aid,'Vision_aided'))
            for j = 1:height(R)
                vision_aided_graph(str2double(controller(end)), str2double(user(end)), m(R.Tissue{j})) = R.Error_mm_(j);
            end
        else
            for j = 1:height(R)
                no_vision_aided_graph(str2double(controller(end)), str2double(user(end)), m(R.Tissue{j})) = R.Error_mm_(j);
            end
        end
        
    end
    
end

mean_vision_aided_graph = (vision_aided_graph(:,1,:) + vision_aided_graph(:,2,:))./2;

mean_no_vision_aided_graph = (no_vision_aided_graph(:,1,:) + no_vision_aided_graph(:,2,:))./2;

l = {'Skin','Fat','Muscle','Bone'};
% figure
% barh(-squeeze(vision_aided_graph(:,1,:))');
% title('Vision aided error (mm)');
% set(gca,'yticklabel',l);
% set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
% legend('Controller 1', 'Controller 2', 'Controller 3');
% grid on
% 
% figure
% barh(-squeeze(no_vision_aided_graph(:,1,:))');
% title('No-vision aided error (mm)');
% set(gca,'yticklabel',l);
% set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
% legend('Controller 1', 'Controller 2', 'Controller 3');
% grid on
% 
% figure
% barh(-squeeze(vision_aided_graph(:,2,:))');
% title('Vision aided error (mm)');
% set(gca,'yticklabel',l);
% set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
% legend('Controller 1', 'Controller 2', 'Controller 3');
% grid on
% 
% figure
% barh(-squeeze(no_vision_aided_graph(:,2,:))');
% title('No-vision aided error (mm)');
% set(gca,'yticklabel',l);
% set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
% legend('Controller 1', 'Controller 2', 'Controller 3');
% grid on

figure
barh(-squeeze(mean_vision_aided_graph(:,1,:))');
title('Vision aided error (m)');
set(gca,'yticklabel',l);
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
legend('Controller 1', 'Controller 2', 'Controller 3');
grid on

name = fullfile(parent, '0_Visual_aid_graph.png');
    
print(name, '-dpng');

figure
barh(-squeeze(mean_no_vision_aided_graph(:,1,:))');
title('No-vision aided error (m)');
set(gca,'yticklabel',l);
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
legend('Controller 1', 'Controller 2', 'Controller 3');
grid on

name = fullfile(parent, '1_No_visual_aid_graph.png');
    
print(name, '-dpng');



