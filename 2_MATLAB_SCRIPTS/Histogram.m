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

contact_vision_aided_graph = zeros(3,2,4); %controller, user, layer
contact_no_vision_aided_graph = zeros(3,2,4); %controller, user, layer

perforation_vision_aided_graph = zeros(3,2,4); %controller, user, layer
perforation_no_vision_aided_graph = zeros(3,2,4); %controller, user, layer

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
                contact_vision_aided_graph(str2double(controller(end)), str2double(user(end)), m(R.Tissue{j})) = R.Error_mm_(j);
            end
        else
            for j = 1:height(R)
                contact_no_vision_aided_graph(str2double(controller(end)), str2double(user(end)), m(R.Tissue{j})) = R.Error_mm_(j);
            end
        end
    end
    
    if(~isempty(S))
        if (strcmp(aid,'Vision_aided'))
            for j = 1:height(S)
                perforation_vision_aided_graph(str2double(controller(end)), str2double(user(end)), m(S.Tissue{j})) = S.Error_mm_(j);
            end
        else
            for j = 1:height(S)
                perforation_no_vision_aided_graph(str2double(controller(end)), str2double(user(end)), m(S.Tissue{j})) = S.Error_mm_(j);
            end
        end
    end
    
    
    
end

mean_contact_vision_aided_graph = (contact_vision_aided_graph(:,1,:) + contact_vision_aided_graph(:,2,:))./2;
mean_contact_no_vision_aided_graph = (contact_no_vision_aided_graph(:,1,:) + contact_no_vision_aided_graph(:,2,:))./2;

mean_perforation_vision_aided_graph = (perforation_vision_aided_graph(:,1,:) + perforation_vision_aided_graph(:,2,:))./2;
mean_perforation_no_vision_aided_graph = (perforation_no_vision_aided_graph(:,1,:) + perforation_no_vision_aided_graph(:,2,:))./2;



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
barh(-squeeze(mean_contact_vision_aided_graph(:,1,:))');
title('CONTACT Vision aided error (m)');
set(gca,'yticklabel',l);
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
legend('Controller 1', 'Controller 2', 'Controller 3');
grid on

name = fullfile(parent, '0_Contact_Visual_aid_graph.png');
    
print(name, '-dpng');

figure
barh(-squeeze(mean_contact_no_vision_aided_graph(:,1,:))');
title('CONTACT No-vision aided error (m)');
set(gca,'yticklabel',l);
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
legend('Controller 1', 'Controller 2', 'Controller 3');
grid on

name = fullfile(parent, '0_Contact_No_visual_aid_graph.png');
    
print(name, '-dpng');


figure
barh(-squeeze(mean_perforation_vision_aided_graph(:,1,:))');
title('PERFORATION Vision aided error (m)');
set(gca,'yticklabel',l);
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
legend('Controller 1', 'Controller 2', 'Controller 3');
grid on

name = fullfile(parent, '1_Perforation_Visual_aid_graph.png');
    
print(name, '-dpng');

figure
barh(-squeeze(mean_perforation_no_vision_aided_graph(:,1,:))');
title('PERFORATION No-vision aided error (m)');
set(gca,'yticklabel',l);
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
legend('Controller 1', 'Controller 2', 'Controller 3');
grid on

name = fullfile(parent, '1_Perforation_No_visual_aid_graph.png');
    
print(name, '-dpng');



