close all;                     	
clear all;                         	
clc;                          	

port = "COM5";              
baudrate = 115200;          
s = serialport(port, baudrate);
flush(s);


dataMatrix = [];                 
acc=[];
gyr=[];
R=[];
tcAcc=[];
linAcc=[];
linVel=[];
pos=[];

samplePeriod=1/250;
startTime = tic;



%  figure('Name', 'Acceleration'); % canlı grafik
%  h1 = animatedline('Color','r');
%  h2 = animatedline('Color','g');
%  h3 = animatedline('Color','b');
%  legend('Acc X', 'Acc Y', 'Acc Z');
% xlabel('Örnek');
%  ylabel('Değer');
% grid on;
% 
% figure('NumberTitle', 'off', 'Name', 'Gyroscope');
% hold on;
%  g1 = animatedline('Color','r');
%  g2 = animatedline('Color','g');
%  g3 = animatedline('Color','b');
% xlabel('sample');
% ylabel('dps');
% title('Gyroscope');
% legend('X', 'Y', 'Z');
% 
% figure('NumberTitle', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
% hold on;
%  a1 = animatedline('Color','r');
%  a2 = animatedline('Color','g');
%  a3 = animatedline('Color','b');
% xlabel('sample');
% ylabel('g');
% title('''Tilt-compensated'' accelerometer');
% legend('X', 'Y', 'Z');
% 
% figure('NumberTitle', 'off', 'Name', 'Linear Acceleration');
% hold on;
%  la1 = animatedline('Color','r');
%  la2 = animatedline('Color','g');
%  la3 = animatedline('Color','b');
% xlabel('sample');
% ylabel('g');
% title('Linear acceleration');
% legend('X', 'Y', 'Z');
% 
% 
% figure('NumberTitle', 'off', 'Name', 'Linear Velocity');
% hold on;
%  lv1 = animatedline('Color','r');
%  lv2 = animatedline('Color','g');
%  lv3 = animatedline('Color','b');
% xlabel('sample');
% ylabel('g');
% title('Linear velocity');
% legend('X', 'Y', 'Z');
% 
% figure('NumberTitle', 'off', 'Name', 'Linear Position');
% hold on;
%  lp1 = animatedline('Color','r');
%  lp2 = animatedline('Color','g');
%  lp3 = animatedline('Color','b');
% xlabel('sample');
% ylabel('g');
% title('Linear position');
% legend('X', 'Y', 'Z');

SamplePlotFreq=1;
anim = InitLiveSixDOF(...
    'AxisLength', 0.2, ...
    'Trail', 'DotsOnly', ...
    'SamplePlotFreq', 4, ...
    'Title', 'Gerçek Zamanlı Demo', ...
    'FullScreen', true, ...
    'Xlabel', 'X', ...
    'Ylabel', 'Y', ...
    'Zlabel', 'Z', ...
    'View', [45 30] ...
);
%get(anim.orgHandle, 'Type') 


format long g
i = 0;
while true   
        line = readline(s);                     
        values = sscanf(line, '%f');           
        if numel(values) == 27          
            i = i + 1;
     
        dataMatrix(i, :) = values';         
            acc = dataMatrix(:, 1:3);
            gyr = dataMatrix(:, 4:6);
            R = reshape(dataMatrix(i, 7:15), [3, 3])';
            tcAcc=dataMatrix(:,16:18);
            linAcc=dataMatrix(:,19:21);
            linVel=dataMatrix(:,22:24);
            pos=dataMatrix(:,25:27);
               
            
            % addpoints(a1, i, tcAcc(i,1));
            % addpoints(a2, i, tcAcc(i,2));
            % addpoints(a3, i, tcAcc(i,3));
            % 
            % addpoints(h1, i, acc(i,1));
            % addpoints(h2, i, acc(i,2));
            % addpoints(h3, i, acc(i,3));
            % 
            % addpoints(g1, i, gyr(i,1));
            % addpoints(g2, i, gyr(i,2));
            % addpoints(g3, i, gyr(i,3));
            % 
            % addpoints(la1, i, linAcc(i,1));
            % addpoints(la2, i, linAcc(i,2));
            % addpoints(la3, i, linAcc(i,3));
            % 
            % addpoints(lv1, i, linVelHP(i,1));
            % addpoints(lv2, i, linVelHP(i,2));
            % addpoints(lv3, i, linVelHP(i,3));
            % 
            % addpoints(lp1, i, pos(i,1));
            % addpoints(lp2, i, pos(i,2));
            % addpoints(lp3, i, pos(i,3));
            % 
            UpdateLiveSixDOF(anim, pos, R);
            drawnow limitrate;
        end
end
