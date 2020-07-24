clear
clc
fclose(instrfind);
%User Defined Properties 
serialPort = 'COM6';            % define COM port #
plotTitle = 'Serial Data Log';  % plot title
xLabel = 'Elapsed Time (s)';    % x-axis label
yLabel = 'Data';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = -2;                     % set y-min
max = 2;                      % set y-max
scrollWidth = 10;               % display period in plot, plot entire data log if <= 0
delay = .01;                    % make sure sample faster than resolution
%Define Function Variables
time = 0;
data_x = 0;
count = 0;
%Set up Plot
plotGraphx = plot(time,data_x,'-mo',...
            'LineWidth',1,...
            'MarkerEdgeColor','k',...
            'MarkerFaceColor',[.49 1 .63],...
            'MarkerSize',2);
title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);

%Open Serial COM Port
s = serial(serialPort,'BaudRate',921600)
disp('Close Plot to End Session');
fopen(s);

data='';
while (strcmp(data,'')== true)
    data = fscanf(s,'%s');
    if(strcmp(data,"HelloMatlab")== true )
        fprintf(s,"HelloSTM");
        disp("Connected!")
    else
        disp('.')
    end
end

tic 
while ( ishandle(plotGraphx)  ) %Loop when Plot is Active && ishandle(plotGraphy) && ishandle(plotGraphz)
   dat = fscanf(s,['%f,%f,%f,%f,%f,%f']); %Read Data from Serial as Float: PLEASE NOTE: Here i modified the code, in order to adjust the correct format of data in input.
   if(~isempty(dat) && isfloat(dat)) %Make sure Data Type is Correct        
       count = count + 1;    
       time(count) = toc;    %Extract Elapsed Time
       data_x(count) = dat(4); %Extract 1st Data Element   
       %Set Axis according to Scroll Width
       if(scrollWidth > 0)
       set(plotGraphx,'XData',time(time > time(count)-scrollWidth),'YData',data_x(time > time(count)-scrollWidth));
       axis([time(count)-scrollWidth time(count) min max]);
       else
       set(plotGraphx,'XData',time,'YData',data_x);
       axis([0 time(count) min max]);
       end
       %Allow MATLAB to Update Plot
       pause(delay);
   end
end
%Close Serial COM Port and Delete useless Variables
fclose(instrfind);
clear count dat delay max min plotGraph plotGrid plotTitle s ...
       scrollWidth serialPort xLabel yLabel;
disp('Session Terminated...');