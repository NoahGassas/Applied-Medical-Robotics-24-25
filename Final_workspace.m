close all
clear all

% declare global variables
%   s           serial port communication
%   hInput1     input widget 1
%   hInput2     input widget 2
%   hPlot       plot widget
%   hFig        figure widget
%   hTimer      continuous timer
%   c           command type
%   y1          data stream series 1
%   y2          data stream series 2
%   r1          length of arm1
%   r2          length of arm2
global s hInput1 hInput2 hPlot hFig hTimer c y1 y2 r1 r2 

%% Set up
% Create serial port object
serialports = serialportlist();
r1 = 10; %cm
r2 = 10; %cm
s = serialport("COM5", 9600);
configureTerminator(s,"CR/LF");
s.UserData = struct("Data",[],"Count",1);
%% Create GUI
hFig = figure;
% Create input field for sending commands to microcontroller
hInput1 = uicontrol('Style', 'edit', 'Position', [20, 20, 100, 25]);
hInput2 = uicontrol('Style', 'edit', 'Position', [120, 20, 100, 25]);
% Create button for sending commands
hSend = uicontrol('Style', 'pushbutton', 'String', 'Send', 'Position', [20, 50, 100, 25], 'Callback', @sendCommand);
% Create plot area
hPlot = axes('Position', [0.2, 0.35, 0.6, 0.6]);
xlim(hPlot, [-20, 20]);
ylim(hPlot, [-20, 20]);
% Set up variables for real-time plotting
c = [];
y1 = [];
y2 = [];
t0 = now;
% Set up timer for continuously receiving data from microcontroller
hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @readDataTimer);
start(hTimer);
hFig.CloseRequestFcn = @closeGUI;

%% Callback function for sending commands
function sendCommand(~, ~)
global s hInput1 hInput2 r1 r2
    %% Get values from input fields as strings but convert to numbers
    input1 = str2num(get(hInput1, 'String'));
    input2 = str2num(get(hInput2, 'String'));
    %validate input
    if numel(input1) ~= 1
        return ; 
    end
    if numel(input2) ~= 1
        return; 
    end
    %% calculate inverse kinematics
    t1=0;
    t2=0;
    x_d = [input1 / 10; input2 / 10];
    [new_t1, new_t2]=manipilability_fuction(x_d);
    New_t1=new_t1*(180 / pi);
    New_t2=new_t2*(180 / pi);
    %% Send command
    cmdStr = sprintf("C%.2f,%.2f;", New_t1, New_t2);
    disp(['Sending command: ', cmdStr]);
    write(s, cmdStr, "string");
end

%% Callback function fo reading time series values from microcontroller
function readDataTimer(src, event)
global s hPlot c y1 y2 r1 r2
% Read the ASCII data from the serialport object.
    dataStr = readline(s)
    if isempty(dataStr)
       disp('No data recived');
       return;
    elseif dataStr == ""
        return;
    else
       disp(['Reading data: ', dataStr]);
    end
 % Parse data values from string and add accumulate into arrays
 % e.g. Arduino sending 2 series: c1,100
    data = sscanf(dataStr, "%c%f,%f");
    if length(data) ~= 3
        return;
    end
    c = [c, data(1)];
    y1 = [y1, data(2)]; 
    y2 = [y2, data(3)];

 % configure callback 
    configureCallback(s, "off");
%% forward kinematics calculation for plotting
   % forward_kinematics() function given the geometry and angles 
   T = forward_kinematics(r1, r2, y1(end), y2(end));
   % TODO retrieve end effector position and save for plotting
   x = T(1,4); 
   y = T(2,4);
  % real-time plotting
  cla(hPlot);
  hold(hPlot, 'on');
  colour = linspace(1,10,length(x));
  scatter(hPlot, x, y, 20, colour, 'filled');
  xlim([-(r1+r2),(r1+r2)]);
  ylim([-(r1+r2),(r1+r2)]);
  drawnow;
end

%% Callback function for closing the GUI
function closeGUI(~, ~)
global s hFig hTimer
    % Stop timer
    stop(hTimer);
    delete(hTimer);  
    % Close serial port
    delete(s);
    % Close GUI
    delete(hFig);
end