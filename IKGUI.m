function varargout = IKGUI(varargin)
% IKGUI MATLAB code for IKGUI.fig
%      IKGUI, by itself, creates a new IKGUI or raises the existing
%      singleton*.
%
%      H = IKGUI returns the handle to a new IKGUI or the handle to
%      the existing singleton*.
%
%      IKGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IKGUI.M with the given input arguments.
%
%      IKGUI('Property','Value',...) creates a new IKGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before IKGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to IKGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help IKGUI

% Last Modified by GUIDE v2.5 02-Nov-2017 18:55:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @IKGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @IKGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before IKGUI is made visible.
function IKGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to IKGUI (see VARARGIN)

% Choose default command line output for IKGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes IKGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = IKGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function x_Callback(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x as text
%        str2double(get(hObject,'String')) returns contents of x as a double


% --- Executes during object creation, after setting all properties.
function x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_Callback(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y as text
%        str2double(get(hObject,'String')) returns contents of y as a double


% --- Executes during object creation, after setting all properties.
function y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_Callback(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z as text
%        str2double(get(hObject,'String')) returns contents of z as a double


% --- Executes during object creation, after setting all properties.
function z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in calculate.
function calculate_Callback(hObject, eventdata, handles)
% hObject    handle to calculate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x = str2num(char(get(handles.x,'String')));
y = str2num(char(get(handles.y,'String')));
z = str2num(char(get(handles.z,'String')));
if isempty(x) || x<-217 || x>217
    msgbox('The value X must be between -217 and 217','Error','error');
elseif isempty(y) || y<-217 || y>0
    msgbox('The value Y must be between -217 and 0','Error','error');
elseif isempty(z) || z<0 || z>40
    msgbox('The value Z must be between 0 and 40','Error','error');
else
    %% Denavit–Hartenberg parameters
    d1=75;      d2=0;    d3=0;     
    a1=0;       a2=105;  a3=96;    
    alfa1=pi/2; alfa2=0; alfa3=pi;
    %% Pythagoras theorem
    a=abs(x); b=abs(y);
    c=sqrt(a^2+b^2);
    alpha=atand(a/b);
    c_=c-30;
    a_=sind(alpha)*c_;
    b_=sqrt(c_^2-a_^2);
    if x>0
        x=a_;   
        y=-b_;   
        z=z+135;
    else
        x=-a_;   
        y=-b_; 
        z=z+135;
    end
    %% Inverse kinematics
    % theta3
    syms u3 'real';
    % substitution equations
    c3 = (1-u3^2)/(1+u3^2); 
    s3 = 2 * u3 / (1+u3^2);

    r = x^2+y^2+z^2;
    f1=a3*c3+a2;
    f2=a3*s3;
    k1=f1^2+f2^2-d1^2;
    %k1=a3^2+a2^2+2*a2*a3*c3-d1^2;
    u3 = double(solve(r-2*d1*z-k1));
    q33 = 2*atan(u3);
    q3=q33(2) ;

    %% theta2
    syms u2 'real';
    % substitution equations
    c2 = (1-u2^2)/(1+u2^2); 
    s2 = 2 * u2 / (1+u2^2);

    c3 = cos(q3);
    s3 = sin(q3);
    f1=a3*c3+a2;
    f2=a3*s3;
    k1=f1^2+f2^2-d1^2;
    k2=d1;

    u2 = double(solve(f1*s2+f2*c2+d1-z));
    q22 = 2*atan(u2);
    q2=q22(2);

    %% theta1
    c2 = cos(q2);
    s2 = sin(q2);
    g1 = (c2*f1)-(s2*f2)+a1;
    s1= y/g1;    
    c1= x/g1;        
    q1=atan2(s1,c1);

    %%
    % Direct kinematics
    H01 = [cos(q1) -sin(q1)*cos(alfa1) sin(q1)*sin(alfa1)  a1*cos(q1);
          sin(q1)  cos(q1)*cos(alfa1)  -cos(q1)*sin(alfa1) a1*sin(q1);
          0        sin(alfa1)          cos(alfa1)          d1;
          0        0                   0                   1
    ];

    H12 = [cos(q2) -sin(q2)*cos(alfa2) sin(q2)*sin(alfa2)  a2*cos(q2);
          sin(q2)  cos(q2)*cos(alfa2)  -cos(q2)*sin(alfa2) a2*sin(q2);
          0        sin(alfa2)          cos(alfa2)          d2;
          0        0                   0                  1
    ];

    H23 = [cos(q3) -sin(q3)*cos(alfa3) sin(q3)*sin(alfa3)  a3*cos(q3);
          sin(q3)  cos(q3)*cos(alfa3)  -cos(q3)*sin(alfa3) a3*sin(q3);
          0        sin(alfa3)          cos(alfa3)          d3;
          0        0                   0                   1
    ];

    H03=H01*H12*H23;
    R03=H03(1:3,1:3);

    % Position and angle of tool
    beta = pi/2;
    R06=[cos(beta)  -sin(beta)  0;
         sin(beta)  cos(beta)   0
         0          0          -1];
    R36=R03'*R06;

    q4=atan2(R36(1,3), -R36(2,3));    
    q5=atan2(R36(3,1), -R36(3,2));

    q=[q1 q2 q3 q4 q5]';
    
    handles.q1 = q1;
    handles.q2 = q2;
    handles.q3 = q3;
    handles.q4 = q4;
    handles.q5 = q5;
    
    guidata(hObject,handles);  % save the updated handles structure
    set(handles.solution,'String',num2str(radtodeg(q)));
end

function solution_Callback(hObject, eventdata, handles)
% hObject    handle to solution (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of solution as text
%        str2double(get(hObject,'String')) returns contents of solution as a double


% --- Executes during object creation, after setting all properties.
function solution_CreateFcn(hObject, eventdata, handles)
% hObject    handle to solution (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in move_robotic_arm.
function move_robotic_arm_Callback(hObject, eventdata, handles)
% hObject    handle to move_robotic_arm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Servo3
if isfield(handles,'q3')
    q3 = handles.q3;
    q3=radtodeg(q3)-10;
    q3=180/q3;
    q3=1/q3;
end

%Servo4
if isfield(handles,'q4')
    q4 = handles.q4;
    q4=radtodeg(q4);
    q4=180/q4;
    q4=1/q4;
    if q4<0
        q4=0;
    end
end

%Servo5
if isfield(handles,'q5')
    q5 = handles.q5;
    q5=radtodeg(q5);
    q5=180/q5;
    q5=1/q5;
    if q5<0
        q5=0;
    end
end

%Servo6
q6 = 0.7;
q6=radtodeg(q6);
q6=180/q6;
q6=1/q6;

% Servo 1.
if isfield(handles,'q1')
    q1 = handles.q1;
    q1=radtodeg(q1);
    q1=180/q1;
    q1=1/q1;
end

%Servo2
if isfield(handles,'q2')
    q2 = handles.q2;
    q2=radtodeg(q2)-10;
    q2=180/q2;
    q2=1/q2;
end
muveServo(q1,q2,q3,q4,q5,q6);
pause(1);
%muveServo2(1, 0.45, 0.4, 0, 0.5, 0.55);

% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on button press in reset_button.
function reset_button_Callback(hObject, eventdata, handles)
% hObject    handle to reset_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

muveServo(0.5,0.45,0.4,0,0.5,0.5);

% --- Executes on button press in scan_button.
function scan_button_Callback(hObject, eventdata, handles)
% hObject    handle to scan_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a;
global Sensor;
global servo7;
muveServo(0.5,0.45,0,0,0.5,0.7);

i = 1;
table = zeros(180,2);
for theta = 0 : 1/180 : 1
    writePosition(servo7, theta);
    dist1 = readTravelTime(Sensor)*170;
    pause(.01);
    dist2 = readTravelTime(Sensor)*170;
    dist = (dist1+dist2)/2;
    table(i,1) = (i-1);
    table(i,2) = round(dist * 100,2);
    i = i + 1;
end

j = 1; 
for theta = 1 : -1/180 : 0
    writePosition(servo7, theta);
    dist1 = readTravelTime(Sensor)*170;
    pause(.01);
    dist2 = readTravelTime(Sensor)*170;
    dist = (dist1+dist2)/2;
    table(i-j,2) = (table(i-j,2) + round(dist * 100,2))/2;
    j = j + 1;
end

polarplot (table(:,1)*pi/180, table (:,2));
title('Graf okoline');
thetalim([0 180]);
grid on;

% --- Executes on button press in connect_popupmenu.
function connect_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to connect_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function connect_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to connect_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in connect_button.
function connect_button_Callback(hObject, eventdata, handles)
% hObject    handle to connect_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Arduino connection
popStrings = handles.connect_popupmenu.String; % All the strings in the menu.
selectedIndex = handles.connect_popupmenu.Value;
selectedString = popStrings{selectedIndex};

delete(instrfind({'PORT'},{selectedString}));
clear a;
clear servo1; clear servo2; clear servo3;
global a;
clc;
a = arduino(selectedString, 'Mega2560', 'Libraries', {'JRodrigoTech/HCSR04', 'Servo'});
global servo1; 
servo1 = servo(a, 'D2');
global servo2;
servo2 = servo(a, 'D3');
global servo3;
servo3 = servo(a, 'D4');
global servo4;
servo4 = servo(a, 'D5');
global servo5;
servo5 = servo(a, 'D6');
global servo6;
servo6 = servo(a, 'D7');

global servo7;
servo7 = servo(a, 'D11');
global Sensor;
Sensor = addon(a, 'JRodrigoTech/HCSR04', 'D12', 'D13');
msgbox('Arduino successfully connected','Connected');

function muveServo(q1, q2, q3, q4, q5, q6);
global a;
global Sensor;
global servo7;
global servo1; global servo2; global servo3; global servo4; global servo5; global servo6;

%Servo3
postionServo3 = readPosition(servo3);
if postionServo3 < q3
    for theta = postionServo3 : 1/180 : q3
        writePosition(servo3, theta);
        pause(.04);
    end
elseif postionServo3 > q3
    for theta = postionServo3 : -1/180 : q3
        writePosition(servo3, theta);
        pause(.04);
    end
end
writePosition(servo3, q3);

%Servo4
postionServo4 = readPosition(servo4);
if postionServo4 < q4
    for theta = postionServo4 : 1/180 : q4
        writePosition(servo4, theta);
        pause(.04);
    end
elseif postionServo4 > q4
    for theta = postionServo4 : -1/180 : q4
        writePosition(servo4, theta);
        pause(.04);
    end
end
writePosition(servo4, q4);

%Servo5
postionServo5 = readPosition(servo5);
if postionServo5 < q5
    for theta = postionServo5 : 1/180 : q5
        writePosition(servo5, theta);
        pause(.04);
    end
elseif postionServo5 > q5
    for theta = postionServo5 : -1/180 : q5
        writePosition(servo5, theta);
        pause(.04);
    end
end
writePosition(servo5, q5);

%Servo6
 q6=0.7;
postionServo6 = readPosition(servo6);
if postionServo6 < q6
    for theta = postionServo6 : 1/180 : q6
        writePosition(servo6, theta);
        pause(.04);
    end
elseif postionServo6 > q6
    for theta = postionServo6 : -1/180 : q6
        writePosition(servo6, theta);
        pause(.04);
    end
end
writePosition(servo6, q6);

% Servo 1.
postionServo1 = readPosition(servo1);
if postionServo1 < q1
    for theta = postionServo1 : 1/180 : q1
        writePosition(servo1, theta);
        pause(.04);
    end
elseif postionServo1 > q1
    for theta = postionServo1 : -1/180 : q1
        writePosition(servo1, theta);
        pause(.04);
    end
end
writePosition(servo1, q1);

%Servo2
postionServo2 = readPosition(servo2);
if postionServo2 < q2
    for theta = postionServo2 : 1/180 : q2
        writePosition(servo2, theta);
        pause(.04);
    end
elseif postionServo2 > q2
    for theta = postionServo2 : -1/180 : q2
        writePosition(servo2, theta);
        pause(.04);
    end
end
writePosition(servo2, q2);





function muveServo2(q1, q2, q3, q4, q5, q6);
global a;
global Sensor;
global servo7;
global servo1; global servo2; global servo3; global servo4; global servo5; global servo6;

%Servo6
postionServo6 = readPosition(servo6);
if postionServo6 < q6
    for theta = postionServo6 : 1/180 : q6
        writePosition(servo6, theta);
        pause(.04);
    end
elseif postionServo6 > q6
    for theta = postionServo6 : -1/180 : q6
        writePosition(servo6, theta);
        pause(.04);
    end
end

%Servo2
postionServo2 = readPosition(servo2);
if postionServo2 < q2
    for theta = postionServo2 : 1/180 : q2
        writePosition(servo2, theta);
        pause(.04);
    end
elseif postionServo2 > q2
    for theta = postionServo2 : -1/180 : q2
        writePosition(servo2, theta);
        pause(.04);
    end
end
writePosition(servo2, q2);

%Servo3
postionServo3 = readPosition(servo3);
if postionServo3 < q3
    for theta = postionServo3 : 1/180 : q3
        writePosition(servo3, theta);
        pause(.04);
    end
elseif postionServo3 > q3
    for theta = postionServo3 : -1/180 : q3
        writePosition(servo3, theta);
        pause(.04);
    end
end
writePosition(servo3, q3);

%Servo4
postionServo4 = readPosition(servo4);
if postionServo4 < q4
    for theta = postionServo4 : 1/180 : q4
        writePosition(servo4, theta);
        pause(.04);
    end
elseif postionServo4 > q4
    for theta = postionServo4 : -1/180 : q4
        writePosition(servo4, theta);
        pause(.04);
    end
end
writePosition(servo4, q4);

%Servo5
postionServo5 = readPosition(servo5);
if postionServo5 < q5
    for theta = postionServo5 : 1/180 : q5
        writePosition(servo5, theta);
        pause(.04);
    end
elseif postionServo5 > q5
    for theta = postionServo5 : -1/180 : q5
        writePosition(servo5, theta);
        pause(.04);
    end
end
writePosition(servo5, q5);

writePosition(servo6, q6);

% Servo 1.
postionServo1 = readPosition(servo1);
if postionServo1 < q1
    for theta = postionServo1 : 1/180 : q1
        writePosition(servo1, theta);
        pause(.04);
    end
elseif postionServo1 > q1
    for theta = postionServo1 : -1/180 : q1
        writePosition(servo1, theta);
        pause(.04);
    end
end
writePosition(servo1, q1);

%Servo2
q2=0.50;
postionServo2 = readPosition(servo2);
if postionServo2 < q2
    for theta = postionServo2 : 1/180 : q2
        writePosition(servo2, theta);
        pause(.04);
    end
elseif postionServo2 > q2
    for theta = postionServo2 : -1/180 : q2
        writePosition(servo2, theta);
        pause(.04);
    end
end
writePosition(servo2, q2);

%Servo6
q6=0.3;
postionServo6 = readPosition(servo6);
if postionServo6 < q6
    for theta = postionServo6 : 1/180 : q6
        writePosition(servo6, theta);
        pause(.04);
    end
elseif postionServo6 > q6
    for theta = postionServo6 : -1/180 : q6
        writePosition(servo6, theta);
        pause(.04);
    end
end

%Servo2
q2=0.3;
postionServo2 = readPosition(servo2);
if postionServo2 < q2
    for theta = postionServo2 : 1/180 : q2
        writePosition(servo2, theta);
        pause(.04);
    end
elseif postionServo2 > q2
    for theta = postionServo2 : -1/180 : q2
        writePosition(servo2, theta);
        pause(.04);
    end
end
writePosition(servo2, q2);

