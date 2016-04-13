function varargout = robotControlPanel(varargin)
% ROBOTCONTROLPANEL MATLAB code for robotControlPanel.fig
%      ROBOTCONTROLPANEL, by itself, creates a new ROBOTCONTROLPANEL or raises the existing
%      singleton*.
%
%      H = ROBOTCONTROLPANEL returns the handle to a new ROBOTCONTROLPANEL or the handle to
%      the existing singleton*.
%
%      ROBOTCONTROLPANEL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOTCONTROLPANEL.M with the given input arguments.
%
%      ROBOTCONTROLPANEL('Property','Value',...) creates a new ROBOTCONTROLPANEL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before robotControlPanel_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to robotControlPanel_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help robotControlPanel

% Last Modified by GUIDE v2.5 13-Apr-2016 13:39:04

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @robotControlPanel_OpeningFcn, ...
                   'gui_OutputFcn',  @robotControlPanel_OutputFcn, ...
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


% --- Executes just before robotControlPanel is made visible.
function robotControlPanel_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to robotControlPanel (see VARARGIN)

% Choose default command line output for robotControlPanel
handles.output = hObject;

controller = varargin{2};
setappdata(handles.figure1, 'controller', controller);
setappdata(handles.figure1, 'mode', 1);

% Update handles structure
guidata(hObject, handles);

initializeSliders(handles);

% UIWAIT makes robotControlPanel wait for user response (see UIRESUME)
% uiwait(handles.figure1);




% --- Outputs from this function are returned to the command line.
function varargout = robotControlPanel_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function initializeSliders(handles)
    controller = getappdata(handles.figure1, 'controller');
    positions = controller.getJointPositions();
    initializeSlider(handles.slider_1, 0, 180, positions(1));
    initializeSlider(handles.slider_2, 0, 180, positions(2));
    initializeSlider(handles.slider_3, 0, 180, positions(3));
    initializeSlider(handles.slider_4, 0, 180, positions(4));
    initializeSlider(handles.slider_5, 0, 180, positions(5));
    initializeSlider(handles.slider_6, 0, 180, positions(6));
    initializeLabels(handles);


function initializeSlider(slider, min, max, value)
    set(slider, ...
        'Min', min, ...
        'Max', max, ...
        'Value', value,...
        'SliderStep', [1/180, 30/360]);

function initializeLabels(handles)
    initializeSliderLabels(handles);
    updateModeLabel(handles);
    
function initializeSliderLabels(handles)
    updateSlider1Label(handles);
    updateSlider2Label(handles);
    updateSlider3Label(handles);
    updateSlider4Label(handles);
    updateSlider5Label(handles);
    updateSlider6Label(handles);

function updateSlider1Label(handles)
    setText(handles.slider_1_label, getSliderValue(handles.slider_1));

function updateSlider2Label(handles)
    setText(handles.slider_2_label, getSliderValue(handles.slider_2));

function updateSlider3Label(handles)
    setText(handles.slider_3_label, getSliderValue(handles.slider_3));

function updateSlider4Label(handles)
    setText(handles.slider_4_label, getSliderValue(handles.slider_4));

function updateSlider5Label(handles)
    setText(handles.slider_5_label, getSliderValue(handles.slider_5));

function updateSlider6Label(handles)
    setText(handles.slider_6_label, getSliderValue(handles.slider_6));
    
function updateModeLabel(handles)
    modeString = getModeString(handles);
    set(handles.mode_label, 'String', modeString);

function setText(textField, value)
    set(textField, 'String', num2str(value));

function value = getSliderValue(slider)
    value = round(get(slider, 'Value'));
    

function toggleMode(handles)
    if(getMode(handles) == 1)
        setappdata(handles.figure1, 'mode', 0);
    else
        setappdata(handles.figure1, 'mode', 1);
    end

function updateRobot(handles)
    pos(1) = getSliderValue(handles.slider_1);
    pos(2) = getSliderValue(handles.slider_2);
    pos(3) = getSliderValue(handles.slider_3);
    pos(4) = getSliderValue(handles.slider_4);
    pos(5) = getSliderValue(handles.slider_5);
    pos(6) = getSliderValue(handles.slider_6);
    controller = getappdata(handles.figure1, 'controller');
    controller.setJointPositions(pos);
    controller.moveRobot();

function mode = getMode(handles)
    mode = getappdata(handles.figure1, 'mode');

function modeString = getModeString(handles)
    if(getMode(handles) == 1)
        modeString = 'joint';
    else
        modeString = 'world';
    end

% --- Executes on slider movement.
function slider_1_Callback(hObject, eventdata, handles)
% hObject    handle to slider_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
updateSlider1Label(handles);
updateRobot(handles);

% --- Executes during object creation, after setting all properties.
function slider_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_2_Callback(hObject, eventdata, handles)
% hObject    handle to slider_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
updateSlider2Label(handles);
updateRobot(handles);


% --- Executes during object creation, after setting all properties.
function slider_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_3_Callback(hObject, eventdata, handles)
% hObject    handle to slider_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
updateSlider3Label(handles);
updateRobot(handles);


% --- Executes during object creation, after setting all properties.
function slider_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_4_Callback(hObject, eventdata, handles)
% hObject    handle to slider_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
updateSlider4Label(handles);
updateRobot(handles);


% --- Executes during object creation, after setting all properties.
function slider_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_5_Callback(hObject, eventdata, handles)
% hObject    handle to slider_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
updateSlider5Label(handles);
updateRobot(handles);


% --- Executes during object creation, after setting all properties.
function slider_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_6_Callback(hObject, eventdata, handles)
% hObject    handle to slider_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
updateSlider6Label(handles);
updateRobot(handles);


% --- Executes during object creation, after setting all properties.
function slider_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in mode_pushbutton.
function mode_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to mode_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
toggleMode(handles);
updateModeLabel(handles);



% --- Executes on button press in onButton.
function onButton_Callback(hObject, eventdata, handles)
% hObject    handle to onButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controller = getappdata(handles.figure1, 'controller');
controller.turnLedOn();

% --- Executes on button press in offButton.
function offButton_Callback(hObject, eventdata, handles)
% hObject    handle to offButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controller = getappdata(handles.figure1, 'controller');
controller.turnLedOff();
