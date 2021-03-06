function varargout = ledControlPanel(varargin)
% LEDCONTROLPANEL MATLAB code for ledControlPanel.fig
%      LEDCONTROLPANEL, by itself, creates a new LEDCONTROLPANEL or raises the existing
%      singleton*.
%
%      H = LEDCONTROLPANEL returns the handle to a new LEDCONTROLPANEL or the handle to
%      the existing singleton*.
%
%      LEDCONTROLPANEL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LEDCONTROLPANEL.M with the given input arguments.
%
%      LEDCONTROLPANEL('Property','Value',...) creates a new LEDCONTROLPANEL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ledControlPanel_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ledControlPanel_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ledControlPanel

% Last Modified by GUIDE v2.5 12-Apr-2016 00:32:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ledControlPanel_OpeningFcn, ...
                   'gui_OutputFcn',  @ledControlPanel_OutputFcn, ...
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


% --- Executes just before ledControlPanel is made visible.
function ledControlPanel_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ledControlPanel (see VARARGIN)



% Choose default command line output for ledControlPanel
handles.output = hObject;

lc = varargin{2};
setappdata(handles.figure1, 'lc', lc);
setappdata(handles.figure1, 'mode', 1);

% Update handles structure
guidata(hObject, handles);


initializeLabels(handles);

% UIWAIT makes ledControlPanel wait for user response (see UIRESUME)
% uiwait(handles.figure1);




% --- Outputs from this function are returned to the command line.
function varargout = ledControlPanel_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in onButton.
function onButton_Callback(hObject, eventdata, handles)
% hObject    handle to onButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
lc = getappdata(handles.figure1, 'lc');
lc.turnLedOn();

% --- Executes on button press in offButton.
function offButton_Callback(hObject, eventdata, handles)
% hObject    handle to offButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
lc = getappdata(handles.figure1, 'lc');
lc.turnLedOff();

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
    setText(handles.slider_1_label, get(handles.slider_1, 'Value'));

function updateSlider2Label(handles)
    setText(handles.slider_2_label, get(handles.slider_2, 'Value'));

function updateSlider3Label(handles)
    setText(handles.slider_3_label, get(handles.slider_3, 'Value'));

function updateSlider4Label(handles)
    setText(handles.slider_4_label, get(handles.slider_4, 'Value'));

function updateSlider5Label(handles)
    setText(handles.slider_5_label, get(handles.slider_5, 'Value'));

function updateSlider6Label(handles)
    setText(handles.slider_6_label, get(handles.slider_6, 'Value'));
    
function updateModeLabel(handles)
    modeString = getModeString(handles);
    set(handles.mode_label, 'String', modeString);

function setText(textField, value)
    set(textField, 'String', num2str(value));


% --- Executes on slider movement.
function slider_1_Callback(hObject, eventdata, handles)
% hObject    handle to slider_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
updateSlider1Label(handles);

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


function toggleMode(handles)
if(getMode(handles) == 1)
    setappdata(handles.figure1, 'mode', 0);
else
    setappdata(handles.figure1, 'mode', 1);
end



function mode = getMode(handles)
mode = getappdata(handles.figure1, 'mode');

function modeString = getModeString(handles)
if(getMode(handles) == 1)
    modeString = 'Joint';
else
    modeString = 'World';
end
