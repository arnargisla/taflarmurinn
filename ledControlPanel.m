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

% Last Modified by GUIDE v2.5 04-Apr-2016 06:24:16

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


% Update handles structure
guidata(hObject, handles);





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
%lc.turnLedOn();
%writeDigitalPin(a, ledPin, true);

% --- Executes on button press in offButton.
function offButton_Callback(hObject, eventdata, handles)
% hObject    handle to offButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
lc = getappdata(handles.figure1, 'lc');
lc.turnLedOff();
%writeDigitalPin(a, ledPin, false);