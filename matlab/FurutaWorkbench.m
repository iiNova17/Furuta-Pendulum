function app = FurutaWorkbench(varargin)
%FURUTAWORKBENCH Launch the disconnected experiment workbench.
% app = FurutaWorkbench(); app = FurutaWorkbench('Visible','off');
app = furuta.WorkbenchApp(varargin{:});
end
