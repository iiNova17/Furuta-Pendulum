function set_controller(mdl, kind, name)
%SET_CONTROLLER Put a controller into the harness's Controller slot (model loaded, not saved).
%   kind 'system'    : name is a FurutaController subclass (MATLAB System block, P = ctrlP)
%   kind 'subsystem' : name is a Simulink subsystem file (.slx made with new_system(name,'Subsystem'))
%                      with inports y (3x1), r and outports duty, mode, dbg (4x1). Its blocks can read
%                      the struct ctrlP and hcfg (e.g. hcfg.Ts as sample time).
slot = [mdl '/Controller'];
impl = [slot '/impl'];
if getSimulinkBlockHandle(impl) ~= -1
    lh = get_param(impl, 'LineHandles');
    h = [lh.Inport, lh.Outport];
    delete_line(h(h ~= -1));
    delete_block(impl);
end
switch kind
    case 'system'
        add_block('simulink/User-Defined Functions/MATLAB System', impl, 'Position', [150 30 300 120]);
        set_param(impl, 'System', name);
        set_param(impl, 'Ts', 'hcfg.Ts', 'P', 'ctrlP', 'SimulateUsing', 'Interpreted execution');
    case 'subsystem'
        add_block('simulink/Ports & Subsystems/Subsystem Reference', impl, ...
            'Position', [150 30 300 120]);
        set_param(impl, 'ReferencedSubsystem', name);
    otherwise
        error('set_controller:kind', 'kind must be ''system'' or ''subsystem''');
end
add_line(slot, 'y/1', 'impl/1');
add_line(slot, 'r/1', 'impl/2');
add_line(slot, 'impl/1', 'duty/1');
add_line(slot, 'impl/2', 'mode/1');
add_line(slot, 'impl/3', 'dbg/1');
end
