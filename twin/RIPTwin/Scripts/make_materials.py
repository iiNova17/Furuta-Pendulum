# Creates the twin's two materials. Run headless from the project folder:
#   UnrealEditor-Cmd.exe RIPTwin.uproject -run=pythonscript -script="<abs path>/Scripts/make_materials.py"
# M_Twin: opaque, two-sided; M_Ghost: translucent, two-sided. Parameters: Color (vector), Roughness, Metallic (+ Opacity on M_Ghost).
import unreal

PATH = '/Game/RIP'
tools = unreal.AssetToolsHelpers.get_asset_tools()
lib = unreal.MaterialEditingLibrary


def param(material, cls, name, value, x, y, prop):
    node = lib.create_material_expression(material, cls, x, y)
    node.set_editor_property('parameter_name', name)
    node.set_editor_property('default_value', value)
    lib.connect_material_property(node, '', prop)


def make(name, opacity=None):
    full = f'{PATH}/{name}'
    if unreal.EditorAssetLibrary.does_asset_exist(full):
        unreal.EditorAssetLibrary.delete_asset(full)
    m = tools.create_asset(name, PATH, unreal.Material, unreal.MaterialFactoryNew())
    m.set_editor_property('two_sided', True)
    param(m, unreal.MaterialExpressionVectorParameter, 'Color', unreal.LinearColor(0.8, 0.8, 0.8, 1.0), -400, 0,
          unreal.MaterialProperty.MP_BASE_COLOR)
    param(m, unreal.MaterialExpressionScalarParameter, 'Roughness', 0.45, -400, 200, unreal.MaterialProperty.MP_ROUGHNESS)
    param(m, unreal.MaterialExpressionScalarParameter, 'Metallic', 0.0, -400, 275, unreal.MaterialProperty.MP_METALLIC)
    if opacity is not None:
        m.set_editor_property('blend_mode', unreal.BlendMode.BLEND_TRANSLUCENT)
        param(m, unreal.MaterialExpressionScalarParameter, 'Opacity', opacity, -400, 350, unreal.MaterialProperty.MP_OPACITY)
    lib.recompile_material(m)
    unreal.EditorAssetLibrary.save_asset(full)
    unreal.log(f'RIPTwin: wrote {full}')


make('M_Twin')
make('M_Ghost', opacity=0.35)
