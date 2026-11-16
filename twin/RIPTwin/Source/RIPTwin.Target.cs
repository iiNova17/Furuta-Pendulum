using UnrealBuildTool;

public class RIPTwinTarget : TargetRules
{
	public RIPTwinTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V4;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_3;
		// UE 5.3 predates the VS 18 toolchain that is also installed here: pin VS 2022
		WindowsPlatform.Compiler = WindowsCompiler.VisualStudio2022;
		WindowsPlatform.CompilerVersion = "14.40.33807";
		ExtraModuleNames.Add("RIPTwin");
	}
}
