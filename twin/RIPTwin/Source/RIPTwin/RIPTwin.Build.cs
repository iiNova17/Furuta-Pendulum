using UnrealBuildTool;

public class RIPTwin : ModuleRules
{
	public RIPTwin(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] {
			"Core", "CoreUObject", "Engine", "InputCore",
			"ProceduralMeshComponent", "Sockets", "Networking", "Json"
		});
	}
}
