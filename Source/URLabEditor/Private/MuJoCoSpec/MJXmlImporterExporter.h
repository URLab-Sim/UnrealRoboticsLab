#pragma once

#include "CoreMinimal.h"
#include "mujoco/mujoco.h"

class AActor;
class UBlueprint;

class FMJXmlImporterExporter
{
public:
	static bool ImportXMLToActor(const FString& Xml, AActor* TargetActor, const mjVFS* Vfs, FString& OutError);
	static bool ImportXMLToBlueprintSCS(const FString& Xml, UBlueprint* Blueprint, const mjVFS* Vfs, FString& OutError);
	static bool ExportActorToXML(AActor* SourceActor, FString& OutXml, FString& OutError);
};
