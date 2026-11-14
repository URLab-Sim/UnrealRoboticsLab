#include "MJSceneComponents.h"
#include "GameFramework/Actor.h"

FString FMJElementRef::ResolveName(const AActor* OwnerActor) const
{
	if (OwnerActor && (ComponentRef.ComponentProperty != NAME_None || !ComponentRef.PathToComponent.IsEmpty()))
	{
		if (UActorComponent* Comp = ComponentRef.GetComponent(const_cast<AActor*>(OwnerActor)))
		{
			if (const UMJSceneComponent* MJScene = Cast<UMJSceneComponent>(Comp))
			{
				return MJScene->GetMJName();
			}
			return Comp->GetName();
		}
	}
	if (!AssetRef.IsNull())
	{
		return AssetRef.GetAssetName();
	}
	return FallbackName;
}
