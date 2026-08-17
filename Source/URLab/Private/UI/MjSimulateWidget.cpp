// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "UI/MjSimulateWidget.h"
#include "UI/MjPropertyRow.h"
#include "UI/MjCameraFeedEntry.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "Transport/NetworkManager.h"
#include "Bridge/RpcDispatcher.h"
#include "Utils/URLabLogging.h"
#include "Components/VerticalBox.h"
#include "Components/VerticalBoxSlot.h"
#include "Components/TextBlock.h"
#include "Components/Button.h"
#include "Components/ComboBoxString.h"
#include "Components/ExpandableArea.h"
#include "Components/HorizontalBox.h"
#include "Components/HorizontalBoxSlot.h"
#include "Components/SizeBox.h"
#include "Components/Spacer.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Entity/MjEntity.h"
#include "MuJoCo/Entity/MjEntityActor.h"
#include "MuJoCo/Entity/MjEntityPawn.h"
#include "MuJoCo/Entity/MjEntityMembers.h"
#include "MuJoCo/Entity/MjControl.h"
#include "MuJoCo/Entity/MjControlIngress.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "State/MjCanonicalName.h"
#include "Styling/SlateTypes.h"
#include "Fonts/SlateFontInfo.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Replay/MjReplayManager.h"
#include "Kismet/GameplayStatics.h"
#include "Components/CheckBox.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
	/** The compiled entity carrying this stable name, or null. */
	const FMjEntity* FindEntity(const UMjPhysicsEngine* Engine, FName EntityName)
	{
		if (Engine == nullptr || EntityName.IsNone())
		{
			return nullptr;
		}
		for (const FMjEntity& E : Engine->GetEntityPartition())
		{
			if (E.Name == EntityName)
			{
				return &E;
			}
		}
		return nullptr;
	}

	/** The short name a member is displayed and addressed by: the compiled name with the entity's
	 *  "<name>_" prefix stripped (matching how the scene assembly attaches a participant). */
	FString ShortMemberName(FName EntityName, const FString& Compiled)
	{
		if (EntityName.IsNone())
		{
			return Compiled;
		}
		const FString Prefix = EntityName.ToString() + TEXT("_");
		return Compiled.StartsWith(Prefix) ? Compiled.RightChop(Prefix.Len()) : Compiled;
	}

	FString CompiledNameOf(const mjModel* Model, int32 ObjType, int32 Id)
	{
		const char* N = mj_id2name(Model, ObjType, Id);
		return N ? FString(UTF8_TO_TCHAR(N)) : FString();
	}

	/** The twist controller on the entity's possess pawn (re-homed off the articulation at handoff),
	 *  or null when the entity has no pawn. */
	UMjTwistController* FindEntityTwist(UWorld* World, FName EntityName)
	{
		if (World == nullptr || EntityName.IsNone())
		{
			return nullptr;
		}
		TArray<AActor*> Pawns;
		UGameplayStatics::GetAllActorsOfClass(World, AMjEntityPawn::StaticClass(), Pawns);
		for (AActor* PawnActor : Pawns)
		{
			AMjEntityPawn* Pawn = Cast<AMjEntityPawn>(PawnActor);
			if (Pawn && Pawn->OwnerEntityName == EntityName)
			{
				return Pawn->FindComponentByClass<UMjTwistController>();
			}
		}
		return nullptr;
	}

	/** Load a scene keyframe into the live data as MuJoCo simulate does: mj_resetDataKeyframe sets the
	 *  whole model (qpos including free joints, qvel, act, ctrl, mocap) then forwards, and ForwardSync
	 *  publishes the result. */
	void ApplyKeyframeReset(UMjPhysicsEngine* Engine, int32 KeyId)
	{
		mjModel* Model = Engine ? Engine->GetModel() : nullptr;
		mjData* Data = Engine ? Engine->GetData() : nullptr;
		if (Model == nullptr || Data == nullptr || KeyId < 0 || KeyId >= Model->nkey)
		{
			return;
		}
		mj_resetDataKeyframe(Model, Data, KeyId);
		// Clear the setpoint store so the next pre-step drain does not overwrite the keyframe's ctrl.
		Engine->ClearControlBuffer();
		Engine->ForwardSync();
	}

	/** The scene keyframes that belong to an entity, as (display name, key id) pairs: those whose
	 *  compiled name carries the entity's prefix, with the prefix stripped for display. When none
	 *  carry it (a single-entity or raw scene where keys are unprefixed) every keyframe is offered. */
	TArray<TPair<FString, int32>> EntityKeyframes(const mjModel* Model, FName EntityName)
	{
		TArray<TPair<FString, int32>> Out;
		if (Model == nullptr)
		{
			return Out;
		}
		const FString Prefix = EntityName.IsNone() ? FString() : (EntityName.ToString() + TEXT("_"));
		TArray<TPair<FString, int32>> Prefixed;
		TArray<TPair<FString, int32>> All;
		for (int32 K = 0; K < Model->nkey; ++K)
		{
			const FString Name = CompiledNameOf(Model, mjOBJ_KEY, K);
			const FString Display = Name.IsEmpty() ? FString::Printf(TEXT("key_%d"), K) : Name;
			All.Add(TPair<FString, int32>(Display, K));
			if (!Prefix.IsEmpty() && Name.StartsWith(Prefix))
			{
				Prefixed.Add(TPair<FString, int32>(Name.RightChop(Prefix.Len()), K));
			}
		}
		return Prefixed.Num() > 0 ? Prefixed : All;
	}
} // namespace

void UMjSimulateWidget::NativeConstruct()
{
	Super::NativeConstruct();

	auto StyleButton = [](UButton* Btn, FLinearColor BGColor) {
		if (!Btn)
			return;
		FButtonStyle Style = Btn->GetStyle();
		Style.Normal.TintColor = FSlateColor(BGColor);
		Style.Hovered.TintColor = FSlateColor(BGColor * 1.5f);
		Style.Pressed.TintColor = FSlateColor(BGColor * 0.5f);
		Btn->SetStyle(Style);
	};

	StyleButton(PlayPauseButton, FLinearColor(0.2f, 0.6f, 0.2f, 0.9f));
	StyleButton(ResetButton, FLinearColor(0.6f, 0.2f, 0.2f, 0.9f));
	StyleButton(RecordButton, FLinearColor(0.8f, 0.3f, 0.1f, 0.9f));
	StyleButton(ReplayButton, FLinearColor(0.2f, 0.4f, 0.8f, 0.9f));
	StyleButton(SnapshotButton, FLinearColor(0.2f, 0.6f, 0.8f, 0.9f));
	StyleButton(RestoreButton, FLinearColor(0.2f, 0.6f, 0.8f, 0.9f));

	if (TimeText)
	{
		FSlateFontInfo FontInfo = TimeText->GetFont();
		FontInfo.Size = 14;
		FontInfo.TypefaceFontName = TEXT("Bold");
		TimeText->SetFont(FontInfo);
		TimeText->SetColorAndOpacity(FSlateColor(FLinearColor(1.0f, 0.9f, 0.3f, 1.0f)));
	}

	// Shrink Button Fonts
	if (PlayPauseButton)
	{
		if (UTextBlock* BtnText = Cast<UTextBlock>(PlayPauseButton->GetChildAt(0)))
		{
			FSlateFontInfo FontInfo = BtnText->GetFont();
			FontInfo.Size = 14;
			FontInfo.TypefaceFontName = TEXT("Bold");
			BtnText->SetFont(FontInfo);
		}
	}

	if (ResetButton)
	{
		if (UTextBlock* BtnText = Cast<UTextBlock>(ResetButton->GetChildAt(0)))
		{
			FSlateFontInfo FontInfo = BtnText->GetFont();
			FontInfo.Size = 14;
			FontInfo.TypefaceFontName = TEXT("Bold");
			BtnText->SetFont(FontInfo);
		}
	}

	// Dynamic Top Bar Layout
	if (TimeText && PlayPauseButton && ResetButton && ArticulationSelector)
	{
		// Find the top bar horizontal box
		if (UHorizontalBox* TopBar = Cast<UHorizontalBox>(TimeText->GetParent()))
		{
			// Remove everything to rebuild cleanly
			TimeText->RemoveFromParent();
			PlayPauseButton->RemoveFromParent();
			ResetButton->RemoveFromParent();
			ArticulationSelector->RemoveFromParent();
			if (PossessButton)
				PossessButton->RemoveFromParent();

			TopBar->ClearChildren();

			// Add TimeText
			if (UHorizontalBoxSlot* HSlot = TopBar->AddChildToHorizontalBox(TimeText))
			{
				HSlot->SetPadding(FMargin(10, 5, 20, 5)); // Give text breathing room
				HSlot->SetVerticalAlignment(VAlign_Center);
			}

			// Both buttons in identically-sized SizeBoxes
			const float ButtonWidth = 90.0f;
			const float ButtonHeight = 30.0f;

			USizeBox* PlaySizeBox = NewObject<USizeBox>(this);
			PlaySizeBox->SetWidthOverride(ButtonWidth);
			PlaySizeBox->SetHeightOverride(ButtonHeight);

			PlaySizeBox->AddChild(PlayPauseButton);
			if (UHorizontalBoxSlot* HSlot = TopBar->AddChildToHorizontalBox(PlaySizeBox))
			{
				HSlot->SetPadding(FMargin(0, 0, 5, 0));
				HSlot->SetVerticalAlignment(VAlign_Center);
			}

			USizeBox* ResetSizeBox = NewObject<USizeBox>(this);
			ResetSizeBox->SetWidthOverride(ButtonWidth);
			ResetSizeBox->SetHeightOverride(ButtonHeight);

			ResetSizeBox->AddChild(ResetButton);
			if (UHorizontalBoxSlot* HSlot = TopBar->AddChildToHorizontalBox(ResetSizeBox))
			{
				HSlot->SetPadding(FMargin(0, 0, 10, 0));
				HSlot->SetVerticalAlignment(VAlign_Center);
			}

			// Add a Spacer to eat up middle room
			USpacer* TopSpacer = NewObject<USpacer>(this);
			if (UHorizontalBoxSlot* HSlot = TopBar->AddChildToHorizontalBox(TopSpacer))
			{
				HSlot->SetSize(FSlateChildSize(ESlateSizeRule::Fill));
			}

			// Label for articulation selector
			UTextBlock* SelectorLabel = NewObject<UTextBlock>(this);
			SelectorLabel->SetText(FText::FromString(TEXT("Entity:")));
			SelectorLabel->SetColorAndOpacity(FSlateColor(FLinearColor::White));
			if (UHorizontalBoxSlot* LabelSlot = TopBar->AddChildToHorizontalBox(SelectorLabel))
			{
				LabelSlot->SetPadding(FMargin(10, 5, 2, 5));
				LabelSlot->SetVerticalAlignment(VAlign_Center);
			}

			if (UHorizontalBoxSlot* HSlot = TopBar->AddChildToHorizontalBox(ArticulationSelector))
			{
				HSlot->SetPadding(FMargin(10, 5, 5, 5));
				HSlot->SetVerticalAlignment(VAlign_Center);
			}

			// Possess button next to selector
			if (PossessButton)
			{
				USizeBox* PossessSizeBox = NewObject<USizeBox>(this);
				PossessSizeBox->SetWidthOverride(ButtonWidth);
				PossessSizeBox->SetHeightOverride(ButtonHeight);
				PossessSizeBox->AddChild(PossessButton);
				if (UHorizontalBoxSlot* HSlot = TopBar->AddChildToHorizontalBox(PossessSizeBox))
				{
					HSlot->SetPadding(FMargin(0, 0, 10, 0));
					HSlot->SetVerticalAlignment(VAlign_Center);
				}
			}
		}
	}

	if (PlayPauseButton)
	{
		PlayPauseButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::OnPlayPauseClicked);
	}

	if (ResetButton)
	{
		ResetButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::OnResetClicked);
	}

	if (RecordButton)
	{
		RecordButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::OnRecordClicked);
	}

	if (ReplayButton)
	{
		ReplayButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::OnReplayClicked);
	}

	if (SnapshotButton)
	{
		SnapshotButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::OnSnapshotClicked);
	}

	if (RestoreButton)
	{
		RestoreButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::OnRestoreClicked);
	}

	if (PossessButton)
	{
		StyleButton(PossessButton, FLinearColor(0.1f, 0.6f, 0.6f, 0.9f));
		if (UTextBlock* BtnText = Cast<UTextBlock>(PossessButton->GetChildAt(0)))
		{
			BtnText->SetText(FText::FromString(TEXT("Possess")));
			FSlateFontInfo BtnFont = BtnText->GetFont();
			BtnFont.Size = 9;
			BtnText->SetFont(BtnFont);
			BtnText->SetAutoWrapText(false);
			BtnText->SetJustification(ETextJustify::Center);
		}
		PossessButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::OnPossessClicked);
	}

	if (ArticulationSelector)
	{
		ArticulationSelector->OnSelectionChanged.AddDynamic(this, &UMjSimulateWidget::OnArticulationSelected);
		FTableRowStyle RowStyle = ArticulationSelector->GetItemStyle();
		FSlateColor RowBG(FLinearColor(0.15f, 0.15f, 0.18f, 1.0f));
		FSlateColor RowHover(FLinearColor(0.25f, 0.30f, 0.35f, 1.0f));
		RowStyle.SetEvenRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
		RowStyle.SetOddRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
		RowStyle.SetEvenRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
		RowStyle.SetOddRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
		ArticulationSelector->SetItemStyle(RowStyle);
	}

	// Attempt to find manager if not initialized
	if (!ManagerRef)
	{
		SetupDashboard(AAMjManager::GetManager());
	}
}

void UMjSimulateWidget::SetupDashboard(AAMjManager* InManager)
{
	ManagerRef = InManager;
	if (!ManagerRef)
		return;

	PopulateManagerSettings();

	// Populate the entity selector from the compiled partition (the wire/display name is PublicName).
	if (ArticulationSelector)
	{
		ArticulationSelector->ClearOptions();
		int32 EntityCount = 0;
		if (ManagerRef->PhysicsEngine)
		{
			for (const FMjEntity& E : ManagerRef->PhysicsEngine->GetEntityPartition())
			{
				ArticulationSelector->AddOption(MjUtils::PrettifyName(E.PublicName.ToString()));
				++EntityCount;
			}
		}
		if (EntityCount > 0)
		{
			ArticulationSelector->SetSelectedIndex(0);
		}
		else
		{
			// If no entities found, we still need to build the base UI (Physics, Visuals, Replay etc)
			RefreshArticulationControls();
		}
	}
}

void UMjSimulateWidget::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
	Super::NativeTick(MyGeometry, InDeltaTime);

	if (!ManagerRef)
	{
		SetupDashboard(AAMjManager::GetManager());
		if (!ManagerRef)
			return;

		// Start with UI hidden — user presses Tab to show
		if (!bIsMouseEnabled)
		{
			if (UWidget* RootCanvas = Cast<UWidget>(GetRootWidget()))
			{
				RootCanvas->SetVisibility(ESlateVisibility::Hidden);
			}
			if (APlayerController* PC = GetOwningPlayer())
			{
				PC->bShowMouseCursor = false;
				PC->SetInputMode(FInputModeGameOnly());
			}
		}
	}

	if (TimeText)
	{
		// Read active step mode from the dispatcher (if present). Surfaces
		// live / direct / puppet in packaged builds where the editor
		// toolbar pill (SMjStepModeIndicator) isn't visible.
		FString ModeStr;
		if (FURLabRpcDispatcher* Disp = ManagerRef->GetStepDispatcher())
		{
			switch (Disp->GetActiveStepMode())
			{
				case EMjPoseSource::FreeRun:
					ModeStr = TEXT("live");
					break;
				case EMjPoseSource::Stepped:
					ModeStr = TEXT("direct");
					break;
				case EMjPoseSource::StatePushed:
					ModeStr = TEXT("puppet");
					break;
				case EMjPoseSource::Mirror:
					ModeStr = TEXT("mirror");
					break;
			}
		}
		const FString ModeSuffix = ModeStr.IsEmpty()
									 ? FString()
									 : FString::Printf(TEXT("  [%s]"), *ModeStr);

		// Show replay time if replaying, otherwise sim time
		AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
			UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
		if (ReplayMgr && ReplayMgr->bIsReplaying)
		{
			TArray<FMjReplayFrame>& Frames = ReplayMgr->Sessions.FindOrAdd(ReplayMgr->ActiveSessionName).Frames;
			float Duration = Frames.Num() > 0 ? Frames.Last().Timestamp - Frames[0].Timestamp : 0.0f;
			TimeText->SetText(FText::FromString(FString::Printf(TEXT("REPLAY: %.2f / %.2f s  [%s]%s"),
				ReplayMgr->PlaybackTime, Duration, *ReplayMgr->ActiveSessionName, *ModeSuffix)));
		}
		else
		{
			TimeText->SetText(FText::FromString(FString::Printf(TEXT("Time: %.3f s%s"),
				ManagerRef->GetSimTime(), *ModeSuffix)));
		}
	}

	// Locomotion section is now built inline in RefreshArticulationControls

	// Update Play/Pause Button Text based on real manager state
	if (PlayPauseButton)
	{
		if (UTextBlock* BtnText = Cast<UTextBlock>(PlayPauseButton->GetChildAt(0)))
		{
			bool bPaused = ManagerRef->PhysicsEngine ? ManagerRef->PhysicsEngine->bIsPaused : true;
			BtnText->SetText(FText::FromString(bPaused ? TEXT("Play") : TEXT("Pause")));
		}
	}

	// Update record/replay button text based on ReplayManager state
	{
		AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
			UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));

		if (RecordButton)
		{
			if (UTextBlock* BtnText = Cast<UTextBlock>(RecordButton->GetChildAt(0)))
			{
				bool bRecording = ReplayMgr && ReplayMgr->bIsRecording;
				BtnText->SetText(FText::FromString(bRecording ? TEXT("Stop Recording") : TEXT("Record")));
			}
		}

		if (ReplayButton)
		{
			if (UTextBlock* BtnText = Cast<UTextBlock>(ReplayButton->GetChildAt(0)))
			{
				bool bReplaying = ReplayMgr && ReplayMgr->bIsReplaying;
				BtnText->SetText(FText::FromString(bReplaying ? TEXT("Stop Replay") : TEXT("Replay")));
			}
		}
	}

	if (SnapshotButton)
	{
		if (UTextBlock* BtnText = Cast<UTextBlock>(SnapshotButton->GetChildAt(0)))
		{
			BtnText->SetText(FText::FromString(TEXT("Snapshot")));
		}
	}

	if (RestoreButton)
	{
		if (UTextBlock* BtnText = Cast<UTextBlock>(RestoreButton->GetChildAt(0)))
		{
			BtnText->SetText(FText::FromString(TEXT("Restore")));
		}
	}

	// Refresh session dropdown and binding UI if session count or bindings changed
	if (ReplaySessionSelector)
	{
		AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
			UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
		if (ReplayMgr)
		{
			bool bNeedsRefresh = false;
			if (ReplayMgr->GetSessionCount() != CachedSessionCount)
			{
				bNeedsRefresh = true;
			}
			// Check if binding count changed (e.g. after CSV load)
			if (ReplayMgr->GetArticulationBindings().Num() != ReplayEnabledCheckBoxes.Num())
			{
				bNeedsRefresh = true;
			}
			if (bNeedsRefresh)
			{
				RefreshArticulationControls();
			}
		}
	}

	// Toggle input mode via Tab key
	if (APlayerController* PC = GetOwningPlayer())
	{
		if (PC->WasInputKeyJustPressed(EKeys::Tab))
		{
			bIsMouseEnabled = !bIsMouseEnabled;
			PC->bShowMouseCursor = bIsMouseEnabled;

			// We only hide the root inner panel of the UI, rather than the entire Widget itself.
			// If we hide the whole Widget, NativeTick stops firing and Tab can never wake it back up!
			if (UWidget* RootCanvas = Cast<UWidget>(GetRootWidget()))
			{
				if (bIsMouseEnabled)
				{
					RootCanvas->SetVisibility(ESlateVisibility::Visible);
					FInputModeGameAndUI InputMode;
					InputMode.SetWidgetToFocus(TakeWidget());
					PC->SetInputMode(InputMode);
				}
				else
				{
					RootCanvas->SetVisibility(ESlateVisibility::Hidden);
					FInputModeGameOnly InputMode;
					PC->SetInputMode(InputMode);
				}
			}
		}
	}

	// Periodically update monitor values
	UpdateMonitorValues();

	// Update live camera feed thumbnails
	for (UMjCameraFeedEntry* Feed : ActiveCameraFeeds)
	{
		if (Feed)
			Feed->UpdateFeed();
	}
}

void UMjSimulateWidget::PopulateManagerSettings()
{
	if (!PropertyRowClass)
	{
		UE_LOG(LogURLab, Warning, TEXT("MjSimulateWidget: PropertyRowClass is NOT SET in BP Class Defaults."));
		return;
	}
	if (!ManagerRef)
		return;

	if (ManagerSettingsList)
	{
		ManagerSettingsList->ClearChildren();
		ManagerSettingsList->SetVisibility(ESlateVisibility::Visible); // Re-show left panel
	}
}

void UMjSimulateWidget::OnPlayPauseClicked()
{
	if (ManagerRef)
	{
		if (ManagerRef->PhysicsEngine)
			ManagerRef->PhysicsEngine->SetPaused(!ManagerRef->PhysicsEngine->bIsPaused);
	}
}

void UMjSimulateWidget::OnResetClicked()
{
	if (ManagerRef)
	{
		ManagerRef->ResetSimulation();
	}
}

void UMjSimulateWidget::OnRecordClicked()
{
	if (!ManagerRef)
		return;
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
		UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (!ReplayMgr)
		return;

	if (ReplayMgr->bIsRecording)
	{
		ManagerRef->StopRecording();
	}
	else
	{
		ManagerRef->StartRecording();
	}
}

void UMjSimulateWidget::OnReplayClicked()
{
	if (!ManagerRef)
		return;
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
		UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (!ReplayMgr)
		return;

	if (ReplayMgr->bIsReplaying)
	{
		ManagerRef->StopReplay();
	}
	else
	{
		ManagerRef->StartReplay();
	}
}

void UMjSimulateWidget::OnSnapshotClicked()
{
	if (ManagerRef)
	{
		LastSnapshot = ManagerRef->CaptureSnapshot();
	}
}

void UMjSimulateWidget::OnRestoreClicked()
{
	if (ManagerRef && LastSnapshot)
	{
		ManagerRef->RestoreSnapshot(LastSnapshot);
	}
}

void UMjSimulateWidget::OnReplaySessionSelected(FString SelectedItem, ESelectInfo::Type SelectionType)
{
	if (!ManagerRef)
		return;
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
		UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->SetActiveSession(SelectedItem);
	}
}

void UMjSimulateWidget::OnLoadCSVClicked()
{
	UE_LOG(LogURLab, Log, TEXT("MjSimulateWidget: Load Replay clicked"));
	if (!ManagerRef)
	{
		UE_LOG(LogURLab, Warning, TEXT("MjSimulateWidget: No ManagerRef"));
		return;
	}
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
		UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (!ReplayMgr)
	{
		UE_LOG(LogURLab, Warning, TEXT("MjSimulateWidget: No AMjReplayManager in scene! Add one to the level."));
		return;
	}
	if (ReplayMgr->BrowseAndLoadCSV())
	{
		RefreshReplaySessionDropdown();
		if (ReplaySessionSelector)
		{
			ReplaySessionSelector->SetSelectedOption(ReplayMgr->GetActiveSessionName());
		}
	}
}

void UMjSimulateWidget::HandleReplayBindingEnabledChanged(bool bIsChecked)
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
		UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (!ReplayMgr)
		return;

	// Find which checkbox triggered this by checking all stored checkboxes
	for (int32 i = 0; i < ReplayEnabledCheckBoxes.Num(); ++i)
	{
		if (ReplayEnabledCheckBoxes[i] && ReplayEnabledCheckBoxes[i]->IsChecked() != ReplayMgr->GetArticulationBindings()[i].bEnabled)
		{
			if (i < ReplayMgr->GetArticulationBindings().Num())
			{
				FReplayArticulationBinding& Binding = ReplayMgr->GetArticulationBindings()[i];
				Binding.bEnabled = ReplayEnabledCheckBoxes[i]->IsChecked();
				UE_LOG(LogURLab, Log, TEXT("Replay binding '%s' Enabled=%d"),
					Binding.Articulation.IsValid() ? *Binding.Articulation->GetName() : TEXT("<gone>"),
					Binding.bEnabled);
			}
		}
	}
}

void UMjSimulateWidget::HandleReplayBindingRelPosChanged(bool bIsChecked)
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
		UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (!ReplayMgr)
		return;

	for (int32 i = 0; i < ReplayRelPosCheckBoxes.Num(); ++i)
	{
		if (ReplayRelPosCheckBoxes[i] && ReplayRelPosCheckBoxes[i]->IsChecked() != ReplayMgr->GetArticulationBindings()[i].bRelativePosition)
		{
			if (i < ReplayMgr->GetArticulationBindings().Num())
			{
				FReplayArticulationBinding& Binding = ReplayMgr->GetArticulationBindings()[i];
				Binding.bRelativePosition = ReplayRelPosCheckBoxes[i]->IsChecked();
				UE_LOG(LogURLab, Log, TEXT("Replay binding '%s' RelPos=%d"),
					Binding.Articulation.IsValid() ? *Binding.Articulation->GetName() : TEXT("<gone>"),
					Binding.bRelativePosition);
			}
		}
	}
}

void UMjSimulateWidget::OnSaveRecordingClicked()
{
	UE_LOG(LogURLab, Log, TEXT("MjSimulateWidget: Save Recording clicked"));
	if (!ManagerRef)
		return;
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
		UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (!ReplayMgr)
	{
		UE_LOG(LogURLab, Warning, TEXT("MjSimulateWidget: No AMjReplayManager in scene!"));
		return;
	}
	ReplayMgr->BrowseAndSaveRecording();
}

void UMjSimulateWidget::RefreshReplaySessionDropdown()
{
	if (!ReplaySessionSelector)
		return;

	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
		UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (!ReplayMgr)
		return;

	FString CurrentSelection = ReplaySessionSelector->GetSelectedOption();
	ReplaySessionSelector->ClearOptions();

	TArray<FString> Names = ReplayMgr->GetSessionNames();
	for (const FString& Name : Names)
	{
		ReplaySessionSelector->AddOption(Name);
	}

	// Restore selection or default to active session
	if (Names.Contains(CurrentSelection))
	{
		ReplaySessionSelector->SetSelectedOption(CurrentSelection);
	}
	else
	{
		ReplaySessionSelector->SetSelectedOption(ReplayMgr->GetActiveSessionName());
	}

	CachedSessionCount = ReplayMgr->GetSessionCount();
}

void UMjSimulateWidget::RebuildReplayBindingUI(UVerticalBox* ReplayBox)
{
	if (!ReplayBox)
		return;

	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(
		UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (!ReplayMgr)
		return;

	TArray<FReplayArticulationBinding>& Bindings = ReplayMgr->GetArticulationBindings();
	if (Bindings.Num() == 0)
		return;

	// Header
	UTextBlock* HeaderLabel = NewObject<UTextBlock>(this);
	HeaderLabel->SetText(FText::FromString(TEXT("Articulations:")));
	FSlateFontInfo HeaderFont = HeaderLabel->GetFont();
	HeaderFont.Size = 9;
	HeaderLabel->SetFont(HeaderFont);
	HeaderLabel->SetColorAndOpacity(FSlateColor(FLinearColor(0.7f, 0.7f, 0.7f)));
	ReplayBox->AddChildToVerticalBox(HeaderLabel)->SetPadding(FMargin(0, 8, 0, 2));

	for (int32 Idx = 0; Idx < Bindings.Num(); ++Idx)
	{
		FReplayArticulationBinding& Binding = Bindings[Idx];
		if (!Binding.Articulation.IsValid())
			continue;

		UHorizontalBox* Row = NewObject<UHorizontalBox>(this);

		// Enabled checkbox
		UCheckBox* EnabledCB = NewObject<UCheckBox>(this);
		EnabledCB->SetIsChecked(Binding.bEnabled);
		EnabledCB->OnCheckStateChanged.AddDynamic(this, &UMjSimulateWidget::HandleReplayBindingEnabledChanged);
		// Tag the checkbox with the binding index via its tooltip (hacky but simple)
		Row->AddChildToHorizontalBox(EnabledCB)->SetPadding(FMargin(0, 0, 4, 0));

		UTextBlock* EnabledLabel = NewObject<UTextBlock>(this);
		EnabledLabel->SetText(FText::FromString(TEXT("On")));
		FSlateFontInfo SmallFont = EnabledLabel->GetFont();
		SmallFont.Size = 8;
		EnabledLabel->SetFont(SmallFont);
		EnabledLabel->SetColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f)));
		Row->AddChildToHorizontalBox(EnabledLabel)->SetPadding(FMargin(0, 0, 8, 0));

		// RelPos checkbox
		UCheckBox* RelPosCB = NewObject<UCheckBox>(this);
		RelPosCB->SetIsChecked(Binding.bRelativePosition);
		RelPosCB->OnCheckStateChanged.AddDynamic(this, &UMjSimulateWidget::HandleReplayBindingRelPosChanged);
		Row->AddChildToHorizontalBox(RelPosCB)->SetPadding(FMargin(0, 0, 4, 0));

		UTextBlock* RelPosLabel = NewObject<UTextBlock>(this);
		RelPosLabel->SetText(FText::FromString(TEXT("RelPos")));
		RelPosLabel->SetFont(SmallFont);
		RelPosLabel->SetColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f)));
		Row->AddChildToHorizontalBox(RelPosLabel)->SetPadding(FMargin(0, 0, 8, 0));

		// Articulation name
		FString DisplayName = MjUtils::PrettifyName(Binding.Articulation->GetName());
		UTextBlock* NameLabel = NewObject<UTextBlock>(this);
		NameLabel->SetText(FText::FromString(DisplayName));
		NameLabel->SetFont(SmallFont);
		NameLabel->SetColorAndOpacity(FSlateColor(FLinearColor::White));
		Row->AddChildToHorizontalBox(NameLabel);

		ReplayBox->AddChildToVerticalBox(Row)->SetPadding(FMargin(4, 2, 0, 2));

		// Store checkbox references for the handler to find the binding index
		// We use a simple approach: store them in parallel arrays
		ReplayEnabledCheckBoxes.Add(EnabledCB);
		ReplayRelPosCheckBoxes.Add(RelPosCB);
	}
}

void UMjSimulateWidget::OnIntegratorSelected(FString SelectedItem, ESelectInfo::Type SelectionType)
{
	if (!ManagerRef)
		return;

	if (!ManagerRef->PhysicsEngine || !ManagerRef->SceneOption)
		return;
	if (SelectedItem == TEXT("Euler"))
		ManagerRef->SceneOption->Integrator = EMjIntegrator::Euler;
	else if (SelectedItem == TEXT("RK4"))
		ManagerRef->SceneOption->Integrator = EMjIntegrator::RK4;
	else if (SelectedItem == TEXT("Implicit"))
		ManagerRef->SceneOption->Integrator = EMjIntegrator::implicit;
	else if (SelectedItem == TEXT("ImplicitFast"))
		ManagerRef->SceneOption->Integrator = EMjIntegrator::implicitfast;

	ManagerRef->PhysicsEngine->ApplyOptions();
}

void UMjSimulateWidget::OnArticulationSelected(FString SelectedItem, ESelectInfo::Type SelectionType)
{
	if (!ManagerRef)
		return;

	// Camera feed cleanup is now handled in RefreshArticulationControls.
	// Find the entity whose prettified public name matches the selection.
	SelectedEntityName = NAME_None;

	if (ManagerRef->PhysicsEngine)
	{
		for (const FMjEntity& E : ManagerRef->PhysicsEngine->GetEntityPartition())
		{
			if (MjUtils::PrettifyName(E.PublicName.ToString()) == SelectedItem)
			{
				SelectedEntityName = E.Name;
				break;
			}
		}
	}

	// Refresh manager settings to show the selected articulation's toggles
	PopulateManagerSettings();

	RefreshArticulationControls();
	RefreshKeyframeDropdown();
}

void UMjSimulateWidget::RefreshArticulationControls()
{
	if (!ArticulationControlList || !PropertyRowClass)
	{
		UE_LOG(LogURLab, Warning, TEXT("MjSimulateWidget: Refresh failed. Class=%s"),
			PropertyRowClass ? TEXT("Valid") : TEXT("NULL"));
		return;
	}

	// Teardown camera feeds before clearing panels
	for (UMjCameraFeedEntry* OldFeed : ActiveCameraFeeds)
	{
		if (OldFeed)
			OldFeed->UnbindCamera();
	}
	ActiveCameraFeeds.Empty();

	// Monitor rows are recreated below; drop the stale bindings.
	MonitorRows.Reset();
	MonitorIds.Reset();
	MonitorKinds.Reset();

	if (ManagerSettingsList)
	{
		// Safe-unlink our persistent buttons so they don't get destroyed by ClearChildren
		if (SnapshotButton)
			SnapshotButton->RemoveFromParent();
		if (RestoreButton)
			RestoreButton->RemoveFromParent();
		if (RecordButton)
			RecordButton->RemoveFromParent();
		if (ReplayButton)
			ReplayButton->RemoveFromParent();
		if (ReplaySessionSelector)
			ReplaySessionSelector->RemoveFromParent();
		if (LoadCSVButton)
			LoadCSVButton->RemoveFromParent();
		if (SaveRecordingButton)
			SaveRecordingButton->RemoveFromParent();
		if (KeyframeSelector)
			KeyframeSelector->RemoveFromParent();
		if (ResetToKeyframeButton)
			ResetToKeyframeButton->RemoveFromParent();
		if (WatchSelector)
			WatchSelector->RemoveFromParent();
		ReplayEnabledCheckBoxes.Empty();
		ReplayRelPosCheckBoxes.Empty();

		ManagerSettingsList->ClearChildren();
	}
	ArticulationControlList->ClearChildren();

	if (!SelectedEntityName.IsNone())
	{
		UE_LOG(LogURLab, Log, TEXT("MjSimulateWidget: Refreshing controls for %s"), *SelectedEntityName.ToString());
	}
	else
	{
		UE_LOG(LogURLab, Log, TEXT("MjSimulateWidget: Refreshing global controls (No entity selected)"));
	}

	auto CreateSection = [&](UVerticalBox* ParentList, const FString& Title, UVerticalBox*& OutContentBox) {
		UExpandableArea* ExpArea = NewObject<UExpandableArea>(this);
		UTextBlock* HeaderText = NewObject<UTextBlock>(this);
		HeaderText->SetText(FText::FromString(Title));

		FSlateFontInfo FontInfo = HeaderText->GetFont();
		FontInfo.Size = 11;
		FontInfo.TypefaceFontName = TEXT("Bold");
		HeaderText->SetFont(FontInfo);
		HeaderText->SetColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.9f, 1.0f, 1.0f)));

		FName HeaderName(TEXT("Header"));
		FName BodyName(TEXT("Body"));

		ExpArea->SetContentForSlot(HeaderName, HeaderText);

		OutContentBox = NewObject<UVerticalBox>(this);
		ExpArea->SetContentForSlot(BodyName, OutContentBox);

		ExpArea->SetIsExpanded(true);

		// Style the expandable area with a visible border
		FExpandableAreaStyle AreaStyle = ExpArea->GetStyle();
		AreaStyle.CollapsedImage.TintColor = FSlateColor(FLinearColor(0.3f, 0.35f, 0.4f, 1.0f));
		AreaStyle.ExpandedImage.TintColor = FSlateColor(FLinearColor(0.3f, 0.35f, 0.4f, 1.0f));
		ExpArea->SetStyle(AreaStyle);

		UVerticalBoxSlot* BoxSlot = ParentList->AddChildToVerticalBox(ExpArea);
		if (BoxSlot)
		{
			BoxSlot->SetPadding(FMargin(0, 3, 0, 5));
			BoxSlot->SetHorizontalAlignment(HAlign_Fill);
		}
	};

	auto AddRow = [&](UVerticalBox* List, const FString& Name, float Initial, EMjPropertyType Type, bool bIsActuator, FVector2D range = FVector2D(0.0f, 1.0f), bool bIsManagerOption = false, bool bEntityScoped = false) -> UMjPropertyRow* {
		UMjPropertyRow* Row = CreateWidget<UMjPropertyRow>(this, PropertyRowClass);
		if (Row)
		{
			FString DisplayName = Name;
			if (bEntityScoped && !bIsManagerOption)
			{
				const FString EntityName = SelectedEntityName.IsNone() ? TEXT("Global") : SelectedEntityName.ToString();
				DisplayName = MjUtils::PrettifyName(Name, EntityName);
			}

			Row->InitializeProperty(Name, Type, Initial, range, DisplayName);
			if (bIsManagerOption)
			{
				Row->OnValueChanged.AddDynamic(this, &UMjSimulateWidget::HandleManagerOptionChanged);
			}
			else if (bIsActuator)
			{
				Row->SetControllable(true);
				Row->OnValueChanged.AddDynamic(this, &UMjSimulateWidget::HandleActuatorChanged);
			}

			UVerticalBoxSlot* VerticalSlot = List->AddChildToVerticalBox(Row);
			if (VerticalSlot)
			{
				VerticalSlot->SetPadding(Type == EMjPropertyType::Header ? FMargin(0, 8, 0, 4) : FMargin(0, 2, 0, 2));
				VerticalSlot->SetHorizontalAlignment(HAlign_Fill);
			}
		}
		return Row;
	};

	// Manager / global panels, arranged to mirror MuJoCo simulate's left-hand
	// collapsible panels (Simulation, Watch, Physics, Rendering, Group enable),
	// followed by URLab-specific panels (Network, Snapshots, Replay).
	if (ManagerSettingsList)
	{
		UMjPhysicsEngine* PE = ManagerRef->PhysicsEngine;
		UMjDebugVisualizer* DV = ManagerRef->DebugVisualizer;
		UMjNetworkManager* NM = ManagerRef->NetworkManager;
		UMjOption* const SceneOption = ManagerRef->SceneOption;

		UVerticalBox* SimulationBox = nullptr;
		CreateSection(ManagerSettingsList, TEXT("SIMULATION"), SimulationBox);
		AddRow(SimulationBox, TEXT("Sim Speed %"), PE ? PE->SimSpeedPercent : 100.0f, EMjPropertyType::Slider, false, FVector2D(5.0f, 100.0f), true);

		if (!KeyframeSelector)
		{
			KeyframeSelector = NewObject<UComboBoxString>(this);
			KeyframeSelector->OnSelectionChanged.AddDynamic(this, &UMjSimulateWidget::OnKeyframeSelected);

			FTableRowStyle RowStyle = KeyframeSelector->GetItemStyle();
			FSlateColor RowBG(FLinearColor(0.15f, 0.15f, 0.18f, 1.0f));
			FSlateColor RowHover(FLinearColor(0.25f, 0.30f, 0.35f, 1.0f));
			RowStyle.SetEvenRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
			RowStyle.SetOddRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
			RowStyle.SetEvenRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
			RowStyle.SetOddRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
			KeyframeSelector->SetItemStyle(RowStyle);
		}
		RefreshKeyframeDropdown();
		if (UVerticalBoxSlot* BoxSlot = SimulationBox->AddChildToVerticalBox(KeyframeSelector))
		{
			BoxSlot->SetPadding(FMargin(0, 5, 0, 5));
		}

		if (!ResetToKeyframeButton)
		{
			ResetToKeyframeButton = NewObject<UButton>(this);
			UTextBlock* BtnText = NewObject<UTextBlock>(ResetToKeyframeButton);
			BtnText->SetText(FText::FromString(TEXT("Load Keyframe")));
			BtnText->SetColorAndOpacity(FSlateColor(FLinearColor::White));
			BtnText->SetJustification(ETextJustify::Center);
			FSlateFontInfo FontInfo = BtnText->GetFont();
			FontInfo.Size = 14;
			BtnText->SetFont(FontInfo);
			ResetToKeyframeButton->AddChild(BtnText);
			ResetToKeyframeButton->SetBackgroundColor(FLinearColor(0.1f, 0.4f, 0.8f, 1.0f));
			ResetToKeyframeButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::HandleResetToKeyframe);
		}
		if (UVerticalBoxSlot* BoxSlot = SimulationBox->AddChildToVerticalBox(ResetToKeyframeButton))
		{
			BoxSlot->SetPadding(FMargin(0, 5, 0, 5));
		}

		UVerticalBox* WatchBox = nullptr;
		CreateSection(ManagerSettingsList, TEXT("WATCH"), WatchBox);
		if (!WatchSelector)
		{
			WatchSelector = NewObject<UComboBoxString>(this);
			WatchSelector->OnSelectionChanged.AddDynamic(this, &UMjSimulateWidget::OnWatchSelected);

			FTableRowStyle RowStyle = WatchSelector->GetItemStyle();
			FSlateColor RowBG(FLinearColor(0.15f, 0.15f, 0.18f, 1.0f));
			FSlateColor RowHover(FLinearColor(0.25f, 0.30f, 0.35f, 1.0f));
			RowStyle.SetEvenRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
			RowStyle.SetOddRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
			RowStyle.SetEvenRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
			RowStyle.SetOddRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
			WatchSelector->SetItemStyle(RowStyle);
		}
		RefreshWatchDropdown();
		if (UVerticalBoxSlot* BoxSlot = WatchBox->AddChildToVerticalBox(WatchSelector))
		{
			BoxSlot->SetPadding(FMargin(0, 5, 0, 5));
		}
		WatchValueRow = AddRow(WatchBox, TEXT("Value"), 0.0f, EMjPropertyType::LabelOnly, false, FVector2D(0, 0), false, false);

		UVerticalBox* PhysicsBox = nullptr;
		CreateSection(ManagerSettingsList, TEXT("PHYSICS"), PhysicsBox);
		{
			UComboBoxString* IntegratorCombo = NewObject<UComboBoxString>(this);
			IntegratorCombo->AddOption(TEXT("Euler"));
			IntegratorCombo->AddOption(TEXT("RK4"));
			IntegratorCombo->AddOption(TEXT("Implicit"));
			IntegratorCombo->AddOption(TEXT("ImplicitFast"));
			if (ManagerRef->SceneOption)
				IntegratorCombo->SetSelectedIndex(
					(int)ManagerRef->SceneOption->Integrator.Get(EMjIntegrator::Euler));
			IntegratorCombo->OnSelectionChanged.AddDynamic(this, &UMjSimulateWidget::OnIntegratorSelected);
			{
				FTableRowStyle RowStyle = IntegratorCombo->GetItemStyle();
				FSlateColor RowBG(FLinearColor(0.15f, 0.15f, 0.18f, 1.0f));
				FSlateColor RowHover(FLinearColor(0.25f, 0.30f, 0.35f, 1.0f));
				RowStyle.SetEvenRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
				RowStyle.SetOddRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
				RowStyle.SetEvenRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
				RowStyle.SetOddRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
				IntegratorCombo->SetItemStyle(RowStyle);
			}
			PhysicsBox->AddChildToVerticalBox(IntegratorCombo);
		}
		AddRow(PhysicsBox, TEXT("Timestep"), SceneOption ? (float)SceneOption->Timestep.Get(0.002) : 0.002f, EMjPropertyType::Slider, false, FVector2D(0.0001f, 0.05f), true);
		AddRow(PhysicsBox, TEXT("Iterations"), SceneOption ? (float)SceneOption->Iterations.Get(50) : 50.0f, EMjPropertyType::Slider, false, FVector2D(5.0f, 200.0f), true);

		UVerticalBox* RenderingBox = nullptr;
		CreateSection(ManagerSettingsList, TEXT("RENDERING"), RenderingBox);
		AddRow(RenderingBox, TEXT("Contact Point"), (DV && DV->bGlobalDrawDebugContactPoints) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		AddRow(RenderingBox, TEXT("Contact Force"), (DV && DV->bGlobalDrawDebugContactForces) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		AddRow(RenderingBox, TEXT("Center of Mass"), (DV && DV->bGlobalDrawDebugCom) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		AddRow(RenderingBox, TEXT("Inertia"), (DV && DV->bGlobalDrawDebugInertia) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		AddRow(RenderingBox, TEXT("Perturbation"), (DV && DV->bGlobalDrawDebugPerturb) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		AddRow(RenderingBox, TEXT("Collision"), (DV && DV->bGlobalDrawDebugCollision) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		AddRow(RenderingBox, TEXT("Joint"), (DV && DV->bGlobalDrawDebugJoints) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		AddRow(RenderingBox, TEXT("Tendon"), (DV && DV->bGlobalDrawTendons) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		AddRow(RenderingBox, TEXT("Contact Visualization"), (DV && DV->bShowDebug) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		AddRow(RenderingBox, TEXT("Quick Collision"), (DV && DV->bGlobalQuickConvertCollision) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);

		if (!SelectedEntityName.IsNone())
		{
			const FMjEntity* Ent = FindEntity(PE, SelectedEntityName);
			const FMjEntityDrawFlags Overlay = Ent ? Ent->Overlay : FMjEntityDrawFlags();
			AddRow(RenderingBox, TEXT("Selected Collision"), Overlay.bDrawDebugCollision ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
			AddRow(RenderingBox, TEXT("Selected Joint"), Overlay.bDrawDebugJoints ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
			AddRow(RenderingBox, TEXT("Selected Site"), Overlay.bDrawDebugSites ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		}

		UVerticalBox* GroupBox = nullptr;
		CreateSection(ManagerSettingsList, TEXT("GROUP ENABLE"), GroupBox);
		AddRow(GroupBox, TEXT("Geom Group 3 (Global)"), (DV && DV->bGlobalShowGroup3) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		if (!SelectedEntityName.IsNone())
		{
			AddRow(GroupBox, TEXT("Selected Geom Group 3"), 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);
		}

		UVerticalBox* NetworkBox = nullptr;
		CreateSection(ManagerSettingsList, TEXT("NETWORK"), NetworkBox);
		AddRow(NetworkBox, TEXT("Enable All Cameras"), (NM && NM->bEnableAllCameras) ? 1.0f : 0.0f, EMjPropertyType::Toggle, false, FVector2D(0, 1), true);

		UVerticalBox* SnapshotBox = nullptr;
		CreateSection(ManagerSettingsList, TEXT("SNAPSHOTS"), SnapshotBox);
		if (SnapshotButton)
		{
			if (UVerticalBoxSlot* BoxSlot = SnapshotBox->AddChildToVerticalBox(SnapshotButton))
			{
				BoxSlot->SetPadding(FMargin(0, 5, 0, 5));
			}
		}
		if (RestoreButton)
		{
			if (UVerticalBoxSlot* BoxSlot = SnapshotBox->AddChildToVerticalBox(RestoreButton))
			{
				BoxSlot->SetPadding(FMargin(0, 5, 0, 5));
			}
		}

		UVerticalBox* ReplayBox = nullptr;
		CreateSection(ManagerSettingsList, TEXT("REPLAY"), ReplayBox);

		if (!ReplaySessionSelector)
		{
			ReplaySessionSelector = NewObject<UComboBoxString>(this);
			ReplaySessionSelector->OnSelectionChanged.AddDynamic(this, &UMjSimulateWidget::OnReplaySessionSelected);

			FTableRowStyle RowStyle = ReplaySessionSelector->GetItemStyle();
			FSlateColor RowBG(FLinearColor(0.15f, 0.15f, 0.18f, 1.0f));
			FSlateColor RowHover(FLinearColor(0.25f, 0.30f, 0.35f, 1.0f));
			RowStyle.SetEvenRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
			RowStyle.SetOddRowBackgroundBrush(FSlateRoundedBoxBrush(RowBG, 0.0f));
			RowStyle.SetEvenRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
			RowStyle.SetOddRowBackgroundHoveredBrush(FSlateRoundedBoxBrush(RowHover, 0.0f));
			RowStyle.SetTextColor(FSlateColor(FLinearColor::White));
			RowStyle.SetSelectedTextColor(FSlateColor(FLinearColor::White));
			ReplaySessionSelector->SetItemStyle(RowStyle);
		}
		ReplayBox->AddChildToVerticalBox(ReplaySessionSelector)->SetPadding(FMargin(0, 5, 0, 5));
		RefreshReplaySessionDropdown();

		RebuildReplayBindingUI(ReplayBox);

		if (RecordButton)
		{
			if (UVerticalBoxSlot* BoxSlot = ReplayBox->AddChildToVerticalBox(RecordButton))
			{
				BoxSlot->SetPadding(FMargin(0, 5, 0, 5));
			}
		}
		if (ReplayButton)
		{
			if (UVerticalBoxSlot* BoxSlot = ReplayBox->AddChildToVerticalBox(ReplayButton))
			{
				BoxSlot->SetPadding(FMargin(0, 5, 0, 5));
			}
		}

		if (!LoadCSVButton)
		{
			LoadCSVButton = NewObject<UButton>(this);
			UTextBlock* BtnLabel = NewObject<UTextBlock>(this);
			BtnLabel->SetText(FText::FromString(TEXT("Load Replay")));
			FSlateFontInfo Font = BtnLabel->GetFont();
			Font.Size = 10;
			BtnLabel->SetFont(Font);
			BtnLabel->SetColorAndOpacity(FSlateColor(FLinearColor::White));
			LoadCSVButton->AddChild(BtnLabel);

			FButtonStyle BtnStyle = LoadCSVButton->GetStyle();
			FLinearColor BtnColor(0.4f, 0.3f, 0.7f, 0.9f);
			BtnStyle.Normal.TintColor = FSlateColor(BtnColor);
			BtnStyle.Hovered.TintColor = FSlateColor(BtnColor * 1.2f);
			BtnStyle.Pressed.TintColor = FSlateColor(BtnColor * 0.8f);
			LoadCSVButton->SetStyle(BtnStyle);

			LoadCSVButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::OnLoadCSVClicked);
		}
		ReplayBox->AddChildToVerticalBox(LoadCSVButton)->SetPadding(FMargin(0, 5, 0, 5));

		if (!SaveRecordingButton)
		{
			SaveRecordingButton = NewObject<UButton>(this);
			UTextBlock* SaveLabel = NewObject<UTextBlock>(this);
			SaveLabel->SetText(FText::FromString(TEXT("Save Recording")));
			FSlateFontInfo SaveFont = SaveLabel->GetFont();
			SaveFont.Size = 10;
			SaveLabel->SetFont(SaveFont);
			SaveLabel->SetColorAndOpacity(FSlateColor(FLinearColor::White));
			SaveRecordingButton->AddChild(SaveLabel);

			FButtonStyle SaveStyle = SaveRecordingButton->GetStyle();
			FLinearColor SaveColor(0.2f, 0.5f, 0.3f, 0.9f);
			SaveStyle.Normal.TintColor = FSlateColor(SaveColor);
			SaveStyle.Hovered.TintColor = FSlateColor(SaveColor * 1.2f);
			SaveStyle.Pressed.TintColor = FSlateColor(SaveColor * 0.8f);
			SaveRecordingButton->SetStyle(SaveStyle);

			SaveRecordingButton->OnClicked.AddDynamic(this, &UMjSimulateWidget::OnSaveRecordingClicked);
		}
		ReplayBox->AddChildToVerticalBox(SaveRecordingButton)->SetPadding(FMargin(0, 5, 0, 5));
	}

	// --- Entity-Specific Sections ---
	if (SelectedEntityName.IsNone())
		return;

	UMjPhysicsEngine* PE = ManagerRef->PhysicsEngine;
	const mjModel* M = PE ? PE->GetModel() : nullptr;
	const FMjEntity* Ent = FindEntity(PE, SelectedEntityName);
	if (!PE || !M || !Ent)
	{
		InvalidateLayoutAndVolatility();
		return;
	}

	// Monitors: Joints — position read from the engine's render snapshot.
	const TArray<FName> JointNames = MjEntityMembers::Names(PE, SelectedEntityName, EMjEntityMember::Joint);
	if (JointNames.Num() > 0)
	{
		UVerticalBox* SecBox = nullptr;
		CreateSection(ArticulationControlList, TEXT("JOINT"), SecBox);
		for (const FName& JointName : JointNames)
		{
			const int32 Id = MjEntityMembers::ResolveId(PE, SelectedEntityName, EMjEntityMember::Joint, JointName);
			if (Id < 0 || Id >= M->njnt)
				continue;
			const int32 Adr = M->jnt_qposadr[Id];
			const float Pos = (Adr >= 0 && Adr < M->nq)
				? static_cast<float>(MjSnapshotValue(*PE, Adr,
					[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.QPos; }))
				: 0.0f;
			UMjPropertyRow* Row = AddRow(SecBox, JointName.ToString(), Pos, EMjPropertyType::LabelOnly, false, FVector2D(0, 0), false, true);
			if (Row)
			{
				MonitorRows.Add(Row);
				MonitorIds.Add(Id);
				MonitorKinds.Add(1);
			}
		}
	}

	// Actuators: interactive setpoint sliders, addressed by entity name + actuator id.
	const TArray<FName> ActNames = MjEntityMembers::Names(PE, SelectedEntityName, EMjEntityMember::Actuator);
	if (ActNames.Num() > 0)
	{
		UVerticalBox* SecBox = nullptr;
		CreateSection(ArticulationControlList, TEXT("CONTROL"), SecBox);
		for (const FName& ActName : ActNames)
		{
			const int32 Id = MjEntityMembers::ResolveId(PE, SelectedEntityName, EMjEntityMember::Actuator, ActName);
			if (Id < 0 || Id >= M->nu)
				continue;
			const FVector2D Range(static_cast<float>(M->actuator_ctrlrange[2 * Id + 0]),
				static_cast<float>(M->actuator_ctrlrange[2 * Id + 1]));
			UMjPropertyRow* Row = AddRow(SecBox, ActName.ToString(), static_cast<float>(PE->GetSetpoint(Id)),
				EMjPropertyType::Slider, true, Range, false, true);
			if (Row)
			{
				MonitorRows.Add(Row);
				MonitorIds.Add(Id);
				MonitorKinds.Add(0);
			}
		}
	}

	// Monitors: Sensors — scalar reading read from the render snapshot's sensordata.
	if (Ent->SensorIds.Num() > 0)
	{
		UVerticalBox* SecBox = nullptr;
		CreateSection(ArticulationControlList, TEXT("SENSORS"), SecBox);
		for (int32 Id : Ent->SensorIds)
		{
			if (Id < 0 || Id >= M->nsensor)
				continue;
			const FString SensorName = ShortMemberName(SelectedEntityName, CompiledNameOf(M, mjOBJ_SENSOR, Id));
			const int32 Adr = M->sensor_adr[Id];
			const float Val = (Adr >= 0 && Adr < M->nsensordata)
				? static_cast<float>(MjSnapshotValue(*PE, Adr,
					[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.SensorData; }))
				: 0.0f;
			UMjPropertyRow* Row = AddRow(SecBox, SensorName, Val, EMjPropertyType::LabelOnly, false, FVector2D(0, 0), false, true);
			if (Row)
			{
				MonitorRows.Add(Row);
				MonitorIds.Add(Id);
				MonitorKinds.Add(2);
			}
		}
	}

	UE_LOG(LogURLab, Log, TEXT("MjSimulateWidget: Added %d actuators, %d joints, %d sensors for %s"),
		ActNames.Num(), JointNames.Num(), Ent->SensorIds.Num(), *SelectedEntityName.ToString());

	// Camera Feeds (Left Panel): the renderer-agnostic cameras whose canonical art
	// segment matches this entity (not the retired articulation's UMjCamera components).
	if (CameraFeedEntryClass && ManagerSettingsList)
	{
		TArray<UMjCamera*> AllCameras;
		ManagerRef->CollectCameras(AllCameras);

		const FString ArtSegment = FMjCanonicalName::Sanitize(Ent->PublicName.ToString());
		TArray<UMjCamera*> Cameras;
		for (UMjCamera* Cam : AllCameras)
		{
			if (!Cam)
				continue;
			FString CamArt, CamPart;
			if (Cam->GetCanonicalName().Split(TEXT("/"), &CamArt, &CamPart) && CamArt == ArtSegment)
			{
				Cameras.Add(Cam);
			}
		}

		if (Cameras.Num() > 0)
		{
			UVerticalBox* SecBox = nullptr;
			CreateSection(ManagerSettingsList, TEXT("CAMERAS"), SecBox);

			for (UMjCamera* Cam : Cameras)
			{
				UMjCameraFeedEntry* Entry = CreateWidget<UMjCameraFeedEntry>(this, CameraFeedEntryClass);
				if (Entry)
				{
					Entry->BindToCamera(Cam);
					UVerticalBoxSlot* VerticalSlot = SecBox->AddChildToVerticalBox(Entry);
					if (VerticalSlot)
					{
						VerticalSlot->SetPadding(FMargin(0, 5, 0, 5));
					}
					ActiveCameraFeeds.Add(Entry);
				}
			}
		}

		UE_LOG(LogURLab, Log, TEXT("MjSimulateWidget: Added %d camera feeds for %s"),
			Cameras.Num(), *SelectedEntityName.ToString());
	}

	// Locomotion sliders: the twist controller now lives on the entity's possess pawn
	// (re-homed off the articulation at handoff), reachable while a pawn exists for it.
	if (ArticulationControlList && PropertyRowClass)
	{
		UMjTwistController* TC = FindEntityTwist(GetWorld(), SelectedEntityName);
		if (TC)
		{
			auto AddTwistRow = [&](UVerticalBox* List, const FString& Name, float Initial, FVector2D range) {
				UMjPropertyRow* Row = CreateWidget<UMjPropertyRow>(this, PropertyRowClass);
				if (Row)
				{
					Row->InitializeProperty(Name, EMjPropertyType::Slider, Initial, range, Name);
					Row->OnValueChanged.AddDynamic(this, &UMjSimulateWidget::HandleTwistOptionChanged);
					if (UVerticalBoxSlot* Slot = List->AddChildToVerticalBox(Row))
					{
						Slot->SetPadding(FMargin(0, 2, 0, 2));
						Slot->SetHorizontalAlignment(HAlign_Fill);
					}
				}
			};

			UVerticalBox* LocoBox = nullptr;
			CreateSection(ArticulationControlList, TEXT("POSSESSION"), LocoBox);
			AddTwistRow(LocoBox, TEXT("Max Forward Speed"), TC->MaxVx, FVector2D(0.0f, 2.0f));
			AddTwistRow(LocoBox, TEXT("Max Strafe Speed"), TC->MaxVy, FVector2D(0.0f, 1.0f));
			AddTwistRow(LocoBox, TEXT("Max Turn Rate"), TC->MaxYawRate, FVector2D(0.0f, 3.14f));
		}
	}

	// force layout recalculation after content changes (camera feeds change panel widths)
	InvalidateLayoutAndVolatility();
}

void UMjSimulateWidget::UpdateMonitorValues()
{
	if (SelectedEntityName.IsNone())
		return;

	UMjPhysicsEngine* PE = ManagerRef ? ManagerRef->PhysicsEngine : nullptr;
	const mjModel* M = PE ? PE->GetModel() : nullptr;
	if (!PE || !M)
		return;

	if (WatchValueRow && WatchId >= 0)
	{
		float WVal = 0.0f;
		if (WatchKind == 1 && WatchId < M->njnt)
		{
			const int32 Adr = M->jnt_qposadr[WatchId];
			if (Adr >= 0 && Adr < M->nq)
				WVal = static_cast<float>(MjSnapshotValue(*PE, Adr,
					[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.QPos; }));
		}
		else if (WatchKind == 2 && WatchId < M->nsensor)
		{
			const int32 Adr = M->sensor_adr[WatchId];
			if (Adr >= 0 && Adr < M->nsensordata)
				WVal = static_cast<float>(MjSnapshotValue(*PE, Adr,
					[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.SensorData; }));
		}
		WatchValueRow->SetValue(WVal);
	}

	for (int32 i = 0; i < MonitorRows.Num(); ++i)
	{
		UMjPropertyRow* Row = MonitorRows[i];
		if (!Row || Row->IsBeingDragged())
			continue;

		const int32 Id = MonitorIds[i];
		float Val = 0.0f;
		switch (MonitorKinds[i])
		{
			case 0: // actuator: the staged setpoint the UI last wrote
				Val = static_cast<float>(PE->GetSetpoint(Id));
				break;
			case 1: // joint: position from the render snapshot's qpos
				if (Id >= 0 && Id < M->njnt)
				{
					const int32 Adr = M->jnt_qposadr[Id];
					if (Adr >= 0 && Adr < M->nq)
						Val = static_cast<float>(MjSnapshotValue(*PE, Adr,
							[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.QPos; }));
				}
				break;
			case 2: // sensor: scalar reading from the render snapshot's sensordata
				if (Id >= 0 && Id < M->nsensor)
				{
					const int32 Adr = M->sensor_adr[Id];
					if (Adr >= 0 && Adr < M->nsensordata)
						Val = static_cast<float>(MjSnapshotValue(*PE, Adr,
							[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.SensorData; }));
				}
				break;
			default:
				break;
		}

		Row->SetValue(Val);
	}
}

void UMjSimulateWidget::HandleManagerOptionChanged(float NewValue, const FString& OptionName)
{
	if (!ManagerRef)
		return;

	UMjPhysicsEngine* PE = ManagerRef->PhysicsEngine;
	UMjDebugVisualizer* DV = ManagerRef->DebugVisualizer;
	UMjNetworkManager* NM = ManagerRef->NetworkManager;

	if (OptionName == TEXT("Timestep"))
	{
		if (ManagerRef->SceneOption)
		{
			ManagerRef->SceneOption->Timestep = NewValue;
		}
	}
	else if (OptionName == TEXT("Iterations"))
	{
		if (ManagerRef->SceneOption)
		{
			ManagerRef->SceneOption->Iterations = (int32)NewValue;
		}
	}
	else if (OptionName == TEXT("Sim Speed %"))
	{
		if (PE)
			PE->SimSpeedPercent = NewValue;
	}
	else if (OptionName == TEXT("Contact Visualization"))
	{
		if (DV)
			DV->bShowDebug = (NewValue > 0.5f);
	}
	else if (OptionName == TEXT("Contact Point"))
	{
		if (DV)
			DV->bGlobalDrawDebugContactPoints = (NewValue > 0.5f);
	}
	else if (OptionName == TEXT("Contact Force"))
	{
		if (DV)
			DV->bGlobalDrawDebugContactForces = (NewValue > 0.5f);
	}
	else if (OptionName == TEXT("Center of Mass"))
	{
		if (DV)
			DV->bGlobalDrawDebugCom = (NewValue > 0.5f);
	}
	else if (OptionName == TEXT("Inertia"))
	{
		if (DV)
			DV->bGlobalDrawDebugInertia = (NewValue > 0.5f);
	}
	else if (OptionName == TEXT("Perturbation"))
	{
		if (DV)
			DV->bGlobalDrawDebugPerturb = (NewValue > 0.5f);
	}
	else if (OptionName == TEXT("Tendon"))
	{
		if (DV)
			DV->bGlobalDrawTendons = (NewValue > 0.5f);
	}
	else if (OptionName == TEXT("Collision"))
	{
		if (DV)
		{
			DV->bGlobalDrawDebugCollision = (NewValue > 0.5f);
			DV->UpdateAllGlobalVisibility();
		}
	}
	else if (OptionName == TEXT("Joint"))
	{
		if (DV)
		{
			DV->bGlobalDrawDebugJoints = (NewValue > 0.5f);
			DV->UpdateAllGlobalVisibility();
		}
	}
	else if (OptionName == TEXT("Geom Group 3 (Global)"))
	{
		if (DV)
		{
			DV->bGlobalShowGroup3 = (NewValue > 0.5f);
			DV->UpdateAllGlobalVisibility();
		}
	}
	else if (OptionName == TEXT("Quick Collision"))
	{
		if (DV)
		{
			DV->bGlobalQuickConvertCollision = (NewValue > 0.5f);
			DV->UpdateAllGlobalVisibility();
		}
	}
	else if (OptionName == TEXT("Enable All Cameras"))
	{
		if (NM)
		{
			NM->bEnableAllCameras = (NewValue > 0.5f);
			NM->UpdateCameraStreamingState();
		}
	}
	else if ((OptionName == TEXT("Selected Collision") || OptionName == TEXT("Selected Joint")
				 || OptionName == TEXT("Selected Site"))
			 && !SelectedEntityName.IsNone() && PE)
	{
		const FMjEntity* Ent = FindEntity(PE, SelectedEntityName);
		FMjEntityDrawFlags Flags = Ent ? Ent->Overlay : FMjEntityDrawFlags();
		const bool bOn = (NewValue > 0.5f);
		if (OptionName == TEXT("Selected Collision"))
			Flags.bDrawDebugCollision = bOn;
		else if (OptionName == TEXT("Selected Joint"))
			Flags.bDrawDebugJoints = bOn;
		else
			Flags.bDrawDebugSites = bOn;
		PE->SetEntityOverlayFlags(SelectedEntityName, Flags);
	}
	else if (OptionName == TEXT("Selected Geom Group 3") && !SelectedEntityName.IsNone())
	{
		if (AMjEntity* Entity = ManagerRef->GetEntity(SelectedEntityName))
		{
			Entity->SetGeomGroupVisible(3, (NewValue > 0.5f));
		}
	}

	// Locomotion twist settings
	if (OptionName == TEXT("Max Forward Speed") || OptionName == TEXT("Max Strafe Speed") || OptionName == TEXT("Max Turn Rate"))
	{
		HandleTwistOptionChanged(NewValue, OptionName);
	}

	// Only apply physics options when physics-related settings change
	if ((OptionName == TEXT("Timestep") || OptionName == TEXT("Iterations")) && PE)
	{
		PE->ApplyOptions();
	}
}

void UMjSimulateWidget::HandleActuatorChanged(float NewValue, const FString& OptionName)
{
	if (SelectedEntityName.IsNone() || !ManagerRef || !ManagerRef->PhysicsEngine)
		return;

	UMjPhysicsEngine* PE = ManagerRef->PhysicsEngine;
	const int32 Id = MjEntityMembers::ResolveId(PE, SelectedEntityName, EMjEntityMember::Actuator, FName(*OptionName));
	if (Id < 0)
		return;

	if (IMjControlIngress* Ingress = PE->GetControlIngress())
	{
		Ingress->WriteCtrl(SelectedEntityName, Id, NewValue, MjControlWho::UI());
	}
}

void UMjSimulateWidget::OnPossessClicked()
{
	if (SelectedEntityName.IsNone() || !ManagerRef)
		return;

	if (!bIsPossessing)
	{
		if (!ManagerRef->PossessEntity(SelectedEntityName))
		{
			UE_LOG(LogURLab, Warning, TEXT("Possess: entity '%s' has no possess pawn"), *SelectedEntityName.ToString());
			return;
		}
		bIsPossessing = true;

		if (PossessButton)
		{
			if (UTextBlock* BtnText = Cast<UTextBlock>(PossessButton->GetChildAt(0)))
			{
				BtnText->SetText(FText::FromString(TEXT("Release")));
			}
		}

		UE_LOG(LogURLab, Log, TEXT("Possessed entity: %s"), *SelectedEntityName.ToString());
	}
	else
	{
		ManagerRef->UnpossessEntity();
		bIsPossessing = false;

		if (PossessButton)
		{
			if (UTextBlock* BtnText = Cast<UTextBlock>(PossessButton->GetChildAt(0)))
			{
				BtnText->SetText(FText::FromString(TEXT("Possess")));
			}
		}

		UE_LOG(LogURLab, Log, TEXT("Released entity"));
	}
}

void UMjSimulateWidget::HandleTwistOptionChanged(float NewValue, const FString& OptionName)
{
	if (SelectedEntityName.IsNone())
	{
		return;
	}

	UMjTwistController* TwistCtrl = FindEntityTwist(GetWorld(), SelectedEntityName);
	if (!TwistCtrl)
	{
		UE_LOG(LogURLab, Warning, TEXT("HandleTwistOptionChanged: no twist controller for entity '%s'"), *SelectedEntityName.ToString());
		return;
	}

	if (OptionName == TEXT("Max Forward Speed"))
	{
		TwistCtrl->MaxVx = NewValue;
	}
	else if (OptionName == TEXT("Max Strafe Speed"))
	{
		TwistCtrl->MaxVy = NewValue;
	}
	else if (OptionName == TEXT("Max Turn Rate"))
	{
		TwistCtrl->MaxYawRate = NewValue;
	}

	UE_LOG(LogURLab, Log, TEXT("Twist option '%s' = %.3f"), *OptionName, NewValue);
}

void UMjSimulateWidget::OnKeyframeSelected(FString SelectedItem, ESelectInfo::Type SelectionType)
{
	// Just stores the selection — HandleResetToKeyframe uses it
	UE_LOG(LogURLab, Verbose, TEXT("Keyframe selected: %s"), *SelectedItem);
}

void UMjSimulateWidget::RefreshKeyframeDropdown()
{
	if (!KeyframeSelector)
		return;

	FString CurrentSelection = KeyframeSelector->GetSelectedOption();
	KeyframeSelector->ClearOptions();

	const mjModel* M = (ManagerRef && ManagerRef->PhysicsEngine) ? ManagerRef->PhysicsEngine->GetModel() : nullptr;
	if (SelectedEntityName.IsNone() || M == nullptr)
		return;

	TArray<FString> Names;
	for (const TPair<FString, int32>& Key : EntityKeyframes(M, SelectedEntityName))
	{
		Names.Add(Key.Key);
		KeyframeSelector->AddOption(Key.Key);
	}
	if (Names.Num() > 0)
	{
		KeyframeSelector->SetSelectedOption(Names.Contains(CurrentSelection) ? CurrentSelection : Names[0]);
	}
}

void UMjSimulateWidget::RefreshWatchDropdown()
{
	if (!WatchSelector)
		return;

	const FString CurrentSelection = WatchSelector->GetSelectedOption();
	WatchSelector->ClearOptions();
	WatchId = -1;
	WatchKind = 255;

	UMjPhysicsEngine* PE = ManagerRef ? ManagerRef->PhysicsEngine : nullptr;
	const mjModel* M = PE ? PE->GetModel() : nullptr;
	if (SelectedEntityName.IsNone() || M == nullptr)
		return;

	TArray<FString> Options;
	for (const FName& JointName : MjEntityMembers::Names(PE, SelectedEntityName, EMjEntityMember::Joint))
	{
		Options.Add(TEXT("joint: ") + JointName.ToString());
	}
	if (const FMjEntity* Ent = FindEntity(PE, SelectedEntityName))
	{
		for (int32 Id : Ent->SensorIds)
		{
			if (Id >= 0 && Id < M->nsensor)
				Options.Add(TEXT("sensor: ") + ShortMemberName(SelectedEntityName, CompiledNameOf(M, mjOBJ_SENSOR, Id)));
		}
	}

	for (const FString& Opt : Options)
	{
		WatchSelector->AddOption(Opt);
	}
	if (Options.Num() > 0)
	{
		const FString Pick = Options.Contains(CurrentSelection) ? CurrentSelection : Options[0];
		WatchSelector->SetSelectedOption(Pick);
		OnWatchSelected(Pick, ESelectInfo::Direct);
	}
}

void UMjSimulateWidget::OnWatchSelected(FString SelectedItem, ESelectInfo::Type SelectionType)
{
	WatchId = -1;
	WatchKind = 255;
	if (SelectedItem.IsEmpty() || SelectedEntityName.IsNone() || !ManagerRef || !ManagerRef->PhysicsEngine)
		return;

	UMjPhysicsEngine* PE = ManagerRef->PhysicsEngine;
	const mjModel* M = PE->GetModel();
	if (M == nullptr)
		return;

	FString Kind, Member;
	if (!SelectedItem.Split(TEXT(": "), &Kind, &Member))
		return;

	if (Kind == TEXT("joint"))
	{
		const int32 Id = MjEntityMembers::ResolveId(PE, SelectedEntityName, EMjEntityMember::Joint, FName(*Member));
		if (Id >= 0 && Id < M->njnt)
		{
			WatchId = Id;
			WatchKind = 1;
		}
	}
	else if (Kind == TEXT("sensor"))
	{
		if (const FMjEntity* Ent = FindEntity(PE, SelectedEntityName))
		{
			for (int32 Id : Ent->SensorIds)
			{
				if (Id >= 0 && Id < M->nsensor
					&& ShortMemberName(SelectedEntityName, CompiledNameOf(M, mjOBJ_SENSOR, Id)) == Member)
				{
					WatchId = Id;
					WatchKind = 2;
					break;
				}
			}
		}
	}
}

void UMjSimulateWidget::HandleResetToKeyframe()
{
	if (SelectedEntityName.IsNone() || !KeyframeSelector || !ManagerRef || !ManagerRef->PhysicsEngine)
		return;

	UMjPhysicsEngine* PE = ManagerRef->PhysicsEngine;
	const mjModel* M = PE->GetModel();
	if (M == nullptr)
		return;

	const FString KeyframeName = KeyframeSelector->GetSelectedOption();
	for (const TPair<FString, int32>& Key : EntityKeyframes(M, SelectedEntityName))
	{
		if (Key.Key == KeyframeName)
		{
			ApplyKeyframeReset(PE, Key.Value);
			break;
		}
	}
}
