// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeSubsystem.h"
#include "Bridge/RenderServiceBinding.h"
#include "Bridge/RpcDispatcher.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Editor.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/App.h"
#include "Misc/ByteSwap.h"
#include "Containers/Ticker.h"
#include "Render/RenderCameraSubsystem.h"
#include "Render/RenderCameraComponent.h"
#include "Transport/ZmqRpcTransport.h"
#include "UObject/UObjectIterator.h"
#include "Utils/AssetCache.h"
#include "Wire/RequestEnvelope.h"
#include "Workflows/RenderServerWorldSubsystem.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabTypedRenderCaptureTest
{
using V = FURLabWireValue;

V Map(std::initializer_list<TPair<FString, V>> Fields)
{
	TArray<FString> Keys;
	TArray<V> Values;
	for (const auto& Field : Fields) { Keys.Add(Field.Key); Values.Add(Field.Value); }
	return V::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

TArray<uint8> TextBytes(const char* Text)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), FCStringAnsi::Strlen(Text));
	return Out;
}

TArray<uint8> Doubles(const TArray<double>& Values)
{
	TArray<uint8> Out;
	Out.SetNumUninitialized(Values.Num() * static_cast<int32>(sizeof(double)));
	for (int32 Index = 0; Index < Values.Num(); ++Index)
	{
		uint64 Bits = 0;
		FMemory::Memcpy(&Bits, &Values[Index], sizeof(Bits));
		Bits = INTEL_ORDER64(Bits);
		FMemory::Memcpy(Out.GetData() + Index * sizeof(double), &Bits, sizeof(Bits));
	}
	return Out;
}

struct FFixture
{
	UWorld* World = nullptr;
	UURLabRenderServerWorldSubsystem* Workflow = nullptr;
	UURLabBridgeServer* Server = nullptr;
	UURLabZmqRpcTransport* Transport = nullptr;
	TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Binding;
	FURLabRpcDispatcher::FRenderServiceRegistration Registration;
	bool bOwnsWorld = false;

	bool Init(FString& Error, bool bUseEditorRenderWorld = false)
	{
		UURLabBridgeSubsystem::SetUnboundForTest(true);
		World = bUseEditorRenderWorld && GEditor
			? GEditor->GetEditorWorldContext().World() : nullptr;
		if (!World)
		{
			World = UWorld::CreateWorld(EWorldType::Game, false);
			bOwnsWorld = true;
		}
		if (!World || !GEngine) return false;
		if (bOwnsWorld)
			GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
		Workflow = World->GetSubsystem<UURLabRenderServerWorldSubsystem>();
		if (!Workflow || !Workflow->Activate(Error)) return false;
		Server = NewObject<UURLabBridgeServer>();
		Server->Start(TEXT(""));
		FMjCapabilitySet Grants;
		Grants.Set(EMjCapability::CameraRpc, true);
		Grants.Set(EMjCapability::AcceptModelLoad, true);
		Server->SetGrants(Grants);
		Binding = FURLabRenderServiceBinding::Create(*Workflow, Error);
		if (!Binding || !Server->GetDispatcher()->AttachRenderService(
			Binding.ToSharedRef(), Registration, Error)) return false;
		Transport = NewObject<UURLabZmqRpcTransport>(Server);
		Transport->SetOwningBridge(Server);
		return true;
	}

	~FFixture()
	{
		if (Server) Server->Stop();
		if (Workflow) Workflow->ShutdownWorkflow();
		if (World && bOwnsWorld)
		{
			World->DestroyWorld(false);
			if (GEngine) GEngine->DestroyWorldContext(World);
		}
		UURLabBridgeSubsystem::SetUnboundForTest(false);
	}
};

bool Connect(UURLabZmqRpcTransport& Transport, FURLabConnectReply& Out)
{
	FURLabConnectRequest Request;
	Request.ProtocolVersion = 2;
	Request.DataPolicy = TEXT("network");
	TArray<uint8> RequestBytes, ReplyBytes;
	return FURLabRequestEnvelopeCodec::EncodeConnectRequest(Request, RequestBytes).IsOk()
		&& Transport.ProcessRequestBytes(RequestBytes, ReplyBytes)
		&& FURLabRequestEnvelopeCodec::DecodeConnectReply(ReplyBytes, Out).IsOk();
}

bool RequestBytes(const FURLabConnectReply& Connection, uint64 Id, const FString& Op,
	V Body, TArray<uint8>& Out)
{
	FURLabRequestEnvelope Request;
	Request.ProtocolVersion = 2;
	Request.BootId = Connection.BootId;
	Request.SessionId = Connection.SessionId;
	Request.RequestId = Id;
	Request.Op = Op;
	Request.Body = MoveTemp(Body);
	return FURLabRequestEnvelopeCodec::Encode(Request, Out).IsOk();
}

bool Await(UURLabZmqRpcTransport& Transport, const TArray<uint8>& Request,
	TArray<uint8>& OutBytes, FURLabReplyEnvelope& Out)
{
	for (int32 Attempt = 0; Attempt < 400; ++Attempt)
	{
		if (!Transport.ProcessRequestBytes(Request, OutBytes)
			|| !FURLabRequestEnvelopeCodec::DecodeReply(OutBytes, Out).IsOk()) return false;
		if (Out.Kind != EURLabReplyKind::Pending) return true;
		FPlatformProcess::SleepNoStats(0.005f);
	}
	return false;
}

bool Send(UURLabZmqRpcTransport& Transport, const FURLabConnectReply& Connection,
	uint64 Id, const FString& Op, V Body, TArray<uint8>& OutBytes, FURLabReplyEnvelope& Out)
{
	TArray<uint8> Request;
	return RequestBytes(Connection, Id, Op, MoveTemp(Body), Request)
		&& Await(Transport, Request, OutBytes, Out);
}

V Proof(uint64 Generation, const FString& Token,
	std::initializer_list<TPair<FString, V>> Extra = {})
{
	TArray<FString> Keys({TEXT("generation"), TEXT("owner_token")});
	TArray<V> Values({V::MakeInt(static_cast<int64>(Generation)), V::MakeString(Token)});
	for (const auto& Field : Extra) { Keys.Add(Field.Key); Values.Add(Field.Value); }
	return V::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

bool Upload(UURLabZmqRpcTransport& Transport, const FURLabConnectReply& Session,
	uint64& Id, const TArray<uint8>& Data, FString& Blob, FString& Sha)
{
	Sha = FURLabAssetCache::Sha256Hex(Data);
	TArray<uint8> Bytes;
	FURLabReplyEnvelope Reply;
	if (!Send(Transport, Session, Id++, TEXT("payload_begin"), Map({
		{TEXT("transfer"), V::MakeString(TEXT("network"))},
		{TEXT("byte_length"), V::MakeInt(Data.Num())},
		{TEXT("sha256"), V::MakeString(Sha)}}), Bytes, Reply)
		|| Reply.Kind != EURLabReplyKind::Result) return false;
	Blob = Reply.Result.FindMapValue(TEXT("blob_id"))->GetString();
	if (!Send(Transport, Session, Id++, TEXT("payload_write"), Map({
		{TEXT("blob_id"), V::MakeString(Blob)}, {TEXT("offset"), V::MakeInt(0)},
		{TEXT("data"), V::MakeBinary(Data)}}), Bytes, Reply)
		|| Reply.Kind != EURLabReplyKind::Result) return false;
	return Send(Transport, Session, Id++, TEXT("payload_commit"), Map({
		{TEXT("blob_id"), V::MakeString(Blob)}}), Bytes, Reply)
		&& Reply.Kind == EURLabReplyKind::Result;
}

struct FGpuState
{
	FAutomationTestBase* Test = nullptr;
	TSharedPtr<FFixture> Fixture;
	FURLabConnectReply Session;
	uint64 Generation = 0;
	FString Token;
	FString SceneRevision;
	FString CameraName;
	int32 BaselineWidth = 0;
	int32 BaselineHeight = 0;
	int64 BaselineConfigurationRevision = 0;
	TArray<FString> Jobs;
	int32 CurrentJob = 0;
	int32 FirstRedPixels = 0;
	double FirstRedCentroidX = -1.0;
	FString FirstConfigurationId;
	FString BaselineGeometryConfigurationId;
	FString OverrideGeometryConfigurationId;
	int32 OverrideRedPixels = 0;
	double OverrideRedCentroidX = -1.0;
	uint64 NextId = 100;
	double Deadline = 0.0;
	double RenewAt = 0.0;
};
}

DEFINE_LATENT_AUTOMATION_COMMAND_ONE_PARAMETER(FURLabPumpTypedRenderCaptureGpu,
	TSharedPtr<URLabTypedRenderCaptureTest::FGpuState>, State);

bool FURLabPumpTypedRenderCaptureGpu::Update()
{
	using namespace URLabTypedRenderCaptureTest;
	if (!State || !State->Fixture || !State->Test) return true;
	// The real editor render world advances through the automation framework's
	// ordinary latent frames. Manual ticking is only for isolated native worlds.
	if (State->Fixture->bOwnsWorld)
	{
		FTSTicker::GetCoreTicker().Tick(1.0f / 60.0f);
		State->Fixture->World->Tick(LEVELTICK_All, 1.0f / 60.0f);
	}
	else if (URenderCameraSubsystem* Scheduler =
		State->Fixture->World->GetSubsystem<URenderCameraSubsystem>())
	{
		// Keep this isolated request/reply pump deterministic. Production editor
		// worlds drive this subsystem through IsTickableInEditor; this direct tick
		// only advances the scheduler inside the synchronous test dispatch loop.
		Scheduler->Tick(0.0f);
	}
	TArray<uint8> ReplyBytes;
	FURLabReplyEnvelope Reply;
	if (State->CurrentJob == 9)
	{
		if (State->Fixture->Binding->GetRetainedCaptureCountForTest() != 0)
		{
			if (FPlatformTime::Seconds() < State->Deadline) return false;
			State->Test->AddError(TEXT("expired owner capture record was not retired"));
			State->Fixture.Reset();
			return true;
		}
		State->Test->TestTrue(TEXT("expired owner permits successor attachment"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("render_owner_attach"), Map({{TEXT("ttl_s"), V::MakeFloat(30.0)}}),
				ReplyBytes, Reply) && Reply.Kind == EURLabReplyKind::Result);
		const V* Owner = Reply.Result.FindMapValue(TEXT("owner"));
		if (Owner)
		{
			const uint64 SuccessorGeneration = static_cast<uint64>(
				Owner->FindMapValue(TEXT("generation"))->GetInt());
			const FString SuccessorToken = Owner->FindMapValue(TEXT("token"))->GetString();
			State->Test->TestTrue(TEXT("successor old-job query reaches exact rejection"),
				Send(*State->Fixture->Transport, State->Session, State->NextId++,
					TEXT("render_capture_poll"), Proof(SuccessorGeneration, SuccessorToken,
						{{TEXT("job_id"), V::MakeString(State->Jobs[1])}}), ReplyBytes, Reply));
			State->Test->TestEqual(TEXT("expired owner's native job is unavailable"),
				Reply.Kind, EURLabReplyKind::Error);
			State->Test->TestTrue(TEXT("successor cannot read predecessor history context"),
				Send(*State->Fixture->Transport, State->Session, State->NextId++,
					TEXT("render_capture_history_query"), Proof(SuccessorGeneration, SuccessorToken, {
						{TEXT("expected_revision"), V::MakeString(State->SceneRevision)},
						{TEXT("geometry_configuration_id"),
							V::MakeString(State->BaselineGeometryConfigurationId)},
						{TEXT("cameras"), V::MakeArray({Map({
							{TEXT("name"), V::MakeString(State->CameraName)},
							{TEXT("configuration_id"), V::MakeString(State->FirstConfigurationId)}})})},
						{TEXT("selector"), Map({{TEXT("domain"), V::MakeString(TEXT("source"))},
							{TEXT("target_time"), V::MakeFloat(2.0)},
							{TEXT("clock_epoch"), V::MakeInt(1)}})}}), ReplyBytes, Reply)
					&& Reply.Kind == EURLabReplyKind::Error);
		}
		State->Fixture.Reset();
		return true;
	}
	if (FPlatformTime::Seconds() >= State->RenewAt)
	{
		if (!Send(*State->Fixture->Transport, State->Session, State->NextId++,
			TEXT("render_owner_renew"), Proof(State->Generation, State->Token,
				{{TEXT("ttl_s"), V::MakeFloat(30.0)}}), ReplyBytes, Reply)
			|| Reply.Kind != EURLabReplyKind::Result)
		{
			State->Test->AddError(TEXT("typed GPU owner renewal failed"));
			State->Fixture.Reset();
			return true;
		}
		State->RenewAt = FPlatformTime::Seconds() + 10.0;
	}
	if (!Send(*State->Fixture->Transport, State->Session, State->NextId++,
		TEXT("render_capture_poll"), Proof(State->Generation, State->Token,
			{{TEXT("job_id"), V::MakeString(State->Jobs[State->CurrentJob])}}), ReplyBytes, Reply))
	{
		State->Test->AddError(TEXT("typed GPU capture poll failed to dispatch"));
		State->Fixture.Reset();
		return true;
	}
	if (Reply.Kind != EURLabReplyKind::Result)
	{
		State->Test->AddError(Reply.Error.Message);
		State->Fixture.Reset();
		return true;
	}
	const V* GeometryIdValue = Reply.Result.FindMapValue(TEXT("geometry_configuration_id"));
	if (!GeometryIdValue)
	{
		State->Test->AddError(TEXT("capture poll omitted geometry_configuration_id"));
		State->Fixture.Reset();
		return true;
	}
	const FString GeometryConfigurationId = GeometryIdValue->GetString();
	if (State->CurrentJob == 7 || State->CurrentJob == 8)
	{
		const V* ReturnedFrame = Reply.Result.FindMapValue(TEXT("frame"));
		State->Test->TestTrue(TEXT("history preserves actual acquisition source time"),
			ReturnedFrame && ReturnedFrame->FindMapValue(TEXT("has_source_time"))->GetBool()
			&& FMath::IsNearlyEqual(ReturnedFrame->FindMapValue(TEXT("source_time"))->GetFloat(),
				State->CurrentJob == 7 ? 1.0 : 5.0));
	}
	const V* Material = Reply.Result.FindMapValue(TEXT("materialization"));
	const V* MaterialState = Material ? Material->FindMapValue(TEXT("state")) : nullptr;
	if (!MaterialState || MaterialState->GetString() == TEXT("pending"))
	{
		if (FPlatformTime::Seconds() < State->Deadline) return false;
		State->Test->AddError(TEXT("typed GPU capture did not materialize before deadline"));
		State->Fixture.Reset();
		return true;
	}
	if (MaterialState->GetString() != TEXT("succeeded"))
	{
		const V* Error = Material->FindMapValue(TEXT("error"));
		State->Test->AddError(Error ? Error->GetString() : TEXT("typed GPU materialization failed"));
		State->Fixture.Reset();
		return true;
	}
	const V* Payload = Material->FindMapValue(TEXT("payload"));
	const V* Cameras = Material->FindMapValue(TEXT("cameras"));
	if (!Payload || !Cameras || Cameras->GetArray().Num() != 1)
	{
		State->Test->AddError(TEXT("typed GPU materialization descriptor is incomplete"));
		State->Fixture.Reset();
		return true;
	}
	const FString Blob = Payload->FindMapValue(TEXT("blob_id"))->GetString();
	const int64 Length = Payload->FindMapValue(TEXT("byte_length"))->GetInt();
	const V* Optics = Cameras->GetArray()[0].FindMapValue(TEXT("optics"));
	const V* Resolution = Optics ? Optics->FindMapValue(TEXT("resolution")) : nullptr;
	if (!Resolution) { State->Test->AddError(TEXT("capture metadata omitted effective optics")); State->Fixture.Reset(); return true; }
	const int32 Width = Resolution->FindMapValue(TEXT("width"))->GetInt();
	const int32 Height = Resolution->FindMapValue(TEXT("height"))->GetInt();
	const V& CameraMetadata = Cameras->GetArray()[0];
	const V* ModeValue = CameraMetadata.FindMapValue(TEXT("mode"));
	const V* DepthNearValue = CameraMetadata.FindMapValue(TEXT("depth_near_m"));
	const V* DepthFarValue = CameraMetadata.FindMapValue(TEXT("depth_far_m"));
	const V* ConfigurationIdValue = CameraMetadata.FindMapValue(TEXT("configuration_id"));
	if (!ModeValue || !DepthNearValue || !DepthFarValue || !ConfigurationIdValue)
	{
		State->Test->AddError(TEXT("capture metadata omitted mode or effective depth range"));
		State->Fixture.Reset();
		return true;
	}
	const FString EffectiveMode = ModeValue->GetString();
	const double EffectiveDepthNearMetres = DepthNearValue->GetFloat();
	const double EffectiveDepthFarMetres = DepthFarValue->GetFloat();
	const FString ConfigurationId = ConfigurationIdValue->GetString();
	State->Test->TestFalse(TEXT("capture metadata includes canonical configuration identity"),
		ConfigurationId.IsEmpty());
	State->Test->TestEqual(TEXT("capture uses requested override width"), Width, 96);
	State->Test->TestEqual(TEXT("capture uses requested override height"), Height, 64);
	TArray<uint8> Pixels;
	const V* NegotiatedReadLimit =
		State->Session.Limits.FindMapValue(TEXT("payload_max_chunk_bytes"));
	if (!NegotiatedReadLimit || NegotiatedReadLimit->GetInt() <= 0)
	{
		State->Test->AddError(TEXT("connect reply omitted a positive payload read limit"));
		State->Fixture.Reset();
		return true;
	}
	const int64 ReadChunkBytes = NegotiatedReadLimit->GetInt();
	for (int64 Offset = 0; Offset < Length; Offset += ReadChunkBytes)
	{
		const int64 Count = FMath::Min<int64>(ReadChunkBytes, Length - Offset);
		if (!Send(*State->Fixture->Transport, State->Session, State->NextId++,
			TEXT("payload_read"), Map({{TEXT("blob_id"), V::MakeString(Blob)},
				{TEXT("offset"), V::MakeInt(Offset)}, {TEXT("byte_length"), V::MakeInt(Count)}}),
			ReplyBytes, Reply) || Reply.Kind != EURLabReplyKind::Result)
		{
			State->Test->AddError(FString::Printf(
				TEXT("typed GPU payload read failed at offset %lld length %lld: %s %s"),
				static_cast<long long>(Offset), static_cast<long long>(Count),
				*Reply.Error.Code, *Reply.Error.Message));
			State->Fixture.Reset();
			return true;
		}
		Pixels.Append(Reply.Result.FindMapValue(TEXT("data"))->GetBinary());
	}
	if (State->CurrentJob == 2)
	{
		State->Test->TestEqual(TEXT("depth result reports effective mode"),
			EffectiveMode, FString(TEXT("depth")));
		State->Test->TestTrue(TEXT("depth result reports effective range in metres"),
			FMath::IsNearlyEqual(EffectiveDepthNearMetres, 0.02, 0.0001)
			&& FMath::IsNearlyEqual(EffectiveDepthFarMetres, 2.0, 0.0001));
		State->Test->TestEqual(TEXT("depth payload has one float per requested pixel"),
			Pixels.Num(), Width * Height * static_cast<int32>(sizeof(float)));
		int32 VisibleDepthPixels = 0;
		int32 FarDepthPixels = 0;
		float CenterDepthMetres = 0.0f;
		for (int32 Index = 0; Index + 3 < Pixels.Num(); Index += 4)
		{
			uint32 Bits = static_cast<uint32>(Pixels[Index])
				| (static_cast<uint32>(Pixels[Index + 1]) << 8)
				| (static_cast<uint32>(Pixels[Index + 2]) << 16)
				| (static_cast<uint32>(Pixels[Index + 3]) << 24);
			float Metres = 0.0f;
			FMemory::Memcpy(&Metres, &Bits, sizeof(Metres));
			if (Metres >= 1.0f && Metres < 1.5f) ++VisibleDepthPixels;
			if (FMath::IsNearlyEqual(Metres, 2.0f, 0.001f)) ++FarDepthPixels;
			if (Index / 4 == (Height / 2) * Width + Width / 2)
				CenterDepthMetres = Metres;
		}
		State->Test->AddInfo(FString::Printf(
			TEXT("typed depth: center=%.4fm visible=%d far=%d total=%d"),
			CenterDepthMetres, VisibleDepthPixels, FarDepthPixels, Width * Height));
		State->Test->TestTrue(TEXT("depth captures the sphere at camera-forward axial metres"),
			VisibleDepthPixels > 10 && CenterDepthMetres >= 1.0f && CenterDepthMetres < 1.5f);
		State->Test->TestTrue(TEXT("depth background and beyond-range samples use effective far metres"),
			FarDepthPixels > 10);
		State->Test->TestTrue(TEXT("materialized depth payload releases through its data session"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("payload_release"), Map({{TEXT("blob_id"), V::MakeString(Blob)}}),
				ReplyBytes, Reply) && Reply.Kind == EURLabReplyKind::Result);
		State->Test->TestTrue(TEXT("released depth descriptor remains native success"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("render_capture_poll"), Proof(State->Generation, State->Token,
					{{TEXT("job_id"), V::MakeString(State->Jobs[State->CurrentJob])}}),
				ReplyBytes, Reply) && Reply.Kind == EURLabReplyKind::Result);
		if (Reply.Kind == EURLabReplyKind::Result)
		{
			State->Test->TestEqual(TEXT("depth descriptor release preserves native success"),
				Reply.Result.FindMapValue(TEXT("state"))->GetString(), FString(TEXT("succeeded")));
			const V* ReleasedMaterial = Reply.Result.FindMapValue(TEXT("materialization"));
			State->Test->TestEqual(TEXT("released depth descriptor becomes materialization failure"),
				ReleasedMaterial->FindMapValue(TEXT("state"))->GetString(), FString(TEXT("failed")));
		}
		++State->CurrentJob;
		State->Deadline = FPlatformTime::Seconds() + 25.0;
		return false;
	}
	if (State->CurrentJob == 3 || State->CurrentJob == 4)
	{
		const bool bInstance = State->CurrentJob == 3;
		State->Test->TestEqual(TEXT("segmentation result reports requested effective mode"),
			EffectiveMode, FString(bInstance ? TEXT("instance") : TEXT("semantic")));
		State->Test->TestEqual(TEXT("segmentation payload stores two int32 lanes per pixel"),
			Pixels.Num(), Width * Height * 2 * static_cast<int32>(sizeof(int32)));
		TSet<int32> ForegroundIds;
		int32 Foreground = 0;
		int32 Background = 0;
		int32 InvalidKinds = 0;
		for (int32 Offset = 0; Offset + 7 < Pixels.Num(); Offset += 8)
		{
			auto ReadI32 = [&Pixels](int32 At)
			{
				const uint32 Bits = static_cast<uint32>(Pixels[At])
					| (static_cast<uint32>(Pixels[At + 1]) << 8)
					| (static_cast<uint32>(Pixels[At + 2]) << 16)
					| (static_cast<uint32>(Pixels[At + 3]) << 24);
				return static_cast<int32>(Bits);
			};
			const int32 ObjectId = ReadI32(Offset);
			const int32 Kind = ReadI32(Offset + 4);
			if (ObjectId < 0)
			{
				if (Kind != -1) ++InvalidKinds;
				++Background;
			}
			else
			{
				if (Kind != (bInstance ? 1 : 2)) ++InvalidKinds;
				ForegroundIds.Add(ObjectId);
				++Foreground;
			}
		}
		State->Test->AddInfo(FString::Printf(TEXT("typed %s segmentation: foreground=%d background=%d ids=%d"),
			bInstance ? TEXT("instance") : TEXT("semantic"), Foreground, Background,
			ForegroundIds.Num()));
		State->Test->TestTrue(TEXT("segmentation contains both neutral scene objects"),
			ForegroundIds.Num() == 2);
		State->Test->TestEqual(TEXT("segmentation pixels use exact neutral kinds"),
			InvalidKinds, 0);
		State->Test->TestTrue(TEXT("segmentation retains a real background region"), Background > 10);
		State->Test->TestTrue(TEXT("segmentation result payload releases"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("payload_release"), Map({{TEXT("blob_id"), V::MakeString(Blob)}}),
				ReplyBytes, Reply) && Reply.Kind == EURLabReplyKind::Result);
		if (bInstance)
		{
			State->Test->TestTrue(TEXT("instance ids are authoritative geom array indices"),
				ForegroundIds.Contains(0) && ForegroundIds.Contains(1));
			++State->CurrentJob;
			State->Deadline = FPlatformTime::Seconds() + 25.0;
			return false;
		}
		State->Test->TestTrue(TEXT("semantic ids are authoritative body array indices"),
			ForegroundIds.Contains(1) && ForegroundIds.Contains(2));
		State->CurrentJob = 5;
		State->Deadline = FPlatformTime::Seconds() + 25.0;
		return false;
	}
	int32 RedPixels = 0;
	int32 NonRedPixels = 0;
	double RedX = 0.0;
	uint8 MaxR = 0;
	uint8 MaxG = 0;
	uint8 MaxB = 0;
	for (int32 Index = 0; Index + 3 < Pixels.Num(); Index += 4)
	{
		MaxR = FMath::Max(MaxR, Pixels[Index]);
		MaxG = FMath::Max(MaxG, Pixels[Index + 1]);
		MaxB = FMath::Max(MaxB, Pixels[Index + 2]);
		const bool bRed = Pixels[Index] > Pixels[Index + 1] + 30
			&& Pixels[Index] > Pixels[Index + 2] + 30;
		if (bRed)
		{
			++RedPixels;
			RedX += (Index / 4) % Width;
		}
		else
		{
			++NonRedPixels;
		}
	}
	State->Test->AddInfo(FString::Printf(
		TEXT("typed capture pixels: maxRGB=(%u,%u,%u) red=%d nonred=%d total=%d"),
		MaxR, MaxG, MaxB, RedPixels, NonRedPixels, Pixels.Num() / 4));
	State->Test->TestTrue(TEXT("typed service payload contains visible red geometry"),
		RedPixels > 10);
	State->Test->TestTrue(TEXT("red geometry does not fill a uniform frame"),
		NonRedPixels > 10);
	int32 CenterRedPixels = 0;
	for (int32 Y = Height / 2 - 2; Y <= Height / 2 + 2; ++Y)
	{
		for (int32 X = Width / 2 - 2; X <= Width / 2 + 2; ++X)
		{
			if (X < 0 || X >= Width || Y < 0 || Y >= Height) continue;
			const int32 Index = (Y * Width + X) * 4;
			if (Pixels.IsValidIndex(Index + 2) && Pixels[Index] > Pixels[Index + 1] + 30
				&& Pixels[Index] > Pixels[Index + 2] + 30)
				++CenterRedPixels;
		}
	}
	if (State->CurrentJob != 5 && State->CurrentJob != 8)
	{
		State->Test->TestTrue(TEXT("baseline optical camera centers the red geometry"),
			CenterRedPixels > 0);
	}
	State->Test->TestEqual(TEXT("typed payload has exact descriptor length"),
		Pixels.Num(), static_cast<int32>(Length));
	if (State->CurrentJob == 0)
	{
		Send(*State->Fixture->Transport, State->Session, State->NextId++,
			TEXT("render_capture_release"), Proof(State->Generation, State->Token,
				{{TEXT("job_id"), V::MakeString(State->Jobs[State->CurrentJob])}}),
			ReplyBytes, Reply);
		State->FirstRedPixels = RedPixels;
		State->FirstRedCentroidX = RedPixels > 0 ? RedX / RedPixels : -1.0;
		State->FirstConfigurationId = ConfigurationId;
		const V HistorySelector = Map({{TEXT("domain"), V::MakeString(TEXT("source"))},
			{TEXT("target_time"), V::MakeFloat(2.0)}, {TEXT("clock_epoch"), V::MakeInt(1)}});
		State->Test->TestTrue(TEXT("history rejects a different effective configuration"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("render_capture_history_query"), Proof(State->Generation, State->Token, {
					{TEXT("expected_revision"), V::MakeString(State->SceneRevision)},
					{TEXT("geometry_configuration_id"),
						V::MakeString(State->BaselineGeometryConfigurationId)},
					{TEXT("cameras"), V::MakeArray({Map({
						{TEXT("name"), V::MakeString(State->CameraName)},
						{TEXT("configuration_id"), V::MakeString(TEXT("wrong"))}})})},
					{TEXT("selector"), HistorySelector}}), ReplyBytes, Reply)
				&& Reply.Kind == EURLabReplyKind::Error);
		const V WrongEpochSelector = Map({{TEXT("domain"), V::MakeString(TEXT("source"))},
			{TEXT("target_time"), V::MakeFloat(1.0)}, {TEXT("clock_epoch"), V::MakeInt(2)}});
		State->Test->TestTrue(TEXT("history rejects a different clock epoch"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("render_capture_history_query"), Proof(State->Generation, State->Token, {
					{TEXT("expected_revision"), V::MakeString(State->SceneRevision)},
					{TEXT("geometry_configuration_id"),
						V::MakeString(State->BaselineGeometryConfigurationId)},
					{TEXT("cameras"), V::MakeArray({Map({
						{TEXT("name"), V::MakeString(State->CameraName)},
						{TEXT("configuration_id"), V::MakeString(ConfigurationId)}})})},
					{TEXT("selector"), WrongEpochSelector}}), ReplyBytes, Reply)
				&& Reply.Kind == EURLabReplyKind::Error);
		State->Test->TestTrue(TEXT("fresh capture seeds exact retained history"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("render_capture_history_query"), Proof(State->Generation, State->Token, {
					{TEXT("expected_revision"), V::MakeString(State->SceneRevision)},
					{TEXT("geometry_configuration_id"),
						V::MakeString(State->BaselineGeometryConfigurationId)},
					{TEXT("cameras"), V::MakeArray({Map({
						{TEXT("name"), V::MakeString(State->CameraName)},
						{TEXT("configuration_id"), V::MakeString(ConfigurationId)}})})},
					{TEXT("selector"), HistorySelector}}), ReplyBytes, Reply)
				&& Reply.Kind == EURLabReplyKind::Result);
		if (Reply.Kind == EURLabReplyKind::Result)
			State->Jobs.Add(Reply.Result.FindMapValue(TEXT("job_id"))->GetString());
		++State->CurrentJob;
		State->Deadline = FPlatformTime::Seconds() + 25.0;
		return false;
	}
	if (State->CurrentJob == 7)
	{
		State->Test->TestEqual(TEXT("baseline history preserves geometry identity"),
			GeometryConfigurationId, State->BaselineGeometryConfigurationId);
		State->Test->TestEqual(TEXT("history materialization preserves exact configuration"),
			ConfigurationId, State->FirstConfigurationId);
		State->Test->TestTrue(TEXT("history query reuses the retained fresh pixels"),
			FMath::Abs(RedPixels - State->FirstRedPixels)
				<= FMath::Max(20, State->FirstRedPixels / 10));
		State->Test->TestTrue(TEXT("history payload releases through normal materializer"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("payload_release"), Map({{TEXT("blob_id"), V::MakeString(Blob)}}),
				ReplyBytes, Reply) && Reply.Kind == EURLabReplyKind::Result);
		++State->CurrentJob;
		State->Deadline = FPlatformTime::Seconds() + 25.0; return false;
	}
	if (State->CurrentJob == 8)
	{
		State->Test->TestEqual(TEXT("override history preserves camera configuration"),
			ConfigurationId, State->FirstConfigurationId);
		State->Test->TestEqual(TEXT("override history preserves geometry identity"),
			GeometryConfigurationId, State->OverrideGeometryConfigurationId);
		State->Test->TestTrue(TEXT("override history reuses changed geometry pixels"),
			FMath::Abs(RedPixels - State->OverrideRedPixels)
				<= FMath::Max(20, State->OverrideRedPixels / 10));
		State->Test->TestTrue(TEXT("override history payload releases"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("payload_release"), Map({{TEXT("blob_id"), V::MakeString(Blob)}}),
				ReplyBytes, Reply) && Reply.Kind == EURLabReplyKind::Result);
		State->Test->TestTrue(TEXT("owner short renewal prepares history-context cleanup"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("render_owner_renew"), Proof(State->Generation, State->Token,
					{{TEXT("ttl_s"), V::MakeFloat(0.05)}}), ReplyBytes, Reply)
				&& Reply.Kind == EURLabReplyKind::Result);
		State->CurrentJob = 9;
		State->RenewAt = TNumericLimits<double>::Max();
		State->Deadline = FPlatformTime::Seconds() + 3.0;
		return false;
	}
	if (State->CurrentJob == 5)
	{
		State->Test->TestEqual(TEXT("override poll preserves submitted geometry identity"),
			GeometryConfigurationId, State->OverrideGeometryConfigurationId);
		State->Test->TestNotEqual(TEXT("override geometry identity differs from baseline"),
			GeometryConfigurationId, State->BaselineGeometryConfigurationId);
		State->OverrideRedPixels = RedPixels;
		State->OverrideRedCentroidX = RedPixels > 0 ? RedX / RedPixels : -1.0;
		State->Test->TestTrue(TEXT("geom override visibly moves the red geometry"),
			FMath::Abs(State->OverrideRedCentroidX - State->FirstRedCentroidX) > 3.0);
		const V Selector = Map({{TEXT("domain"), V::MakeString(TEXT("source"))},
			{TEXT("target_time"), V::MakeFloat(5.0)}, {TEXT("clock_epoch"), V::MakeInt(1)}});
		State->Test->TestTrue(TEXT("override capture seeds geometry-scoped history"),
			Send(*State->Fixture->Transport, State->Session, State->NextId++,
				TEXT("render_capture_history_query"), Proof(State->Generation, State->Token, {
					{TEXT("expected_revision"), V::MakeString(State->SceneRevision)},
					{TEXT("geometry_configuration_id"), V::MakeString(GeometryConfigurationId)},
					{TEXT("cameras"), V::MakeArray({Map({
						{TEXT("name"), V::MakeString(State->CameraName)},
						{TEXT("configuration_id"), V::MakeString(ConfigurationId)}})})},
					{TEXT("selector"), Selector}}), ReplyBytes, Reply)
				&& Reply.Kind == EURLabReplyKind::Result);
		if (Reply.Kind == EURLabReplyKind::Result)
			State->Jobs.Add(Reply.Result.FindMapValue(TEXT("job_id"))->GetString());
		++State->CurrentJob;
		State->Deadline = FPlatformTime::Seconds() + 25.0;
		return false;
	}
	if (State->CurrentJob == 6)
	{
		State->Test->TestEqual(TEXT("next baseline capture restores geometry identity"),
			GeometryConfigurationId, State->BaselineGeometryConfigurationId);
		const double RestoredCentroid = RedPixels > 0 ? RedX / RedPixels : -1.0;
		State->Test->AddInfo(FString::Printf(
			TEXT("geom restore: baseline centroid=%.3f pixels=%d restored centroid=%.3f pixels=%d"),
			State->FirstRedCentroidX, State->FirstRedPixels, RestoredCentroid, RedPixels));
		State->Test->TestTrue(TEXT("next baseline capture restores geometry pixels"),
			FMath::Abs(RestoredCentroid - State->FirstRedCentroidX) <= 3.0);
		++State->CurrentJob;
		State->Deadline = FPlatformTime::Seconds() + 25.0;
		return false;
	}
	const double SecondCentroid = RedPixels > 0 ? RedX / RedPixels : -1.0;
	State->Test->TestTrue(TEXT("Cartesian and qpos captures have comparable red area"),
		FMath::Abs(RedPixels - State->FirstRedPixels)
			<= FMath::Max(20, State->FirstRedPixels / 10));
	State->Test->TestTrue(TEXT("Cartesian and qpos red regions align semantically"),
		FMath::Abs(SecondCentroid - State->FirstRedCentroidX) <= 3.0);
	State->Test->TestTrue(TEXT("baseline describe remains available after overrides"),
		Send(*State->Fixture->Transport, State->Session, State->NextId++,
			TEXT("render_scene_describe"), Proof(State->Generation, State->Token,
				{{TEXT("expected_revision"), V::MakeString(State->SceneRevision)}}),
			ReplyBytes, Reply) && Reply.Kind == EURLabReplyKind::Result);
	if (Reply.Kind == EURLabReplyKind::Result)
	{
		const V* DescribedCameras = Reply.Result.FindMapValue(TEXT("model_cameras"));
		const V* BaselineCamera = nullptr;
		if (DescribedCameras)
		{
			for (const V& Candidate : DescribedCameras->GetArray())
			{
				const V* Name = Candidate.FindMapValue(TEXT("name"));
				if (Name && Name->GetString() == State->CameraName)
				{
					BaselineCamera = &Candidate;
					break;
				}
			}
		}
		State->Test->TestNotNull(TEXT("describe retains the baseline camera"), BaselineCamera);
		if (BaselineCamera)
		{
			const V* BaselineOptics = BaselineCamera->FindMapValue(TEXT("optics"));
			const V* BaselineResolution = BaselineOptics
				? BaselineOptics->FindMapValue(TEXT("resolution")) : nullptr;
			State->Test->TestNotNull(TEXT("baseline camera retains optics"), BaselineResolution);
			if (BaselineResolution)
			{
				State->Test->TestEqual(TEXT("override restores baseline width"),
					static_cast<int32>(BaselineResolution->FindMapValue(TEXT("width"))->GetInt()),
					State->BaselineWidth);
				State->Test->TestEqual(TEXT("override restores baseline height"),
					static_cast<int32>(BaselineResolution->FindMapValue(TEXT("height"))->GetInt()),
					State->BaselineHeight);
			}
			State->Test->TestEqual(TEXT("override preserves baseline configuration revision"),
				BaselineCamera->FindMapValue(TEXT("baseline_configuration_revision"))->GetInt(),
				State->BaselineConfigurationRevision);
		}
	}
	++State->CurrentJob;
	State->Deadline = FPlatformTime::Seconds() + 25.0;
	return false;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedRenderCaptureContractTest,
	"URLab.TypedRenderCapture.DescribeQposZeroReplayAndReplacement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedRenderCaptureContractTest::RunTest(const FString&)
{
	using namespace URLabTypedRenderCaptureTest;
	FFixture Fixture;
	FString Error;
	if (!Fixture.Init(Error)) { AddError(Error); return false; }
	FURLabConnectReply Session;
	if (!Connect(*Fixture.Transport, Session))
	{
		AddError(TEXT("typed capture fixture failed to connect"));
		return false;
	}
	uint64 Id = 1;
	TArray<uint8> Bytes;
	FURLabReplyEnvelope Reply;
	if (!Send(*Fixture.Transport, Session, Id++, TEXT("render_owner_attach"),
		Map({{TEXT("ttl_s"), V::MakeFloat(30.0)}}), Bytes, Reply)
		|| Reply.Kind != EURLabReplyKind::Result)
	{
		AddError(FString::Printf(TEXT("render owner attach failed: %s %s"),
			*Reply.Error.Code, *Reply.Error.Message));
		return false;
	}
	const V* Owner = Reply.Result.FindMapValue(TEXT("owner"));
	if (!Owner) { AddError(TEXT("render owner attach omitted owner result")); return false; }
	const uint64 Generation = static_cast<uint64>(Owner->FindMapValue(TEXT("generation"))->GetInt());
	const FString Token = Owner->FindMapValue(TEXT("token"))->GetString();
	const char* Xml = "<mujoco model='typed_capture'><worldbody><body><geom type='sphere' size='0.1' mass='1'/><camera name='exact'/></body></worldbody></mujoco>";
	FString Blob, Sha;
	if (!Upload(*Fixture.Transport, Session, Id, TextBytes(Xml), Blob, Sha))
	{
		AddError(TEXT("typed capture XML payload upload failed"));
		return false;
	}
	const V Source = Map({{TEXT("format"), V::MakeString(TEXT("xml"))},
		{TEXT("primary"), Map({{TEXT("blob_id"), V::MakeString(Blob)},
			{TEXT("sha256"), V::MakeString(Sha)}})},
		{TEXT("assets"), V::MakeArray(TArray<V>())}});
	if (!Send(*Fixture.Transport, Session, Id++, TEXT("render_source_install"),
		Proof(Generation, Token, {{TEXT("expected_revision"), V::MakeString(TEXT(""))},
			{TEXT("source"), Source}, {TEXT("options"), Map({
				{TEXT("visible_group_mask"), V::MakeInt(7)}})}}), Bytes, Reply)
		|| Reply.Kind != EURLabReplyKind::Result)
	{
		AddError(FString::Printf(TEXT("render source install failed: %s %s"),
			*Reply.Error.Code, *Reply.Error.Message));
		return false;
	}
	const FString Revision = Reply.Result.FindMapValue(TEXT("scene_revision"))->GetString();
	if (!Send(*Fixture.Transport, Session, Id++, TEXT("render_scene_describe"),
		Proof(Generation, Token, {{TEXT("expected_revision"), V::MakeString(Revision)}}),
		Bytes, Reply) || Reply.Kind != EURLabReplyKind::Result)
	{
		AddError(FString::Printf(TEXT("render scene describe failed: %s %s"),
			*Reply.Error.Code, *Reply.Error.Message));
		return false;
	}
	TestTrue(TEXT("joint-free source explicitly supports zero-width qpos"),
		Reply.Result.FindMapValue(TEXT("supports_qpos"))->GetBool()
		&& Reply.Result.FindMapValue(TEXT("qpos_width"))->GetInt() == 0);
	const V* Cameras = Reply.Result.FindMapValue(TEXT("model_cameras"));
	if (!Cameras || Cameras->GetArray().IsEmpty())
	{
		AddError(TEXT("render scene describe omitted the installed model camera"));
		return false;
	}
	const FString CameraName = Cameras->GetArray()[0].FindMapValue(TEXT("name"))->GetString();
	const V Frame = Map({{TEXT("frame_id"), V::MakeInt(9)},
		{TEXT("clock_epoch"), V::MakeInt(2)}, {TEXT("has_source_time"), V::MakeBool(false)}});
	TestTrue(TEXT("source-time presence mismatch dispatches a typed rejection"), Send(
		*Fixture.Transport, Session, Id++, TEXT("render_capture_submit"),
		Proof(Generation, Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)},
			{TEXT("cameras"), V::MakeArray({Map({{TEXT("name"), V::MakeString(CameraName)}})})},
			{TEXT("timeout_s"), V::MakeFloat(5.0)},
			{TEXT("frame"), Map({{TEXT("frame_id"), V::MakeInt(8)},
				{TEXT("clock_epoch"), V::MakeInt(2)},
				{TEXT("has_source_time"), V::MakeBool(true)}})},
			{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("qpos"))}})}}),
		Bytes, Reply));
	TestEqual(TEXT("has_source_time true requires source_time"), Reply.Kind,
		EURLabReplyKind::Error);
	TestTrue(TEXT("partial depth override dispatches rejection"), Send(*Fixture.Transport,
		Session, Id++, TEXT("render_capture_submit"), Proof(Generation, Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)},
			{TEXT("cameras"), V::MakeArray({Map({{TEXT("name"), V::MakeString(CameraName)},
				{TEXT("override"), Map({{TEXT("mode"), V::MakeString(TEXT("depth"))},
					{TEXT("depth_near_m"), V::MakeFloat(0.1)}})}})})},
			{TEXT("timeout_s"), V::MakeFloat(5.0)}, {TEXT("frame"), Frame},
			{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("qpos"))}})}}),
		Bytes, Reply));
	TestEqual(TEXT("depth override requires a complete pair"), Reply.Error.Code,
		FString(TEXT("bad_request")));
	const V Submit = Proof(Generation, Token, {
		{TEXT("expected_revision"), V::MakeString(Revision)},
		{TEXT("cameras"), V::MakeArray({Map({{TEXT("name"), V::MakeString(CameraName)}})})},
		{TEXT("timeout_s"), V::MakeFloat(5.0)}, {TEXT("frame"), Frame},
		{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("qpos"))}})}});
	TArray<uint8> SubmitRequest;
	if (!RequestBytes(Session, Id++, TEXT("render_capture_submit"), Submit, SubmitRequest))
		return false;
	TArray<uint8> First;
	if (!Await(*Fixture.Transport, SubmitRequest, First, Reply)
		|| Reply.Kind != EURLabReplyKind::Result)
	{
		AddError(FString::Printf(TEXT("qpos-zero capture submit failed: %s %s"),
			*Reply.Error.Code, *Reply.Error.Message));
		return false;
	}
	const FString Job = Reply.Result.FindMapValue(TEXT("job_id"))->GetString();
	TArray<uint8> Duplicate;
	TestTrue(TEXT("duplicate submit dispatches"),
		Await(*Fixture.Transport, SubmitRequest, Duplicate, Reply));
	TestEqual(TEXT("submit replay is exact and names one native job"), Duplicate, First);
	TestFalse(TEXT("opaque job id is nonempty"), Job.IsEmpty());
	TestTrue(TEXT("client cannot override negotiated backing"), Send(*Fixture.Transport,
		Session, Id++, TEXT("render_capture_submit"), Map({
			{TEXT("generation"), V::MakeInt(Generation)}, {TEXT("owner_token"), V::MakeString(Token)},
			{TEXT("expected_revision"), V::MakeString(Revision)},
			{TEXT("cameras"), V::MakeArray({Map({{TEXT("name"), V::MakeString(CameraName)}})})},
			{TEXT("timeout_s"), V::MakeFloat(5.0)}, {TEXT("frame"), Frame},
			{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("qpos"))}})},
			{TEXT("backing"), V::MakeString(TEXT("shm"))}}), Bytes, Reply));
	TestEqual(TEXT("backing override fails exact request validation"), Reply.Error.Execution,
		EURLabExecution::Rejected);
	TestTrue(TEXT("source replacement commits"), Send(*Fixture.Transport, Session, Id++,
		TEXT("render_source_install"), Proof(Generation, Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)}, {TEXT("source"), Source},
			{TEXT("options"), Map({{TEXT("visible_group_mask"), V::MakeInt(7)}})}}),
		Bytes, Reply) && Reply.Kind == EURLabReplyKind::Result);
	TestTrue(TEXT("old-scene job remains pollable after replacement"), Send(
		*Fixture.Transport, Session, Id++, TEXT("render_capture_poll"),
		Proof(Generation, Token, {{TEXT("job_id"), V::MakeString(Job)}}), Bytes, Reply));
	TestEqual(TEXT("replacement cancels queued old-scene capture"),
		Reply.Result.FindMapValue(TEXT("state"))->GetString(), FString(TEXT("cancelled")));
	const V* PolledFrame = Reply.Result.FindMapValue(TEXT("frame"));
	TestTrue(TEXT("replacement poll retains the original frame identity"),
		PolledFrame != nullptr);
	if (PolledFrame)
	{
		const V* PolledRevision = PolledFrame->FindMapValue(TEXT("scene_revision"));
		TestTrue(TEXT("replacement poll returns an original scene revision"),
			PolledRevision != nullptr);
		if (PolledRevision)
			TestEqual(TEXT("replacement poll retains original scene revision"),
				PolledRevision->GetString(), Revision);
	}
	TestTrue(TEXT("explicit owner release retires its exact capture records"), Send(
		*Fixture.Transport, Session, Id++, TEXT("render_owner_release"),
		Proof(Generation, Token), Bytes, Reply) && Reply.Kind == EURLabReplyKind::Result);
	TestTrue(TEXT("same protocol scope can attach a successor owner"), Send(
		*Fixture.Transport, Session, Id++, TEXT("render_owner_attach"),
		Map({{TEXT("ttl_s"), V::MakeFloat(30.0)}}), Bytes, Reply)
		&& Reply.Kind == EURLabReplyKind::Result);
	const V* Successor = Reply.Result.FindMapValue(TEXT("owner"));
	if (!Successor) return false;
	const uint64 SuccessorGeneration = static_cast<uint64>(
		Successor->FindMapValue(TEXT("generation"))->GetInt());
	const FString SuccessorToken = Successor->FindMapValue(TEXT("token"))->GetString();
	TestTrue(TEXT("successor cannot address retired predecessor job"), Send(
		*Fixture.Transport, Session, Id++, TEXT("render_capture_poll"),
		Proof(SuccessorGeneration, SuccessorToken,
			{{TEXT("job_id"), V::MakeString(Job)}}), Bytes, Reply));
	TestEqual(TEXT("retired predecessor job is absent"), Reply.Kind, EURLabReplyKind::Error);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedRenderCaptureGpuTest,
	"URLab.TypedRenderCaptureGPU.QposCartesianEquivalentStoredPixels",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabTypedRenderCaptureGpuTest::RunTest(const FString&)
{
	using namespace URLabTypedRenderCaptureTest;
	if (!FApp::CanEverRender())
	{
		AddInfo(TEXT("No RHI: typed render capture GPU path requires offscreen rendering"));
		return true;
	}
	auto State = MakeShared<FGpuState>();
	State->Test = this;
	State->Fixture = MakeShared<FFixture>();
	FString Error;
	if (!State->Fixture->Init(Error, true)) { AddError(Error); return false; }
	if (!Connect(*State->Fixture->Transport, State->Session)) return false;
	uint64 Id = 1;
	TArray<uint8> ReplyBytes;
	FURLabReplyEnvelope Reply;
	if (!Send(*State->Fixture->Transport, State->Session, Id++, TEXT("render_owner_attach"),
		Map({{TEXT("ttl_s"), V::MakeFloat(30.0)}}), ReplyBytes, Reply)) return false;
	const V* Owner = Reply.Result.FindMapValue(TEXT("owner"));
	if (!Owner) return false;
	State->Generation = static_cast<uint64>(
		Owner->FindMapValue(TEXT("generation"))->GetInt());
	State->Token = Owner->FindMapValue(TEXT("token"))->GetString();
	const char* Xml = "<mujoco model='typed_gpu'><asset><material name='red_emissive' rgba='1 0.01 0.01 1' emission='3'/></asset><worldbody><body name='target'><geom type='sphere' size='0.25' material='red_emissive'/><camera name='exact' pos='0 0 1.5'/></body><body name='second' pos='0.55 0 0'><geom type='sphere' size='0.18' material='red_emissive'/></body></worldbody></mujoco>";
	FString Blob, Sha;
	if (!Upload(*State->Fixture->Transport, State->Session, Id, TextBytes(Xml), Blob, Sha))
		return false;
	const V Source = Map({{TEXT("format"), V::MakeString(TEXT("xml"))},
		{TEXT("primary"), Map({{TEXT("blob_id"), V::MakeString(Blob)},
			{TEXT("sha256"), V::MakeString(Sha)}})},
		{TEXT("assets"), V::MakeArray(TArray<V>())}});
	if (!Send(*State->Fixture->Transport, State->Session, Id++,
		TEXT("render_source_install"), Proof(State->Generation, State->Token, {
			{TEXT("expected_revision"), V::MakeString(TEXT(""))}, {TEXT("source"), Source},
			{TEXT("options"), Map({{TEXT("visible_group_mask"), V::MakeInt(7)}})}}),
		ReplyBytes, Reply) || Reply.Kind != EURLabReplyKind::Result) return false;
	const FString Revision = Reply.Result.FindMapValue(TEXT("scene_revision"))->GetString();
	if (!Send(*State->Fixture->Transport, State->Session, Id++,
		TEXT("render_scene_describe"), Proof(State->Generation, State->Token,
			{{TEXT("expected_revision"), V::MakeString(Revision)}}), ReplyBytes, Reply)
		|| Reply.Kind != EURLabReplyKind::Result) return false;
	const V* Cameras = Reply.Result.FindMapValue(TEXT("model_cameras"));
	if (!Cameras || Cameras->GetArray().IsEmpty()) return false;
	const FString CameraName = Cameras->GetArray()[0].FindMapValue(TEXT("name"))->GetString();
	// This fixture borrows the editor world to participate in the real render loop.
	// Exclude unrelated editor primitives so clear pixels provide a controlled no-hit
	// region for the depth-far sentinel while the installed source remains visible.
	URenderCameraComponent* InstalledCamera = nullptr;
	for (TObjectIterator<URenderCameraComponent> It; It; ++It)
	{
		if (It->GetWorld() == State->Fixture->World && It->GetCanonicalName() == CameraName)
		{
			InstalledCamera = *It;
			break;
		}
	}
	if (!TestNotNull(TEXT("installed GPU camera is discoverable"), InstalledCamera)) return false;
	TArray<UPrimitiveComponent*> HiddenEditorPrimitives;
	for (TObjectIterator<UPrimitiveComponent> It; It; ++It)
	{
		if (It->GetWorld() != State->Fixture->World || It->IsA<USceneCaptureComponent2D>()) continue;
		bool bInstalledGeom = false;
		for (const FName& Tag : It->ComponentTags)
		{
			if (Tag.ToString().StartsWith(TEXT("URLabRenderGeom=")))
			{
				bInstalledGeom = true;
				break;
			}
		}
		if (!bInstalledGeom) HiddenEditorPrimitives.Add(*It);
	}
	InstalledCamera->SetVisibilityComponents({}, HiddenEditorPrimitives, -1);
	// Visibility is baseline camera configuration, so describe it after applying
	// the fixture isolation and use that revision for restoration assertions.
	if (!Send(*State->Fixture->Transport, State->Session, Id++,
		TEXT("render_scene_describe"), Proof(State->Generation, State->Token,
			{{TEXT("expected_revision"), V::MakeString(Revision)}}), ReplyBytes, Reply)
		|| Reply.Kind != EURLabReplyKind::Result) return false;
	Cameras = Reply.Result.FindMapValue(TEXT("model_cameras"));
	if (!Cameras || Cameras->GetArray().IsEmpty()) return false;
	const V* BaselineOptics = Cameras->GetArray()[0].FindMapValue(TEXT("optics"));
	const V* BaselineResolution = BaselineOptics
		? BaselineOptics->FindMapValue(TEXT("resolution")) : nullptr;
	const V* BaselineConfig = Cameras->GetArray()[0].FindMapValue(
		TEXT("baseline_configuration_revision"));
	if (!BaselineResolution || !BaselineConfig) return false;
	State->SceneRevision = Revision;
	State->CameraName = CameraName;
	State->BaselineWidth = BaselineResolution->FindMapValue(TEXT("width"))->GetInt();
	State->BaselineHeight = BaselineResolution->FindMapValue(TEXT("height"))->GetInt();
	State->BaselineConfigurationRevision = BaselineConfig->GetInt();
	const V CameraSelection = Map({{TEXT("name"), V::MakeString(CameraName)},
		{TEXT("override"), Map({{TEXT("mode"), V::MakeString(TEXT("real"))},
			{TEXT("resolution"), Map({{TEXT("width"), V::MakeInt(96)},
				{TEXT("height"), V::MakeInt(64)}})}})}});
	const V* Bodies = Reply.Result.FindMapValue(TEXT("body_names"));
	if (!Bodies) return false;
	const int32 BodyCount = Bodies->GetArray().Num();
	const bool bHasUserCamera = Reply.Result.FindMapValue(TEXT("user_camera")) != nullptr;
	const V Frame = Map({{TEXT("frame_id"), V::MakeInt(1)},
		{TEXT("clock_epoch"), V::MakeInt(1)}, {TEXT("has_source_time"), V::MakeBool(true)},
		{TEXT("source_time"), V::MakeFloat(1.0)}});
	if (!Send(*State->Fixture->Transport, State->Session, Id++,
		TEXT("render_capture_submit"), Proof(State->Generation, State->Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)},
			{TEXT("cameras"), V::MakeArray({CameraSelection})},
			{TEXT("timeout_s"), V::MakeFloat(20.0)}, {TEXT("frame"), Frame},
			{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("qpos"))}})}}),
		ReplyBytes, Reply) || Reply.Kind != EURLabReplyKind::Result) return false;
	State->Jobs.Add(Reply.Result.FindMapValue(TEXT("job_id"))->GetString());
	State->BaselineGeometryConfigurationId = Reply.Result.FindMapValue(
		TEXT("geometry_configuration_id"))->GetString();
	if (!TestFalse(TEXT("baseline geometry identity is nonempty"),
		State->BaselineGeometryConfigurationId.IsEmpty())) return false;
	TArray<double> Cartesian;
	auto AddPose = [&Cartesian](double X, double Y, double Z)
	{
		Cartesian.Add(X); Cartesian.Add(Y); Cartesian.Add(Z);
		Cartesian.Add(1.0); Cartesian.Add(0.0); Cartesian.Add(0.0); Cartesian.Add(0.0);
	};
	for (int32 Index = 0; Index < BodyCount; ++Index)
		AddPose(Index == 2 ? 0.55 : 0.0, 0.0, 0.0);
	// The wire camera remains in source RH optical coordinates. The shared
	// decoder applies the -Z-forward/+Y-up optical correction.
	AddPose(0.0, 0.0, 1.5);
	if (bHasUserCamera)
		AddPose(0.0, 0.0, 0.0);
	FString CartesianBlob, CartesianSha;
	if (!Upload(*State->Fixture->Transport, State->Session, Id, Doubles(Cartesian),
		CartesianBlob, CartesianSha)) return false;
	const V CartesianFrame = Map({{TEXT("frame_id"), V::MakeInt(2)},
		{TEXT("clock_epoch"), V::MakeInt(1)}, {TEXT("has_source_time"), V::MakeBool(false)}});
	if (!Send(*State->Fixture->Transport, State->Session, Id++,
		TEXT("render_capture_submit"), Proof(State->Generation, State->Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)},
			{TEXT("cameras"), V::MakeArray({CameraSelection})},
			{TEXT("timeout_s"), V::MakeFloat(20.0)}, {TEXT("frame"), CartesianFrame},
			{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("cartesian"))},
				{TEXT("payload"), Map({{TEXT("blob_id"), V::MakeString(CartesianBlob)},
					{TEXT("sha256"), V::MakeString(CartesianSha)}})}})}}),
		ReplyBytes, Reply) || Reply.Kind != EURLabReplyKind::Result) return false;
	State->Jobs.Add(Reply.Result.FindMapValue(TEXT("job_id"))->GetString());
	if (!TestEqual(TEXT("equivalent Cartesian capture uses baseline geometry identity"),
		Reply.Result.FindMapValue(TEXT("geometry_configuration_id"))->GetString(),
		State->BaselineGeometryConfigurationId)) return false;
	const V DepthResolution = Map({{TEXT("width"), V::MakeInt(96)},
		{TEXT("height"), V::MakeInt(64)}});
	const V DepthOverride = Map({{TEXT("mode"), V::MakeString(TEXT("depth"))},
		{TEXT("resolution"), DepthResolution},
		{TEXT("depth_near_m"), V::MakeFloat(0.02)},
		{TEXT("depth_far_m"), V::MakeFloat(2.0)}});
	const V DepthSelection = Map({{TEXT("name"), V::MakeString(CameraName)},
		{TEXT("override"), DepthOverride}});
	if (!TestEqual(TEXT("depth fixture builds a map selection"), DepthSelection.GetType(),
		EURLabWireValueType::Map)
		|| !TestEqual(TEXT("depth fixture builds a map override"), DepthOverride.GetType(),
			EURLabWireValueType::Map)
		|| !TestEqual(TEXT("depth fixture builds a map resolution"), DepthResolution.GetType(),
			EURLabWireValueType::Map)) return false;
	const V DepthFrame = Map({{TEXT("frame_id"), V::MakeInt(3)},
		{TEXT("clock_epoch"), V::MakeInt(1)}, {TEXT("has_source_time"), V::MakeBool(false)}});
	if (!Send(*State->Fixture->Transport, State->Session, Id++,
		TEXT("render_capture_submit"), Proof(State->Generation, State->Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)},
			{TEXT("cameras"), V::MakeArray({DepthSelection})},
			{TEXT("timeout_s"), V::MakeFloat(20.0)}, {TEXT("frame"), DepthFrame},
			{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("qpos"))}})}}),
		ReplyBytes, Reply) || Reply.Kind != EURLabReplyKind::Result) return false;
	State->Jobs.Add(Reply.Result.FindMapValue(TEXT("job_id"))->GetString());
	for (const TCHAR* Mode : {TEXT("instance"), TEXT("semantic")})
	{
		const V SegmentationSelection = Map({{TEXT("name"), V::MakeString(CameraName)},
			{TEXT("override"), Map({{TEXT("mode"), V::MakeString(Mode)},
				{TEXT("resolution"), Map({{TEXT("width"), V::MakeInt(96)},
					{TEXT("height"), V::MakeInt(64)}})}})}});
		const V SegmentationFrame = Map({{TEXT("frame_id"), V::MakeInt(State->Jobs.Num() + 1)},
			{TEXT("clock_epoch"), V::MakeInt(1)}, {TEXT("has_source_time"), V::MakeBool(false)}});
		if (!Send(*State->Fixture->Transport, State->Session, Id++,
			TEXT("render_capture_submit"), Proof(State->Generation, State->Token, {
				{TEXT("expected_revision"), V::MakeString(Revision)},
				{TEXT("cameras"), V::MakeArray({SegmentationSelection})},
				{TEXT("timeout_s"), V::MakeFloat(20.0)}, {TEXT("frame"), SegmentationFrame},
				{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("qpos"))}})}}),
			ReplyBytes, Reply) || Reply.Kind != EURLabReplyKind::Result) return false;
		State->Jobs.Add(Reply.Result.FindMapValue(TEXT("job_id"))->GetString());
	}
	const V GeomOverride = Map({{TEXT("geom_index"), V::MakeInt(0)},
		{TEXT("position_m"), V::MakeArray({V::MakeFloat(-0.55), V::MakeFloat(0.0),
			V::MakeFloat(0.0)})},
		{TEXT("quaternion_wxyz"), V::MakeArray({V::MakeFloat(1.0), V::MakeFloat(0.0),
			V::MakeFloat(0.0), V::MakeFloat(0.0)})}});
	const V OverrideFrame = Map({{TEXT("frame_id"), V::MakeInt(5)},
		{TEXT("clock_epoch"), V::MakeInt(1)}, {TEXT("has_source_time"), V::MakeBool(true)},
		{TEXT("source_time"), V::MakeFloat(5.0)}});
	if (!Send(*State->Fixture->Transport, State->Session, Id++,
		TEXT("render_capture_submit"), Proof(State->Generation, State->Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)},
			{TEXT("cameras"), V::MakeArray({CameraSelection})},
			{TEXT("geom_overrides"), V::MakeArray({GeomOverride})},
			{TEXT("timeout_s"), V::MakeFloat(20.0)}, {TEXT("frame"), OverrideFrame},
			{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("qpos"))}})}}),
		ReplyBytes, Reply) || Reply.Kind != EURLabReplyKind::Result) return false;
	State->Jobs.Add(Reply.Result.FindMapValue(TEXT("job_id"))->GetString());
	State->OverrideGeometryConfigurationId = Reply.Result.FindMapValue(
		TEXT("geometry_configuration_id"))->GetString();
	if (!TestNotEqual(TEXT("override submission has distinct geometry identity"),
		State->OverrideGeometryConfigurationId, State->BaselineGeometryConfigurationId))
		return false;
	const V RestoredFrame = Map({{TEXT("frame_id"), V::MakeInt(6)},
		{TEXT("clock_epoch"), V::MakeInt(1)}, {TEXT("has_source_time"), V::MakeBool(true)},
		{TEXT("source_time"), V::MakeFloat(6.0)}});
	if (!Send(*State->Fixture->Transport, State->Session, Id++,
		TEXT("render_capture_submit"), Proof(State->Generation, State->Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)},
			{TEXT("cameras"), V::MakeArray({CameraSelection})},
			{TEXT("timeout_s"), V::MakeFloat(20.0)}, {TEXT("frame"), RestoredFrame},
			{TEXT("pose"), Map({{TEXT("kind"), V::MakeString(TEXT("qpos"))}})}}),
		ReplyBytes, Reply) || Reply.Kind != EURLabReplyKind::Result) return false;
	State->Jobs.Add(Reply.Result.FindMapValue(TEXT("job_id"))->GetString());
	if (!TestEqual(TEXT("queued baseline submission retains baseline geometry identity"),
		Reply.Result.FindMapValue(TEXT("geometry_configuration_id"))->GetString(),
		State->BaselineGeometryConfigurationId)) return false;
	State->NextId = Id;
	State->Deadline = FPlatformTime::Seconds() + 25.0;
	State->RenewAt = FPlatformTime::Seconds() + 10.0;
	ADD_LATENT_AUTOMATION_COMMAND(FURLabPumpTypedRenderCaptureGpu(State));
	return true;
}
#endif
