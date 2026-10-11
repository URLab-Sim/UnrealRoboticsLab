// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Misc/AutomationTest.h"

#include "Async/Async.h"
#include "Async/TaskGraphInterfaces.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "Transport/ZmqRpcTransport.h"
#include "Wire/RequestEnvelope.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
using V = FURLabWireValue;

constexpr uint32 IntegrationStateMask = 0x3fff;

const char* TypedControlXml = R"(
<mujoco model="typed_control_dispatch">
  <option timestep="0.004" gravity="0 0 0"/>
  <worldbody>
    <body name="link">
      <joint name="hinge" type="hinge" damping="0.05"/>
      <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.04" mass="1"/>
    </body>
  </worldbody>
  <actuator><motor name="hinge_motor" joint="hinge" gear="1"/></actuator>
</mujoco>)";

TArray<uint8> TypedControlBytesOf(const char* Text)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), FCStringAnsi::Strlen(Text));
	return Out;
}

V Map(std::initializer_list<TPair<FString, V>> Fields)
{
	TArray<FString> Keys;
	TArray<V> Values;
	for (const TPair<FString, V>& Field : Fields)
	{
		Keys.Add(Field.Key);
		Values.Add(Field.Value);
	}
	return V::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

bool Connect(UURLabZmqRpcTransport& Transport, FURLabConnectReply& Out)
{
	TArray<uint8> RequestBytes;
	TArray<uint8> ReplyBytes;
	return FURLabRequestEnvelopeCodec::EncodeConnectRequest({2}, RequestBytes).IsOk()
		&& Transport.ProcessRequestBytes(RequestBytes, ReplyBytes)
		&& FURLabRequestEnvelopeCodec::DecodeConnectReply(ReplyBytes, Out).IsOk();
}

bool Disconnect(UURLabZmqRpcTransport& Transport, const FURLabConnectReply& Connection)
{
	TArray<uint8> RequestBytes;
	TArray<uint8> ReplyBytes;
	FURLabDisconnectReply Reply;
	return FURLabRequestEnvelopeCodec::EncodeDisconnectRequest(
		{2, Connection.BootId, Connection.SessionId}, RequestBytes).IsOk()
		&& Transport.ProcessRequestBytes(RequestBytes, ReplyBytes)
		&& FURLabRequestEnvelopeCodec::DecodeDisconnectReply(ReplyBytes, Reply).IsOk()
		&& Reply.bDisconnected;
}

bool EncodeRequest(const FURLabConnectReply& Connection, uint64 RequestId,
	const FString& Op, V Body, TArray<uint8>& OutBytes)
{
	FURLabRequestEnvelope Request;
	Request.ProtocolVersion = 2;
	Request.BootId = Connection.BootId;
	Request.SessionId = Connection.SessionId;
	Request.RequestId = RequestId;
	Request.Op = Op;
	Request.Body = MoveTemp(Body);
	return FURLabRequestEnvelopeCodec::Encode(Request, OutBytes).IsOk();
}

bool Send(UURLabZmqRpcTransport& Transport, const FURLabConnectReply& Connection,
	uint64 RequestId, const FString& Op, V Body, TArray<uint8>& OutReplyBytes,
	FURLabReplyEnvelope& OutReply)
{
	TArray<uint8> RequestBytes;
	return EncodeRequest(Connection, RequestId, Op, MoveTemp(Body), RequestBytes)
		&& Transport.ProcessRequestBytes(RequestBytes, OutReplyBytes)
		&& FURLabRequestEnvelopeCodec::DecodeReply(OutReplyBytes, OutReply).IsOk();
}

V ClaimBody(uint64 Generation)
{
	return Map({
		{TEXT("model_generation"), V::MakeInt(static_cast<int64>(Generation))},
		{TEXT("ttl_s"), V::MakeFloat(30.0)}});
}

V ControlBody(uint64 Generation, const FString& Token,
	std::initializer_list<TPair<FString, V>> Extra = {})
{
	TArray<FString> Keys({TEXT("model_generation"), TEXT("control_token")});
	TArray<V> Values({
		V::MakeInt(static_cast<int64>(Generation)), V::MakeString(Token)});
	for (const TPair<FString, V>& Field : Extra)
	{
		Keys.Add(Field.Key);
		Values.Add(Field.Value);
	}
	return V::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

bool ReadClaim(const FURLabReplyEnvelope& Reply, FString& OutToken, uint64& OutGeneration)
{
	if (Reply.Kind != EURLabReplyKind::Result) return false;
	const V* Control = Reply.Result.FindMapValue(TEXT("control"));
	const V* Token = Control ? Control->FindMapValue(TEXT("token")) : nullptr;
	const V* Generation = Control ? Control->FindMapValue(TEXT("model_generation")) : nullptr;
	if (!Token || !Generation || Token->GetString().IsEmpty() || Generation->GetInt() <= 0)
		return false;
	OutToken = Token->GetString();
	OutGeneration = static_cast<uint64>(Generation->GetInt());
	return true;
}

bool SameObservation(const FMjModelObservation& A, const FMjModelObservation& B)
{
	return A.Identity.Matches(B.Identity)
		&& A.ClockEpoch == B.ClockEpoch
		&& A.StepSequence == B.StepSequence
		&& A.Time == B.Time
		&& A.Qpos == B.Qpos
		&& A.Qvel == B.Qvel
		&& A.Ctrl == B.Ctrl
		&& A.SensorData == B.SensorData;
}

V TamperedCapturedState(const V& Captured)
{
	const V& Identity = *Captured.FindMapValue(TEXT("model"));
	V TamperedIdentity = Map({
		{TEXT("content_sha256"), V::MakeString(
			Identity.FindMapValue(TEXT("content_sha256"))->GetString() + TEXT("00"))},
		{TEXT("generation"), *Identity.FindMapValue(TEXT("generation"))},
		{TEXT("mujoco_version"), *Identity.FindMapValue(TEXT("mujoco_version"))},
		{TEXT("nq"), *Identity.FindMapValue(TEXT("nq"))},
		{TEXT("nv"), *Identity.FindMapValue(TEXT("nv"))},
		{TEXT("nu"), *Identity.FindMapValue(TEXT("nu"))},
		{TEXT("na"), *Identity.FindMapValue(TEXT("na"))},
		{TEXT("nsensordata"), *Identity.FindMapValue(TEXT("nsensordata"))}});
	return Map({
		{TEXT("model"), MoveTemp(TamperedIdentity)},
		{TEXT("clock_epoch"), *Captured.FindMapValue(TEXT("clock_epoch"))},
		{TEXT("mask"), *Captured.FindMapValue(TEXT("mask"))},
		{TEXT("scalar"), *Captured.FindMapValue(TEXT("scalar"))},
		{TEXT("complete_checkpoint"), *Captured.FindMapValue(TEXT("complete_checkpoint"))},
		{TEXT("state_size"), *Captured.FindMapValue(TEXT("state_size"))},
		{TEXT("state"), *Captured.FindMapValue(TEXT("state"))}});
}

struct FTypedControlFixture
{
	UURLabBridgeServer* Server = nullptr;
	UURLabZmqRpcTransport* Transport = nullptr;
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime;
	FURLabRpcDispatcher::FSimulationRuntimeRegistration Registration;
	uint64 Generation = 0;

	bool Init(bool bAcceptInput, FString& OutError)
	{
		Server = NewObject<UURLabBridgeServer>();
		Server->Start(TEXT(""));
		FMjCapabilitySet Grants;
		Grants.Set(EMjCapability::AcceptInput, bAcceptInput);
		Server->SetGrants(Grants);

		Runtime = MakeShared<FMjSimulationRuntime, ESPMode::ThreadSafe>();
		FMjOwnedModelCandidate Candidate;
		if (!FMjOwnedModelCandidate::FromBytes(
			EMjModelSourceKind::XmlWithAssets, TypedControlBytesOf(TypedControlXml), {},
			Candidate, OutError, TEXT("typed control dispatch fixture")))
		{
			return false;
		}
		const FMjSimulationInstallResult Installed = Runtime->Install(MoveTemp(Candidate));
		if (!Installed.bCommitted)
		{
			OutError = Installed.Error;
			return false;
		}
		Generation = Installed.Generation;
		if (!Server->GetDispatcher()->AttachSimulationRuntime(
			Runtime.ToSharedRef(), Registration, OutError))
		{
			return false;
		}
		Transport = NewObject<UURLabZmqRpcTransport>(Server);
		Transport->SetOwningBridge(Server);
		return true;
	}

	void SetAcceptInput(bool bAcceptInput)
	{
		FMjCapabilitySet Grants;
		Grants.Set(EMjCapability::AcceptInput, bAcceptInput);
		Server->SetGrants(Grants);
	}

	~FTypedControlFixture()
	{
		if (Server) Server->Stop();
	}
};
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedControlPermissionAndSessionTest,
	"URLab.TypedControlDispatch.PermissionRetentionAndSessionAuthority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedControlPermissionAndSessionTest::RunTest(const FString&)
{
	FTypedControlFixture Fixture;
	FString Error;
	if (!TestTrue(TEXT("independent control runtime installs"), Fixture.Init(true, Error)))
	{
		AddError(Error);
		return false;
	}
	FURLabConnectReply SessionA;
	FURLabConnectReply SessionB;
	if (!TestTrue(TEXT("first ZMQ session connects"), Connect(*Fixture.Transport, SessionA))
		|| !TestTrue(TEXT("second ZMQ session connects"), Connect(*Fixture.Transport, SessionB)))
	{
		return false;
	}
	TestNotEqual(TEXT("sessions have distinct identities"), SessionA.SessionId, SessionB.SessionId);

	TArray<uint8> DeniedRequest;
	if (!TestTrue(TEXT("encode denied claim"), EncodeRequest(SessionA, 1,
		TEXT("simulation_claim_control"), ClaimBody(Fixture.Generation), DeniedRequest)))
	{
		return false;
	}
	TFuture<TArray<uint8>> WorkerAttempt = Async(EAsyncExecution::ThreadPool,
		[Transport = Fixture.Transport, DeniedRequest]()
		{
			TArray<uint8> ReplyBytes;
			Transport->ProcessRequestBytes(DeniedRequest, ReplyBytes);
			return ReplyBytes;
		});
	FURLabReplyEnvelope Pending;
	const TArray<uint8> PendingBytes = WorkerAttempt.Get();
	if (!TestTrue(TEXT("worker claim pending decodes"),
		FURLabRequestEnvelopeCodec::DecodeReply(PendingBytes, Pending).IsOk())
		|| !TestEqual(TEXT("claim admitted while input granted"),
			Pending.Kind, EURLabReplyKind::Pending))
	{
		return false;
	}
	Fixture.SetAcceptInput(false);
	FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
	TArray<uint8> DeniedReplyBytes;
	FURLabReplyEnvelope DeniedReply;
	if (!TestTrue(TEXT("retrieve execution-time denial through ZMQ bytes"),
		Fixture.Transport->ProcessRequestBytes(DeniedRequest, DeniedReplyBytes))
		|| !TestTrue(TEXT("denied claim decodes"),
			FURLabRequestEnvelopeCodec::DecodeReply(DeniedReplyBytes, DeniedReply).IsOk()))
	{
		return false;
	}
	TestEqual(TEXT("execution-time permission revocation denies admitted claim"),
		DeniedReply.Error.Code, FString(TEXT("capability_disabled")));
	Fixture.SetAcceptInput(true);
	TArray<uint8> RetainedDenied;
	TestTrue(TEXT("duplicate denied claim remains queryable"),
		Fixture.Transport->ProcessRequestBytes(DeniedRequest, RetainedDenied));
	TestEqual(TEXT("grant change cannot rewrite retained denial"), RetainedDenied, DeniedReplyBytes);

	TArray<uint8> ClaimReplyBytes;
	FURLabReplyEnvelope ClaimReply;
	if (!TestTrue(TEXT("fresh claim succeeds after grant"), Send(*Fixture.Transport, SessionA, 2,
		TEXT("simulation_claim_control"), ClaimBody(Fixture.Generation), ClaimReplyBytes, ClaimReply)))
	{
		return false;
	}
	FString Token;
	uint64 ClaimedGeneration = 0;
	if (!TestTrue(TEXT("fresh claim returns opaque authority"),
		ReadClaim(ClaimReply, Token, ClaimedGeneration)))
	{
		return false;
	}
	TestEqual(TEXT("claim binds installed generation"), ClaimedGeneration, Fixture.Generation);

	FMjModelObservation BeforeForeign;
	Fixture.Runtime->GetObservation(BeforeForeign);
	TArray<V> ForeignControls;
	ForeignControls.Add(V::MakeFloat(0.5));
	TArray<uint8> ForeignReplyBytes;
	FURLabReplyEnvelope ForeignReply;
	if (!TestTrue(TEXT("cross-session token reaches typed handler"), Send(*Fixture.Transport,
		SessionB, 1, TEXT("simulation_set_control"),
		ControlBody(Fixture.Generation, Token,
			{{TEXT("ctrl"), V::MakeArray(MoveTemp(ForeignControls))}}),
		ForeignReplyBytes, ForeignReply)))
	{
		return false;
	}
	TestEqual(TEXT("another session cannot use the token"),
		ForeignReply.Error.Code, FString(TEXT("not_control_owner")));
	FMjModelObservation AfterForeign;
	Fixture.Runtime->GetObservation(AfterForeign);
	TestTrue(TEXT("cross-session rejection preserves runtime"),
		SameObservation(BeforeForeign, AfterForeign));
	TestTrue(TEXT("original authority remains claimed"),
		Fixture.Runtime->GetControlStatus().bClaimed);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedControlMutationRetentionTest,
	"URLab.TypedControlDispatch.RetainedStepAndNestedState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedControlMutationRetentionTest::RunTest(const FString&)
{
	FTypedControlFixture Fixture;
	FString Error;
	if (!TestTrue(TEXT("independent control runtime installs"), Fixture.Init(true, Error)))
	{
		AddError(Error);
		return false;
	}
	FURLabConnectReply Session;
	if (!TestTrue(TEXT("ZMQ session connects"), Connect(*Fixture.Transport, Session))) return false;
	TArray<uint8> ReplyBytes;
	FURLabReplyEnvelope Reply;
	if (!TestTrue(TEXT("control claim"), Send(*Fixture.Transport, Session, 1,
		TEXT("simulation_claim_control"), ClaimBody(Fixture.Generation), ReplyBytes, Reply)))
	{
		return false;
	}
	FString Token;
	uint64 ClaimedGeneration = 0;
	if (!TestTrue(TEXT("claim result"), ReadClaim(Reply, Token, ClaimedGeneration))) return false;

	TArray<uint8> StepRequest;
	if (!TestTrue(TEXT("encode exact step ticket"), EncodeRequest(Session, 2,
		TEXT("simulation_step"), ControlBody(ClaimedGeneration, Token,
			{{TEXT("count"), V::MakeInt(3)}}), StepRequest)))
	{
		return false;
	}
	TArray<uint8> FirstStep;
	TArray<uint8> DuplicateStep;
	FURLabReplyEnvelope FirstStepReply;
	if (!TestTrue(TEXT("first step dispatch"),
		Fixture.Transport->ProcessRequestBytes(StepRequest, FirstStep))
		|| !TestTrue(TEXT("first step reply decodes"),
			FURLabRequestEnvelopeCodec::DecodeReply(FirstStep, FirstStepReply).IsOk())
		|| !TestEqual(TEXT("first step returns a result"),
			FirstStepReply.Kind, EURLabReplyKind::Result))
	{
		return false;
	}
	TestEqual(TEXT("step executes requested count once"),
		Fixture.Runtime->GetStepSequence(), static_cast<uint64>(3));
	TestTrue(TEXT("duplicate step dispatch"),
		Fixture.Transport->ProcessRequestBytes(StepRequest, DuplicateStep));
	TestEqual(TEXT("duplicate step returns byte-exact retained reply"), DuplicateStep, FirstStep);
	TestEqual(TEXT("duplicate step does not execute again"),
		Fixture.Runtime->GetStepSequence(), static_cast<uint64>(3));

	TArray<uint8> CapturedBytes;
	FURLabReplyEnvelope Captured;
	if (!TestTrue(TEXT("capture integration state"), Send(*Fixture.Transport, Session, 3,
		TEXT("simulation_capture_state"),
		Map({{TEXT("mask"), V::MakeInt(IntegrationStateMask)}}), CapturedBytes, Captured))
		|| !TestEqual(TEXT("capture returns nested-state source"),
			Captured.Kind, EURLabReplyKind::Result))
	{
		return false;
	}
	if (!TestTrue(TEXT("mutate after capture"), Send(*Fixture.Transport, Session, 4,
		TEXT("simulation_step"), ControlBody(ClaimedGeneration, Token,
			{{TEXT("count"), V::MakeInt(2)}}), ReplyBytes, Reply)))
	{
		return false;
	}
	TestEqual(TEXT("post-capture mutation advances"),
		Fixture.Runtime->GetStepSequence(), static_cast<uint64>(5));

	const uint64 EpochBeforeApply = Fixture.Runtime->GetClockEpoch();
	if (!TestTrue(TEXT("apply exact captured-state map"), Send(*Fixture.Transport, Session, 5,
		TEXT("simulation_apply_state"), ControlBody(ClaimedGeneration, Token,
			{{TEXT("state"), Captured.Result}}), ReplyBytes, Reply)))
	{
		return false;
	}
	TestEqual(TEXT("nested apply succeeds"), Reply.Kind, EURLabReplyKind::Result);
	TestEqual(TEXT("nested apply restores captured sequence"),
		Fixture.Runtime->GetStepSequence(), static_cast<uint64>(0));
	TestTrue(TEXT("nested apply advances epoch"), Fixture.Runtime->GetClockEpoch() > EpochBeforeApply);

	FMjModelObservation BeforeTamper;
	Fixture.Runtime->GetObservation(BeforeTamper);
	V Tampered = TamperedCapturedState(Captured.Result);
	if (!TestTrue(TEXT("tampered nested identity reaches domain validation"),
		Send(*Fixture.Transport, Session, 6, TEXT("simulation_apply_state"),
			ControlBody(ClaimedGeneration, Token, {{TEXT("state"), MoveTemp(Tampered)}}),
			ReplyBytes, Reply)))
	{
		return false;
	}
	TestEqual(TEXT("tampered content hash is invalid state"),
		Reply.Error.Code, FString(TEXT("invalid_state")));
	FMjModelObservation AfterTamper;
	Fixture.Runtime->GetObservation(AfterTamper);
	TestTrue(TEXT("tampered state rejection is atomic"),
		SameObservation(BeforeTamper, AfterTamper));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedControlQueuedLifetimeTest,
	"URLab.TypedControlDispatch.QueuedDisconnectAndShutdown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedControlQueuedLifetimeTest::RunTest(const FString&)
{
	{
		FTypedControlFixture Fixture;
		FString Error;
		if (!TestTrue(TEXT("disconnect fixture installs"), Fixture.Init(true, Error)))
		{
			AddError(Error);
			return false;
		}
		FURLabConnectReply Session;
		if (!TestTrue(TEXT("disconnect fixture connects"), Connect(*Fixture.Transport, Session)))
			return false;
		TArray<uint8> ClaimRequest;
		if (!TestTrue(TEXT("encode disconnect claim"), EncodeRequest(Session, 1,
			TEXT("simulation_claim_control"), ClaimBody(Fixture.Generation), ClaimRequest)))
		{
			return false;
		}
		TFuture<TArray<uint8>> WorkerAttempt = Async(EAsyncExecution::ThreadPool,
			[Transport = Fixture.Transport, ClaimRequest]()
			{
				TArray<uint8> ReplyBytes;
				Transport->ProcessRequestBytes(ClaimRequest, ReplyBytes);
				return ReplyBytes;
			});
		const TArray<uint8> PendingBytes = WorkerAttempt.Get();
		FURLabReplyEnvelope Pending;
		if (!TestTrue(TEXT("worker claim pending decodes"),
			FURLabRequestEnvelopeCodec::DecodeReply(PendingBytes, Pending).IsOk())
			|| !TestEqual(TEXT("worker claim is pending"),
				Pending.Kind, EURLabReplyKind::Pending))
		{
			return false;
		}
		TestTrue(TEXT("disconnect before game-thread drain"), Disconnect(*Fixture.Transport, Session));
		FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
		TestFalse(TEXT("queued claim cannot revive disconnected session"),
			Fixture.Runtime->GetControlStatus().bClaimed);
		TestEqual(TEXT("disconnected pending work is released"),
			Fixture.Server->GetDispatcher()->GetRequestLedgerForTest()->GetStats().Pending, 0);
	}

	{
		FTypedControlFixture Fixture;
		FString Error;
		if (!TestTrue(TEXT("shutdown fixture installs"), Fixture.Init(true, Error)))
		{
			AddError(Error);
			return false;
		}
		FURLabConnectReply Session;
		if (!TestTrue(TEXT("shutdown fixture connects"), Connect(*Fixture.Transport, Session)))
			return false;
		const TSharedRef<FURLabRequestLedger, ESPMode::ThreadSafe> Ledger =
			Fixture.Server->GetDispatcher()->GetRequestLedgerForTest();
		TArray<uint8> ClaimRequest;
		if (!TestTrue(TEXT("encode shutdown claim"), EncodeRequest(Session, 1,
			TEXT("simulation_claim_control"), ClaimBody(Fixture.Generation), ClaimRequest)))
		{
			return false;
		}
		TFuture<TArray<uint8>> WorkerAttempt = Async(EAsyncExecution::ThreadPool,
			[Transport = Fixture.Transport, ClaimRequest]()
			{
				TArray<uint8> ReplyBytes;
				Transport->ProcessRequestBytes(ClaimRequest, ReplyBytes);
				return ReplyBytes;
			});
		FURLabReplyEnvelope Pending;
		const TArray<uint8> PendingBytes = WorkerAttempt.Get();
		if (!TestTrue(TEXT("shutdown worker claim pending decodes"),
			FURLabRequestEnvelopeCodec::DecodeReply(PendingBytes, Pending).IsOk())
			|| !TestEqual(TEXT("shutdown worker claim is pending"),
				Pending.Kind, EURLabReplyKind::Pending))
		{
			return false;
		}
		TestEqual(TEXT("claim is accounted before shutdown"), Ledger->GetStats().Pending, 1);
		Fixture.Server->Stop();
		TestNull(TEXT("shutdown destroys dispatcher before scheduled drain"),
			Fixture.Server->GetDispatcher());
		TestEqual(TEXT("shutdown cancellation releases pending ledger credit"),
			Ledger->GetStats().Pending, 0);
		TestFalse(TEXT("cancelled queued claim never acquires lease"),
			Fixture.Runtime->GetControlStatus().bClaimed);
		FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
		TestEqual(TEXT("late queue callback remains safely empty"), Ledger->GetStats().Pending, 0);
		TestFalse(TEXT("late queue callback cannot acquire a lease"),
			Fixture.Runtime->GetControlStatus().bClaimed);
	}
	return !HasAnyErrors();
}

#endif
