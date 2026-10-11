// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfig.h"
#include "Bridge/RetainedCompletion.h"
#include "Bridge/CameraStreamServiceBinding.h"
#include "Bridge/RpcErrorCodes.h"
#include "MuJoCo/Core/MjModelDescriptor.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "MuJoCo/Runtime/MjSimulationInteractionOwner.h"
#include "Bridge/InteractionWire.h"
#include "Wire/OperationSchema.h"
#include "Wire/UrlabWire.gen.h"
#include "Async/Async.h"

namespace
{
static_assert(URLabWire::Limits::RequestWindow == FURLabRequestLedger::WindowWidth);
static_assert(URLabWire::Limits::MaxSessionPending == FURLabRequestLedger::MaxPendingPerSession);
static_assert(URLabWire::Limits::MaxGlobalPending == FURLabRequestLedger::MaxPendingGlobal);
static_assert(URLabWire::Limits::MaxInlineReplyBytes == FURLabRequestLedger::MaxInlineReplyBytes);
using V = FURLabWireValue;

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

V Doubles(const TArray<double>& Input)
{
	TArray<V> Values;
	Values.Reserve(Input.Num());
	for (double Value : Input) Values.Add(V::MakeFloat(Value));
	return V::MakeArray(MoveTemp(Values));
}

bool Signed(uint64 Value, int64& Out)
{
	if (Value > static_cast<uint64>(MAX_int64)) return false;
	Out = static_cast<int64>(Value);
	return true;
}

V Identity(const FMjModelIdentity& I)
{
	return Map({
		{TEXT("content_sha256"), V::MakeString(I.ContentSha256)},
		{TEXT("generation"), V::MakeInt(static_cast<int64>(I.Generation))},
		{TEXT("mujoco_version"), V::MakeInt(I.MuJoCoVersion)},
		{TEXT("nq"), V::MakeInt(I.Nq)}, {TEXT("nv"), V::MakeInt(I.Nv)},
		{TEXT("nu"), V::MakeInt(I.Nu)}, {TEXT("na"), V::MakeInt(I.Na)},
		{TEXT("nsensordata"), V::MakeInt(I.NSensorData)}});
}

template <typename T, typename Fn>
V ArrayOf(const TArray<T>& Items, Fn&& Convert)
{
	TArray<V> Values;
	Values.Reserve(Items.Num());
	for (const T& Item : Items) Values.Add(Convert(Item));
	return V::MakeArray(MoveTemp(Values));
}

V Describe(const FMjModelDescriptor& D)
{
	auto Named = [](const FMjNamedObjectDescriptor& X) { return Map({
		{TEXT("id"), V::MakeInt(X.Id)}, {TEXT("name"), V::MakeString(X.Name)}}); };
	return Map({
		{TEXT("model"), Identity(D.Identity)},
		{TEXT("joints"), ArrayOf(D.Joints, [](const FMjJointDescriptor& X) { return Map({
			{TEXT("id"), V::MakeInt(X.Id)}, {TEXT("name"), V::MakeString(X.Name)},
			{TEXT("type"), V::MakeInt(X.Type)}, {TEXT("qpos_address"), V::MakeInt(X.QposAddress)},
			{TEXT("qpos_width"), V::MakeInt(X.QposWidth)}, {TEXT("dof_address"), V::MakeInt(X.DofAddress)},
			{TEXT("dof_width"), V::MakeInt(X.DofWidth)}, {TEXT("limited"), V::MakeBool(X.bLimited)},
			{TEXT("range_min"), V::MakeFloat(X.RangeMin)}, {TEXT("range_max"), V::MakeFloat(X.RangeMax)}}); })},
		{TEXT("actuators"), ArrayOf(D.Actuators, [](const FMjActuatorDescriptor& X) { return Map({
			{TEXT("id"), V::MakeInt(X.Id)}, {TEXT("name"), V::MakeString(X.Name)},
			{TEXT("control_address"), V::MakeInt(X.ControlAddress)},
			{TEXT("control_width"), V::MakeInt(X.ControlWidth)},
			{TEXT("controls"), ArrayOf(X.Controls, [](const FMjActuatorControlDescriptor& C) { return Map({
				{TEXT("limited"), V::MakeBool(C.bLimited)}, {TEXT("range_min"), V::MakeFloat(C.RangeMin)},
				{TEXT("range_max"), V::MakeFloat(C.RangeMax)}}); })}}); })},
		{TEXT("sensors"), ArrayOf(D.Sensors, [](const FMjSensorDescriptor& X) { return Map({
			{TEXT("id"), V::MakeInt(X.Id)}, {TEXT("name"), V::MakeString(X.Name)},
			{TEXT("address"), V::MakeInt(X.Address)}, {TEXT("dimension"), V::MakeInt(X.Dimension)},
			{TEXT("type"), V::MakeInt(X.Type)}, {TEXT("semantic"), V::MakeInt(static_cast<int64>(X.Semantic))}}); })},
		{TEXT("bodies"), ArrayOf(D.Bodies, Named)}, {TEXT("cameras"), ArrayOf(D.Cameras, Named)}});
}

V Observation(const FMjModelObservation& O)
{
	return Map({
		{TEXT("model"), Identity(O.Identity)},
		{TEXT("clock_epoch"), V::MakeInt(static_cast<int64>(O.ClockEpoch))},
		{TEXT("step_sequence"), V::MakeInt(static_cast<int64>(O.StepSequence))},
		{TEXT("time"), V::MakeFloat(O.Time)}, {TEXT("qpos"), Doubles(O.Qpos)},
		{TEXT("qvel"), Doubles(O.Qvel)}, {TEXT("ctrl"), Doubles(O.Ctrl)},
		{TEXT("sensordata"), Doubles(O.SensorData)}});
}

V ControlStatus(const FMjSimulationControlStatus& Status)
{
	return Map({
		{TEXT("claimed"), V::MakeBool(Status.bClaimed)},
		{TEXT("model_generation"), V::MakeInt(static_cast<int64>(Status.ModelGeneration))},
		{TEXT("source_kind"), V::MakeString(Status.SourceKind ==
			EMjSimulationControlSourceKind::Local ? TEXT("local") : TEXT("remote"))},
		{TEXT("ttl_s"), V::MakeFloat(Status.TtlSeconds)},
		{TEXT("expires"), V::MakeBool(Status.bExpires)},
		{TEXT("seconds_remaining"), V::MakeFloat(Status.SecondsRemaining)},
		{TEXT("control_revision"), V::MakeInt(static_cast<int64>(Status.ControlRevision))}});
}

V CameraStream(const FURLabCameraStreamDescriptor& D)
{
	return Map({{TEXT("kind"), V::MakeString(TEXT("camera_frame"))},
		{TEXT("schema_version"), V::MakeInt(1)},
		{TEXT("producer_boot_id"), V::MakeString(D.ProducerBootId)},
		{TEXT("stream_id"), V::MakeString(D.StreamId)},
		{TEXT("stream_generation"), V::MakeInt(D.StreamGeneration)},
		{TEXT("source_revision"), V::MakeString(D.SourceRevision)},
		{TEXT("camera_name"), V::MakeString(D.CameraName)},
		{TEXT("camera_configuration_revision"), V::MakeInt(D.CameraConfigurationRevision)},
		{TEXT("encoding"), V::MakeString(D.Encoding)},
		{TEXT("width"), V::MakeInt(D.Width)}, {TEXT("height"), V::MakeInt(D.Height)},
		{TEXT("stride_bytes"), V::MakeInt(D.StrideBytes)}});
}

V ClockStatus(const FMjSimulationClockStatus& Status)
{
	return Map({
		{TEXT("mode"), V::MakeString(Status.Mode == EMjSimulationClockMode::FreeRun
			? TEXT("freerun") : TEXT("stepped"))},
		{TEXT("model_generation"), V::MakeInt(static_cast<int64>(Status.ModelGeneration))},
		{TEXT("time_step_s"), V::MakeFloat(Status.TimeStepSeconds)},
		{TEXT("accumulated_debt_s"), V::MakeFloat(Status.AccumulatedDebtSeconds)},
		{TEXT("dropped_time_s"), V::MakeFloat(Status.DroppedTimeSeconds)},
		{TEXT("dropped_step_count"), V::MakeInt(static_cast<int64>(Status.DroppedStepCount))},
		{TEXT("max_steps_per_advance"), V::MakeInt(Status.MaxStepsPerAdvance)},
		{TEXT("max_retained_debt_s"), V::MakeFloat(Status.MaxRetainedDebtSeconds)}});
}


const TCHAR* ControlErrorCode(EMjSimulationControlError Error)
{
	switch (Error)
	{
	case EMjSimulationControlError::NoModel: return TEXT("model_not_ready");
	case EMjSimulationControlError::InvalidSession:
	case EMjSimulationControlError::InactiveSession: return TEXT("session_expired");
	case EMjSimulationControlError::InvalidToken:
	case EMjSimulationControlError::NotClaimed: return TEXT("not_control_owner");
	case EMjSimulationControlError::WrongModelGeneration: return TEXT("stale_model_generation");
	case EMjSimulationControlError::AlreadyClaimed: return TEXT("control_claimed");
	case EMjSimulationControlError::Expired: return TEXT("control_expired");
	case EMjSimulationControlError::WrongControlWidth: return TEXT("dim_mismatch");
	case EMjSimulationControlError::UnknownActuator: return TEXT("unknown_actuator");
	case EMjSimulationControlError::CounterExhausted: return TEXT("counter_exhausted");
	case EMjSimulationControlError::WrongClockMode: return TEXT("wrong_clock_mode");
	case EMjSimulationControlError::InvalidState: return TEXT("invalid_state");
	case EMjSimulationControlError::ObservationUnavailable:
		return TEXT("mutation_committed_observation_unavailable");
	case EMjSimulationControlError::InvalidTtl:
	case EMjSimulationControlError::NonFinite:
	case EMjSimulationControlError::OutOfRange:
	case EMjSimulationControlError::InvalidStepCount: return TEXT("bad_value");
	default: return TEXT("internal_error");
	}
}

const TCHAR* ClockErrorCode(EMjSimulationClockError Error)
{
	switch (Error)
	{
	case EMjSimulationClockError::NoModel: return TEXT("model_not_ready");
	case EMjSimulationClockError::WrongModelGeneration: return TEXT("stale_model_generation");
	case EMjSimulationClockError::InvalidMode: return TEXT("bad_value");
	case EMjSimulationClockError::CounterExhausted: return TEXT("counter_exhausted");
	default: return TEXT("internal_error");
	}
}


bool ReadIdentity(const V& Value, FMjModelIdentity& Out)
{
	auto Int32Field = [&Value](const TCHAR* Name, int32& Target)
	{
		const V* Field = Value.FindMapValue(Name);
		if (!Field || Field->GetInt() > MAX_int32) return false;
		Target = static_cast<int32>(Field->GetInt());
		return true;
	};
	const V* Hash = Value.FindMapValue(TEXT("content_sha256"));
	const V* Generation = Value.FindMapValue(TEXT("generation"));
	if (!Hash || !Generation) return false;
	Out.ContentSha256 = Hash->GetString();
	Out.Generation = static_cast<uint64>(Generation->GetInt());
	return Int32Field(TEXT("mujoco_version"), Out.MuJoCoVersion)
		&& Int32Field(TEXT("nq"), Out.Nq) && Int32Field(TEXT("nv"), Out.Nv)
		&& Int32Field(TEXT("nu"), Out.Nu) && Int32Field(TEXT("na"), Out.Na)
		&& Int32Field(TEXT("nsensordata"), Out.NSensorData);
}

FMjSimulationControlHandle ControlHandle(const FURLabRequestEnvelope& Request)
{
	FMjSimulationControlHandle Handle;
	Handle.SessionId = Request.SessionId;
	Handle.ClaimToken = Request.Body.FindMapValue(TEXT("control_token"))->GetString();
	Handle.ModelGeneration = static_cast<uint64>(
		Request.Body.FindMapValue(TEXT("model_generation"))->GetInt());
	return Handle;
}

bool ValidateOperationMetadata(const FURLabRequestEnvelope& Request,
	bool& bOutRequiresInput, FString& OutError)
{
	const URLabWire::TypedOp::FDecl* Decl = URLabWire::TypedOp::Find(Request.Op);
	if (!Decl || !Decl->Service || !Decl->Execution || !Decl->Authority || !Decl->Permission
		|| FString(Decl->Service) != TEXT("simulation")
		|| FString(Decl->Execution) != TEXT("simulation_queue"))
	{
		OutError = TEXT("operation has missing or invalid simulation service metadata");
		return false;
	}
	const bool bClaim = Request.Op == TEXT("simulation_claim_control");
	const bool bNoAuthority = Request.Op == TEXT("simulation_describe")
		|| Request.Op == TEXT("simulation_observe")
		|| Request.Op == TEXT("simulation_capture_state")
		|| Request.Op == TEXT("simulation_control_status")
		|| Request.Op == TEXT("simulation_clock_status");
	const FString ExpectedAuthority = bClaim ? TEXT("session")
		: bNoAuthority ? TEXT("none") : TEXT("control");
	const FString ExpectedPermission = bNoAuthority ? TEXT("none") : TEXT("accept_input");
	if (FString(Decl->Authority) != ExpectedAuthority
		|| FString(Decl->Permission) != ExpectedPermission)
	{
		OutError = TEXT("operation authority or permission metadata is invalid");
		return false;
	}
	bOutRequiresInput = ExpectedPermission == TEXT("accept_input");
	return true;
}

void ExecuteTypedSimulation(
		const TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe>& Completion,
	const TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe>& Runtime,
	const TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe>& CameraBinding,
	const TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>& ControlSession,
	const TWeakObjectPtr<UURLabBridgeServer>& Bridge, bool bRequiresInput,
	const FString& MetadataError)
{
	if (!MetadataError.IsEmpty())
	{
		Completion->Error(TEXT("server_schema_error"), MetadataError);
		return;
	}
	if (bRequiresInput && (!Bridge.IsValid()
		|| !Bridge->GrantsPermission(EMjCapability::AcceptInput)))
	{
		Completion->Error(TEXT("capability_disabled"),
			TEXT("This instance does not currently grant accept_input"));
		return;
	}
	if (!Runtime)
	{
		Completion->Error(TEXT("service_unavailable"), TEXT("Simulation service is unavailable"));
		return;
	}

	const FString& Op = Completion->Request.Op;
	if (Op == TEXT("simulation_interaction_grant_issue")
		|| Op == TEXT("simulation_interaction_grant_revoke"))
	{
		const TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> Owner =
			Runtime->GetInteractionOwner();
		if (!Owner)
		{
			Completion->Error(TEXT("unsupported"),
				TEXT("Simulation pose publisher has no interaction handler"));
			return;
		}
		FURLabInteractionSourceIdentity Source;
		if (!URLabInteractionWire::ParseSource(
			*Completion->Request.Body.FindMapValue(TEXT("source")), Source))
		{
			Completion->Error(TEXT("source_revision_conflict"),
				TEXT("Interaction source identity is invalid"));
			return;
		}
		FString Error;
		if (Op == TEXT("simulation_interaction_grant_revoke"))
		{
			const bool bRevoked = Owner->RevokeGrant(*Runtime,
				ControlHandle(Completion->Request), Source,
				Completion->Request.Body.FindMapValue(TEXT("grant_token"))->GetString(), Error);
			if (!bRevoked)
			{
				Completion->Error(Error.Contains(TEXT("control")) ? TEXT("control_invalid")
					: TEXT("grant_invalid"), Error);
				return;
			}
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
				Map({{TEXT("revoked"), V::MakeBool(true)}})));
			return;
		}
		const double Ttl = Completion->Request.Body.FindMapValue(TEXT("ttl_s"))->GetFloat();
		FURLabInteractionGrant Grant;
		if (!Owner->IssueGrant(*Runtime, ControlHandle(Completion->Request), Source,
			Completion->Request.Body.FindMapValue(TEXT("viewer_boot_id"))->GetString(),
			Ttl, Grant, Error))
		{
			Completion->Error(Error.Contains(TEXT("control")) ? TEXT("control_invalid")
				: Error.Contains(TEXT("TTL")) ? TEXT("bad_request")
				: TEXT("source_revision_conflict"), Error);
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
			Map({{TEXT("grant"), URLabInteractionWire::Grant(Grant, Ttl)}})));
		return;
	}
	if (Op == TEXT("simulation_camera_stream_configure"))
	{
		if (!CameraBinding)
		{
			Completion->Error(TEXT("capability_disabled"), TEXT("camera_frames output is disabled")); return;
		}
		const FString Name = Completion->Request.Body.FindMapValue(TEXT("camera_name"))->GetString();
		const bool bEnabled = Completion->Request.Body.FindMapValue(TEXT("enabled"))->GetBool();
		TOptional<double> Rate;
		if (const V* Field = Completion->Request.Body.FindMapValue(TEXT("rate_hz"))) Rate = Field->GetFloat();
		FURLabCameraStreamDescriptor Descriptor; FString Error;
		if (!CameraBinding->ConfigureSimulation(ControlHandle(Completion->Request), Name,
			bEnabled, Rate, Descriptor, Error))
		{
			Completion->Error(Error.Contains(TEXT("generation")) || Error.Contains(TEXT("control"))
				? TEXT("control_invalid") : TEXT("camera_invalid"), Error); return;
		}
		TArray<FString> Keys{TEXT("camera_name"), TEXT("enabled")};
		TArray<V> Values{V::MakeString(Name), V::MakeBool(bEnabled)};
		if (Rate.IsSet()) { Keys.Add(TEXT("rate_hz")); Values.Add(V::MakeFloat(Rate.GetValue())); }
		if (bEnabled) { Keys.Add(TEXT("stream")); Values.Add(CameraStream(Descriptor)); }
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
			V::MakeMap(MoveTemp(Keys), MoveTemp(Values)))); return;
	}
	if (Op == TEXT("simulation_describe"))
	{
		FMjModelDescriptor D;
		if (!Runtime->GetDescriptor(D))
		{
			Completion->Error(TEXT("model_not_ready"), TEXT("No installed model is ready"));
			return;
		}
		if (D.Identity.Generation > static_cast<uint64>(MAX_int64))
		{
			Completion->Error(TEXT("encoding_error"), TEXT("Model generation exceeds the wire integer range"));
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Describe(D)));
		return;
	}
	if (Op == TEXT("simulation_observe"))
	{
		FMjModelObservation O;
		if (!Runtime->GetObservation(O))
		{
			Completion->Error(TEXT("model_not_ready"), TEXT("A coherent observation is unavailable"));
			return;
		}
		int64 Epoch, Sequence, Generation;
		if (!Signed(O.ClockEpoch, Epoch) || !Signed(O.StepSequence, Sequence)
			|| !Signed(O.Identity.Generation, Generation))
		{
			Completion->Error(TEXT("encoding_error"), TEXT("Observation counters exceed the wire integer range"));
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(
			Completion->Request, Observation(O)));
		return;
	}
	if (Op == TEXT("simulation_capture_state"))
	{
		const int64 Mask = Completion->Request.Body.FindMapValue(TEXT("mask"))->GetInt();
		FMjModelState State;
		const EMjModelStateError Error = Runtime->CaptureState(static_cast<uint32>(Mask), State);
		if (Error != EMjModelStateError::None)
		{
			const TCHAR* Code = Error == EMjModelStateError::NoModel ? TEXT("model_not_ready")
				: Error == EMjModelStateError::InvalidMask ? TEXT("invalid_state_mask")
				: Error == EMjModelStateError::NonFinite ? TEXT("invalid_state")
				: TEXT("state_unavailable");
			Completion->Error(Code, TEXT("The requested model state could not be captured"));
			return;
		}
		int64 Epoch, Generation;
		if (!Signed(State.ClockEpoch, Epoch) || !Signed(State.Identity.Generation, Generation))
		{
			Completion->Error(TEXT("encoding_error"), TEXT("State counters exceed the wire integer range"));
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("model"), Identity(State.Identity)}, {TEXT("clock_epoch"), V::MakeInt(Epoch)},
			{TEXT("mask"), V::MakeInt(State.StateMask)},
			{TEXT("scalar"), V::MakeString(TEXT("float64"))},
			{TEXT("complete_checkpoint"), V::MakeBool(State.bCompleteCheckpoint)},
			{TEXT("state_size"), V::MakeInt(State.Values.Num())},
			{TEXT("state"), Doubles(State.Values)}})));
		return;
	}
	if (Op == TEXT("simulation_control_status"))
	{
		Completion->Publish(FURLabReplyEnvelope::MakeResult(
			Completion->Request, ControlStatus(Runtime->GetControlStatus())));
		return;
	}
	if (Op == TEXT("simulation_clock_status"))
	{
		const FMjSimulationClockStatus Status = Runtime->GetClockStatus();
		int64 Generation = 0, Dropped = 0;
		if (!Signed(Status.ModelGeneration, Generation)
			|| !Signed(Status.DroppedStepCount, Dropped))
		{
			Completion->Error(TEXT("encoding_error"),
				TEXT("Clock counters exceed the wire integer range"));
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(
			Completion->Request, ClockStatus(Status)));
		return;
	}
	if (Op == TEXT("simulation_claim_control"))
	{
		FMjSimulationControlClaimRequest Request;
		Request.Session = ControlSession;
		Request.SourceKind = EMjSimulationControlSourceKind::Remote;
		Request.ExpectedModelGeneration = static_cast<uint64>(
			Completion->Request.Body.FindMapValue(TEXT("model_generation"))->GetInt());
		Request.TtlSeconds = Completion->Request.Body.FindMapValue(TEXT("ttl_s"))->GetFloat();
		const FMjSimulationControlClaimResult Result = Runtime->ClaimControl(Request);
		if (!Result.IsOk())
		{
			Completion->Error(ControlErrorCode(Result.Error), Result.Message);
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("control"), Map({
				{TEXT("token"), V::MakeString(Result.Handle.ClaimToken)},
				{TEXT("model_generation"), V::MakeInt(static_cast<int64>(Result.Handle.ModelGeneration))}})},
			{TEXT("status"), ControlStatus(Result.Status)}})));
		return;
	}

	const FMjSimulationControlHandle Handle = ControlHandle(Completion->Request);
	if (Op == TEXT("simulation_set_clock"))
	{
		const FString& Mode = Completion->Request.Body.FindMapValue(TEXT("mode"))->GetString();
		const FMjSimulationClockChangeResult Result = Runtime->AuthorizedSetClockMode(
			Handle, Mode == TEXT("freerun") ? EMjSimulationClockMode::FreeRun
				: EMjSimulationClockMode::Stepped);
		if (Result.AuthorityError != EMjSimulationControlError::None)
		{
			Completion->Error(ControlErrorCode(Result.AuthorityError), Result.Message);
			return;
		}
		if (Result.Error != EMjSimulationClockError::None)
		{
			Completion->Error(ClockErrorCode(Result.Error), Result.Message);
			return;
		}
		int64 Generation = 0, Dropped = 0;
		if (!Signed(Result.Status.ModelGeneration, Generation)
			|| !Signed(Result.Status.DroppedStepCount, Dropped))
		{
			Completion->Error(TEXT("encoding_error"),
				TEXT("Clock counters exceed the wire integer range"));
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("changed"), V::MakeBool(Result.bCommitted)},
			{TEXT("clock"), ClockStatus(Result.Status)}})));
		return;
	}
	if (Op == TEXT("simulation_renew_control"))
	{
		const FMjSimulationControlLeaseResult Result = Runtime->RenewControl(Handle,
			Completion->Request.Body.FindMapValue(TEXT("ttl_s"))->GetFloat());
		if (!Result.IsOk())
		{
			Completion->Error(ControlErrorCode(Result.Error), Result.Message);
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("changed"), V::MakeBool(Result.bChanged)},
			{TEXT("status"), ControlStatus(Result.Status)}})));
		return;
	}
	if (Op == TEXT("simulation_release_control"))
	{
		const FMjSimulationControlLeaseResult Result = Runtime->ReleaseControl(Handle);
		if (!Result.IsOk())
		{
			Completion->Error(ControlErrorCode(Result.Error), Result.Message);
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("changed"), V::MakeBool(Result.bChanged)},
			{TEXT("status"), ControlStatus(Result.Status)}})));
		return;
	}

	FMjSimulationMutationResult Mutation;
	if (Op == TEXT("simulation_set_control"))
	{
		TArray<double> Values;
		for (const V& Value : Completion->Request.Body.FindMapValue(TEXT("ctrl"))->GetArray())
		{
			Values.Add(Value.GetFloat());
		}
		Mutation = Runtime->SetControlVector(Handle, Values);
	}
	else if (Op == TEXT("simulation_set_actuators"))
	{
		TMap<FString, TArray<double>> Values;
		for (const V& Entry : Completion->Request.Body.FindMapValue(TEXT("actuators"))->GetArray())
		{
			const FString& Name = Entry.FindMapValue(TEXT("name"))->GetString();
			if (Values.Contains(Name))
			{
				Completion->Error(TEXT("bad_request"),
					FString::Printf(TEXT("actuator '%s' appears more than once"), *Name));
				return;
			}
			TArray<double> Controls;
			for (const V& Value : Entry.FindMapValue(TEXT("values"))->GetArray())
			{
				Controls.Add(Value.GetFloat());
			}
			Values.Add(Name, MoveTemp(Controls));
		}
		Mutation = Runtime->SetNamedActuatorControls(Handle, Values);
	}
	else if (Op == TEXT("simulation_step"))
	{
		Mutation = Runtime->AuthorizedStep(Handle, static_cast<uint32>(
			Completion->Request.Body.FindMapValue(TEXT("count"))->GetInt()));
	}
	else if (Op == TEXT("simulation_reset"))
	{
		Mutation = Runtime->AuthorizedReset(Handle);
	}
	else if (Op == TEXT("simulation_forward"))
	{
		Mutation = Runtime->AuthorizedForward(Handle);
	}
	else if (Op == TEXT("simulation_apply_state"))
	{
		const V& StateValue = *Completion->Request.Body.FindMapValue(TEXT("state"));
		const V& ModelValue = *StateValue.FindMapValue(TEXT("model"));
		FMjModelState State;
		if (!ReadIdentity(ModelValue, State.Identity))
		{
			Completion->Error(TEXT("invalid_state"),
				TEXT("captured state identity exceeds native model dimensions"));
			return;
		}
		State.ClockEpoch = static_cast<uint64>(
			StateValue.FindMapValue(TEXT("clock_epoch"))->GetInt());
		State.StateMask = static_cast<uint32>(StateValue.FindMapValue(TEXT("mask"))->GetInt());
		State.Scalar = EMjModelStateScalar::Float64;
		State.bCompleteCheckpoint = StateValue.FindMapValue(TEXT("complete_checkpoint"))->GetBool();
		for (const V& Value : StateValue.FindMapValue(TEXT("state"))->GetArray())
		{
			State.Values.Add(Value.GetFloat());
		}
		Mutation = Runtime->AuthorizedApplyState(Handle, State);
	}
	else
	{
		Completion->Error(TEXT("server_schema_error"),
			TEXT("active simulation operation has no runtime handler"));
		return;
	}

	if (!Mutation.bCommitted || Mutation.Error != EMjSimulationControlError::None)
	{
		Completion->Error(ControlErrorCode(Mutation.Error), Mutation.Message);
		return;
	}
	Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
		{TEXT("committed"), V::MakeBool(true)},
		{TEXT("observation"), Observation(Mutation.Observation)},
		{TEXT("status"), ControlStatus(Mutation.Status)}})),
		TEXT("mutation_committed_observation_unavailable"));
}
} // namespace

bool FURLabRpcDispatcher::DispatchTypedSimulation(const FURLabRequestEnvelope& Request,
	const FURLabRequestDigest& Digest, const FURLabRequestLedger::FSessionRef& Session,
	const TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>& ControlSession,
	TArray<uint8>& OutReplyBytes)
{
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime;
	FURLabRequestLedger::FAdmissionResult Admission;
	bool bStartDrain = false;
	const TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe> Completion =
		MakeShared<FURLabRetainedCompletion, ESPMode::ThreadSafe>(
			RequestLedger, Session, Request, Digest);
	{
		// Admission and enqueue share this short lock, establishing one accepted
		// order across transport threads. Work retains the exact runtime and session
		// scope selected here; detach cannot replace either beneath queued work.
		FScopeLock Lock(&DispatchMutex);
		Runtime = SimulationRuntime;
		const TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> CameraBinding = CameraStreamService;
		Admission = RequestLedger->Admit(
			Session, static_cast<int64>(Request.RequestId), Digest);
		if (Admission.Status == FURLabRequestLedger::Accepted)
		{
			bool bRequiresInput = false;
			FString MetadataError;
			ValidateOperationMetadata(Request, bRequiresInput, MetadataError);
			const TWeakObjectPtr<UURLabBridgeServer> Bridge = OwningBridge;
			FURLabGameThreadCommandQueue::FWork Work;
			Work.Execute = [Completion, Runtime, CameraBinding, ControlSession, Bridge,
				bRequiresInput, MetadataError = MoveTemp(MetadataError)]() mutable
			{
				ExecuteTypedSimulation(Completion, Runtime, CameraBinding, ControlSession, Bridge,
					bRequiresInput, MetadataError);
			};
			Work.Cancel = [Completion]()
			{
				Completion->Error(TEXT("service_unavailable"),
					TEXT("Simulation command was cancelled during service shutdown"));
			};
			bStartDrain = SimulationCommandQueue->Enqueue(MoveTemp(Work));
		}
	}
	const EURLabRetainedAdmissionReply AdmissionReply =
		FURLabRetainedCompletion::EncodeAdmission(Admission, Request, OutReplyBytes);
	if (AdmissionReply != EURLabRetainedAdmissionReply::Accepted)
		return AdmissionReply == EURLabRetainedAdmissionReply::Replied;

	if (IsInGameThread())
	{
		SimulationCommandQueue->Drain();
		const FURLabRequestLedger::FAdmissionResult Completed =
			RequestLedger->Admit(Session, static_cast<int64>(Request.RequestId), Digest);
		if (Completed.Terminal == FURLabRequestLedger::Completed && Completed.ReplyBytes)
		{
			OutReplyBytes = *Completed.ReplyBytes;
			return true;
		}
		if (Completed.Terminal == FURLabRequestLedger::ResultUnavailable)
		{
			return FURLabRequestEnvelopeCodec::EncodeReply(FURLabReplyEnvelope::MakeError(Request,
				TEXT("result_unavailable"), TEXT("The retained result exceeded the inline reply limit"),
				EURLabExecution::Terminal), OutReplyBytes).IsOk();
		}
	}
	else if (bStartDrain)
	{
		const TSharedRef<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe> Queue =
			SimulationCommandQueue;
		AsyncTask(ENamedThreads::GameThread, [Queue]() { Queue->Drain(); });
	}
	FURLabRequestEnvelopeCodec::EncodeReply(FURLabReplyEnvelope::MakePending(Request), OutReplyBytes);
	return true;
}
