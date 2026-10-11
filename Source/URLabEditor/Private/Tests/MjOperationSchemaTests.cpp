// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Misc/AutomationTest.h"
#include "Wire/OperationSchema.h"

#include <limits>

namespace
{
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

V Doubles(std::initializer_list<double> Input)
{
	TArray<V> Values;
	for (double Value : Input)
	{
		Values.Add(V::MakeFloat(Value));
	}
	return V::MakeArray(MoveTemp(Values));
}

V ModelIdentity()
{
	return Map({
		{TEXT("content_sha256"), V::MakeString(TEXT("abab"))},
		{TEXT("generation"), V::MakeInt(4)},
		{TEXT("mujoco_version"), V::MakeInt(3011001)},
		{TEXT("nq"), V::MakeInt(2)},
		{TEXT("nv"), V::MakeInt(2)},
		{TEXT("nu"), V::MakeInt(1)},
		{TEXT("na"), V::MakeInt(0)},
		{TEXT("nsensordata"), V::MakeInt(1)},
	});
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActiveOperationSchemaTest,
	"URLab.Protocol.OperationSchema.ActiveReads",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjActiveOperationSchemaTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const V Empty = Map({});
	TestTrue(TEXT("describe has an exact empty body"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_describe"), Empty).IsOk());
	TestTrue(TEXT("observe has an exact empty body"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_observe"), Empty).IsOk());
	TestTrue(TEXT("capture accepts a positive uint32 mask"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_capture_state"),
			Map({{TEXT("mask"), V::MakeInt(0x3fff)}})).IsOk());

	const URLabWire::TypedOp::FDecl* Describe =
		URLabWire::TypedOp::Find(TEXT("simulation_describe"));
	TestNotNull(TEXT("active op metadata generated"), Describe);
	if (Describe)
	{
		TestEqual(TEXT("service explicit"), FString(Describe->Service), FString(TEXT("simulation")));
		TestEqual(TEXT("permission explicit"), FString(Describe->Permission), FString(TEXT("none")));
		TestEqual(TEXT("authority explicit"), FString(Describe->Authority), FString(TEXT("none")));
		TestEqual(TEXT("execution explicit"), FString(Describe->Execution),
			FString(TEXT("simulation_queue")));
		TestEqual(TEXT("one adapter"), Describe->NumAdapters, 1);
		TestEqual(TEXT("ZMQ adapter"), FString(Describe->Adapters[0]), FString(TEXT("zmq")));
	}

	TestFalse(TEXT("zero mask rejected"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_capture_state"),
			Map({{TEXT("mask"), V::MakeInt(0)}})).IsOk());
	TestFalse(TEXT("float mask rejected without numeric coercion"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_capture_state"),
			Map({{TEXT("mask"), V::MakeFloat(1.0)}})).IsOk());
	TestFalse(TEXT("missing mask rejected"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_capture_state"), Empty).IsOk());
	TestFalse(TEXT("unknown empty-body field rejected"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_describe"),
			Map({{TEXT("extra"), V::MakeInt(1)}})).IsOk());
	TestFalse(TEXT("duplicate field rejected"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_capture_state"),
			Map({{TEXT("mask"), V::MakeInt(1)}, {TEXT("mask"), V::MakeInt(1)}})).IsOk());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjResultOptionalOperationSchemaTest,
	"URLab.Protocol.OperationSchema.ResultOptionalFields",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjResultOptionalOperationSchemaTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const V Describe = Map({
		{TEXT("scene_revision"), V::MakeString(TEXT("scene-a"))},
		{TEXT("scene_generation"), V::MakeInt(1)},
		{TEXT("body_names"), V::MakeArray(TArray<V>())},
		{TEXT("model_cameras"), V::MakeArray(TArray<V>())},
		{TEXT("light_names"), V::MakeArray(TArray<V>())},
		{TEXT("supports_qpos"), V::MakeBool(false)},
		{TEXT("qpos_width"), V::MakeInt(0)},
	});
	TestTrue(TEXT("optional nested user camera may be absent"),
		FURLabOperationSchema::ValidateResultBody(TEXT("render_scene_describe"), Describe).IsOk());
	const V MissingRequired = Map({
		{TEXT("scene_generation"), V::MakeInt(1)},
		{TEXT("body_names"), V::MakeArray(TArray<V>())},
		{TEXT("model_cameras"), V::MakeArray(TArray<V>())},
		{TEXT("light_names"), V::MakeArray(TArray<V>())},
		{TEXT("supports_qpos"), V::MakeBool(false)},
		{TEXT("qpos_width"), V::MakeInt(0)},
	});
	TestFalse(TEXT("missing required result field still rejects"),
		FURLabOperationSchema::ValidateResultBody(
			TEXT("render_scene_describe"), MissingRequired).IsOk());

	const V PendingPoll = Map({
		{TEXT("state"), V::MakeString(TEXT("queued"))},
		{TEXT("frame"), Map({
			{TEXT("scene_revision"), V::MakeString(TEXT("scene-a"))},
			{TEXT("frame_id"), V::MakeInt(0)},
			{TEXT("clock_epoch"), V::MakeInt(0)},
			{TEXT("has_source_time"), V::MakeBool(false)},
		})},
		{TEXT("gpu_writers_drained"), V::MakeBool(true)},
		{TEXT("geometry_configuration_id"), V::MakeString(TEXT("baseline"))},
		{TEXT("materialization"), Map({
			{TEXT("state"), V::MakeString(TEXT("pending"))},
		})},
	});
	TestTrue(TEXT("tagged result recursion preserves nested optional fields"),
		FURLabOperationSchema::ValidateResultBody(
			TEXT("render_capture_poll"), PendingPoll).IsOk());
	const V MissingTaggedRequired = Map({
		{TEXT("state"), V::MakeString(TEXT("succeeded"))},
		{TEXT("payload"), Map({})},
	});
	const V InvalidPoll = Map({
		{TEXT("state"), V::MakeString(TEXT("succeeded"))},
		{TEXT("frame"), *PendingPoll.FindMapValue(TEXT("frame"))},
		{TEXT("gpu_writers_drained"), V::MakeBool(true)},
		{TEXT("geometry_configuration_id"), V::MakeString(TEXT("baseline"))},
		{TEXT("materialization"), MissingTaggedRequired},
	});
	TestFalse(TEXT("tagged result recursion still requires variant fields"),
		FURLabOperationSchema::ValidateResultBody(
			TEXT("render_capture_poll"), InvalidPoll).IsOk());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRecursiveOperationSchemaTest,
	"URLab.Protocol.OperationSchema.RecursiveShapes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRecursiveOperationSchemaTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const URLabWire::TypedRequest::FShape* NamedControl =
		URLabWire::TypedRequest::Find(TEXT("simulation_set_actuators"));
	const URLabWire::TypedRequest::FShape* State =
		URLabWire::TypedRequest::Find(TEXT("simulation_apply_state"));
	TestNotNull(TEXT("planned named-control shape generated"), NamedControl);
	TestNotNull(TEXT("planned state shape generated"), State);
	if (!NamedControl || !State)
	{
		return false;
	}

	TArray<V> Updates;
	Updates.Add(Map({
		{TEXT("name"), V::MakeString(TEXT("motor"))},
		{TEXT("values"), Doubles({1.0, -2.0})},
	}));
	const V NamedBody = Map({
		{TEXT("model_generation"), V::MakeInt(4)},
		{TEXT("control_token"), V::MakeString(TEXT("lease"))},
		{TEXT("actuators"), V::MakeArray(MoveTemp(Updates))},
	});
	TestTrue(TEXT("nested actuator maps validate recursively for the active operation"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_set_actuators"),
			NamedBody).IsOk());

	const V ValidState = Map({
		{TEXT("model_generation"), V::MakeInt(4)},
		{TEXT("control_token"), V::MakeString(TEXT("lease"))},
		{TEXT("state"), Map({
			{TEXT("model"), ModelIdentity()},
			{TEXT("clock_epoch"), V::MakeInt(2)},
			{TEXT("mask"), V::MakeInt(3)},
			{TEXT("scalar"), V::MakeString(TEXT("float64"))},
			{TEXT("complete_checkpoint"), V::MakeBool(true)},
			{TEXT("state_size"), V::MakeInt(2)},
			{TEXT("state"), Doubles({1.0, 2.0})},
		})},
	});
	TestTrue(TEXT("complete captured-state identity and dependent length validate"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_apply_state"),
			ValidState).IsOk());
	const V ShortState = Map({
		{TEXT("model_generation"), V::MakeInt(4)},
		{TEXT("control_token"), V::MakeString(TEXT("lease"))},
		{TEXT("state"), Map({
			{TEXT("model"), ModelIdentity()},
			{TEXT("clock_epoch"), V::MakeInt(2)},
			{TEXT("mask"), V::MakeInt(3)},
			{TEXT("scalar"), V::MakeString(TEXT("float64"))},
			{TEXT("complete_checkpoint"), V::MakeBool(true)},
			{TEXT("state_size"), V::MakeInt(2)},
			{TEXT("state"), Doubles({1.0})},
		})},
	});
	TestFalse(TEXT("dependent state array mismatch rejects"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("simulation_apply_state"),
			ShortState).IsOk());

	static const URLabWire::TypedRequest::FField PrimitiveFields[] = {
		{TEXT("text"), TEXT("string"), nullptr, nullptr, true},
		{TEXT("name"), TEXT("nonempty_string"), nullptr, nullptr, true},
		{TEXT("enabled"), TEXT("bool"), nullptr, nullptr, true},
		{TEXT("ratio"), TEXT("finite_float"), nullptr, nullptr, true},
		{TEXT("index"), TEXT("nonnegative_int64"), nullptr, nullptr, true},
		{TEXT("generation"), TEXT("positive_int64"), nullptr, nullptr, true},
		{TEXT("mask"), TEXT("positive_uint32"), nullptr, nullptr, true},
		{TEXT("blob"), TEXT("binary"), nullptr, nullptr, true},
		{TEXT("scalar"), TEXT("literal"), nullptr, TEXT("float64"), true},
		{TEXT("optional"), TEXT("string"), nullptr, nullptr, false},
	};
	static const URLabWire::TypedRequest::FShape PrimitiveShape = {
		TEXT("test_primitives"), PrimitiveFields, UE_ARRAY_COUNT(PrimitiveFields)};
	TArray<uint8> Bytes = {1, 2, 3};
	const V Primitives = Map({
		{TEXT("text"), V::MakeString(TEXT(""))},
		{TEXT("name"), V::MakeString(TEXT("motor"))},
		{TEXT("enabled"), V::MakeBool(true)},
		{TEXT("ratio"), V::MakeFloat(0.5)},
		{TEXT("index"), V::MakeInt(0)},
		{TEXT("generation"), V::MakeInt(1)},
		{TEXT("mask"), V::MakeInt(1)},
		{TEXT("blob"), V::MakeBinary(MoveTemp(Bytes))},
		{TEXT("scalar"), V::MakeString(TEXT("float64"))},
	});
	TestTrue(TEXT("all primitive types and omitted optional field validate"),
		FURLabOperationSchema::ValidateShape(Primitives, PrimitiveShape).IsOk());
	TArray<uint8> NonFiniteBytes = {1};
	const V NonFinite = Map({
		{TEXT("text"), V::MakeString(TEXT(""))},
		{TEXT("name"), V::MakeString(TEXT("motor"))},
		{TEXT("enabled"), V::MakeBool(true)},
		{TEXT("ratio"), V::MakeFloat(std::numeric_limits<double>::infinity())},
		{TEXT("index"), V::MakeInt(0)},
		{TEXT("generation"), V::MakeInt(1)},
		{TEXT("mask"), V::MakeInt(1)},
		{TEXT("blob"), V::MakeBinary(MoveTemp(NonFiniteBytes))},
		{TEXT("scalar"), V::MakeString(TEXT("float64"))},
	});
	TestFalse(TEXT("non-finite Float64 rejects"),
		FURLabOperationSchema::ValidateShape(NonFinite, PrimitiveShape).IsOk());
	return true;
}
