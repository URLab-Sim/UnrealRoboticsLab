// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Dom/JsonObject.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Wire/OperationSchema.h"
#include "Wire/RequestEnvelope.h"

#include <initializer_list>
#include <limits>

namespace
{
int32 HexNibble(TCHAR Character)
{
	if (Character >= '0' && Character <= '9') return Character - '0';
	if (Character >= 'a' && Character <= 'f') return Character - 'a' + 10;
	if (Character >= 'A' && Character <= 'F') return Character - 'A' + 10;
	return INDEX_NONE;
}

bool HexBytes(const FString& Hex, TArray<uint8>& Out)
{
	if ((Hex.Len() & 1) != 0) return false;
	Out.Reset(Hex.Len() / 2);
	for (int32 Index = 0; Index < Hex.Len(); Index += 2)
	{
		const int32 High = HexNibble(Hex[Index]);
		const int32 Low = HexNibble(Hex[Index + 1]);
		if (High < 0 || Low < 0) return false;
		Out.Add(static_cast<uint8>((High << 4) | Low));
	}
	return true;
}

FString LowerHex(const TArray<uint8>& Bytes)
{
	return BytesToHex(Bytes.GetData(), Bytes.Num()).ToLower();
}

TSharedPtr<FJsonObject> LoadVectors(FAutomationTestBase& Test)
{
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!Test.TestTrue(TEXT("UnrealRoboticsLab plugin is registered"), Plugin.IsValid()))
		return nullptr;
	const FString Path = FPaths::Combine(Plugin->GetBaseDir(), TEXT("wire"),
		TEXT("request_envelope_vectors.json"));
	FString Text;
	if (!Test.TestTrue(TEXT("request envelope fixture is readable"),
		FFileHelper::LoadFileToString(Text, *Path)))
		return nullptr;
	TSharedPtr<FJsonObject> Root;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Text);
	if (!Test.TestTrue(TEXT("request envelope fixture is valid JSON"),
		FJsonSerializer::Deserialize(Reader, Root) && Root.IsValid()))
		return nullptr;
	return Root;
}

FURLabWireValue OneEntryBody(const FString& Key, FURLabWireValue Value)
{
	TArray<FString> Keys{Key};
	TArray<FURLabWireValue> Values;
	Values.Add(MoveTemp(Value));
	return FURLabWireValue::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRequestEnvelopeSharedVectors,
	"URLab.Wire.RequestEnvelope.SharedCanonicalVectors",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRequestEnvelopeSharedVectors::RunTest(const FString& Parameters)
{
	const TSharedPtr<FJsonObject> Root = LoadVectors(*this);
	if (!Root) return false;

	const TArray<TSharedPtr<FJsonValue>>* Valid = nullptr;
	if (!TestTrue(TEXT("valid vector array exists"), Root->TryGetArrayField(TEXT("valid"), Valid)))
		return false;
	for (const TSharedPtr<FJsonValue>& Item : *Valid)
	{
		const TSharedPtr<FJsonObject> Vector = Item->AsObject();
		const FString Name = Vector->GetStringField(TEXT("name"));
		TArray<uint8> Packed;
		if (!TestTrue(*FString::Printf(TEXT("%s MessagePack hex decodes"), *Name),
			HexBytes(Vector->GetStringField(TEXT("msgpack_hex")), Packed)))
			continue;

		FURLabRequestEnvelope Envelope;
		const FURLabWireStatus Decode = FURLabRequestEnvelopeCodec::Decode(Packed, Envelope);
		if (!TestTrue(*FString::Printf(TEXT("%s decodes: %s"), *Name, *Decode.Detail), Decode.IsOk()))
			continue;
		TestEqual(*FString::Printf(TEXT("%s protocol"), *Name), Envelope.ProtocolVersion, 2);
		TestEqual(*FString::Printf(TEXT("%s request ID"), *Name), Envelope.RequestId, uint64(1));
		if (Name == TEXT("all_types_order_a"))
		{
			const FURLabWireValue* Binary = Envelope.Body.FindMapValue(TEXT("binary"));
			const FURLabWireValue* Positive = Envelope.Body.FindMapValue(TEXT("positive"));
			const FURLabWireValue* MinusZero = Envelope.Body.FindMapValue(TEXT("minus_zero"));
			const FURLabWireValue* NestedMap = Envelope.Body.FindMapValue(TEXT("map"));
			const TCHAR NulKeyXChars[] = {'a', 0, 'x'};
			const TCHAR NulKeyYChars[] = {'a', 0, 'y'};
			const FString NulKeyX = FString::ConstructFromPtrSize(
				NulKeyXChars, UE_ARRAY_COUNT(NulKeyXChars));
			const FString NulKeyY = FString::ConstructFromPtrSize(
				NulKeyYChars, UE_ARRAY_COUNT(NulKeyYChars));
			const FURLabWireValue* NulX = Envelope.Body.FindMapValue(NulKeyX);
			const FURLabWireValue* NulY = Envelope.Body.FindMapValue(NulKeyY);
			TestTrue(TEXT("binary remains binary"), Binary
				&& Binary->GetType() == EURLabWireValueType::Binary
				&& Binary->GetBinary() == TArray<uint8>{0, 255, 16});
			TestTrue(TEXT("positive integer remains integer"), Positive
				&& Positive->GetType() == EURLabWireValueType::Int64
				&& Positive->GetInt() == 42);
			TestTrue(TEXT("float remains float"), MinusZero
				&& MinusZero->GetType() == EURLabWireValueType::Float64);
			TestTrue(TEXT("nested map remains map"), NestedMap
				&& NestedMap->GetType() == EURLabWireValueType::Map);
			TestTrue(TEXT("distinct embedded-NUL key x survives"), NulX
				&& NulX->GetType() == EURLabWireValueType::Int64 && NulX->GetInt() == 3);
			TestTrue(TEXT("distinct embedded-NUL key y survives"), NulY
				&& NulY->GetType() == EURLabWireValueType::Int64 && NulY->GetInt() == 4);
			const FURLabWireValue* Text = Envelope.Body.FindMapValue(TEXT("text"));
			TestTrue(TEXT("embedded-NUL value keeps suffix"), Text
				&& Text->GetType() == EURLabWireValueType::String
				&& Text->GetString().Len() == 8 && Text->GetString()[3] == 0
				&& Text->GetString()[4] == 't' && Text->GetString()[5] == 'a'
				&& Text->GetString()[6] == 'i' && Text->GetString()[7] == 'l');
		}

		TArray<uint8> Canonical;
		const FURLabWireStatus Encoded = FURLabRequestEnvelopeCodec::CanonicalBytes(
			Envelope.Op, Envelope.Body, Canonical);
		if (!TestTrue(*FString::Printf(TEXT("%s canonicalizes: %s"), *Name, *Encoded.Detail),
			Encoded.IsOk()))
			continue;
		TestEqual(*FString::Printf(TEXT("%s canonical bytes"), *Name), LowerHex(Canonical),
			Vector->GetStringField(TEXT("canonical_hex")));

		FURLabRequestDigest Digest;
		const FURLabWireStatus Hashed = FURLabRequestEnvelopeCodec::Digest(
			Envelope.Op, Envelope.Body, Digest);
		if (!TestTrue(*FString::Printf(TEXT("%s hashes: %s"), *Name, *Hashed.Detail), Hashed.IsOk()))
			continue;
		TestEqual(*FString::Printf(TEXT("%s known SHA-256"), *Name), Digest.ToHex(),
			Vector->GetStringField(TEXT("sha256")));
	}

	const TArray<TSharedPtr<FJsonValue>>* Invalid = nullptr;
	if (!TestTrue(TEXT("invalid vector array exists"), Root->TryGetArrayField(TEXT("invalid"), Invalid)))
		return false;
	for (const TSharedPtr<FJsonValue>& Item : *Invalid)
	{
		const TSharedPtr<FJsonObject> Vector = Item->AsObject();
		const FString Name = Vector->GetStringField(TEXT("name"));
		TArray<uint8> Packed;
		if (!TestTrue(*FString::Printf(TEXT("%s invalid hex decodes"), *Name),
			HexBytes(Vector->GetStringField(TEXT("msgpack_hex")), Packed)))
			continue;
		FURLabRequestEnvelope Unchanged;
		Unchanged.RequestId = 917;
		FURLabWireLimits Limits;
		const TSharedPtr<FJsonObject>* CustomLimits = nullptr;
		if (Vector->TryGetObjectField(TEXT("limits"), CustomLimits)
			&& CustomLimits && CustomLimits->IsValid())
		{
			Limits.MaxInputBytes = static_cast<int32>(
				(*CustomLimits)->GetNumberField(TEXT("max_input_bytes")));
		}
		const FURLabWireStatus Status = FURLabRequestEnvelopeCodec::Decode(
			Packed, Unchanged, Limits);
		TestFalse(*FString::Printf(TEXT("%s is rejected"), *Name), Status.IsOk());
		TestEqual(*FString::Printf(TEXT("%s error code"), *Name),
			FString(FURLabRequestEnvelopeCodec::ErrorCodeString(Status.Code)),
			Vector->GetStringField(TEXT("error_code")));
		TestEqual(*FString::Printf(TEXT("%s leaves output unchanged"), *Name),
			Unchanged.RequestId, uint64(917));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRequestEnvelopeProgrammaticValidation,
	"URLab.Wire.RequestEnvelope.ProgrammaticCanonicalValidation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRequestEnvelopeProgrammaticValidation::RunTest(const FString& Parameters)
{
	TArray<uint8> Output;
	FURLabWireValue Duplicate = FURLabWireValue::MakeMap(
		TArray<FString>{TEXT("x"), TEXT("x")},
		TArray<FURLabWireValue>{FURLabWireValue::MakeInt(1), FURLabWireValue::MakeInt(2)});
	TestEqual(TEXT("programmatic duplicate map keys reject"),
		FURLabRequestEnvelopeCodec::CanonicalBytes(TEXT("render_frame"), Duplicate, Output).Code,
		EURLabWireError::DuplicateKey);
	TArray<uint8> SeededOutput{91, 92};
	TestEqual(TEXT("seeded canonical output rejects duplicate"),
		FURLabRequestEnvelopeCodec::CanonicalBytes(
			TEXT("render_frame"), Duplicate, SeededOutput).Code,
		EURLabWireError::DuplicateKey);
	TestTrue(TEXT("failed canonical encoding leaves output unchanged"),
		SeededOutput == TArray<uint8>{91, 92});
	FURLabRequestDigest SeededDigest;
	for (uint8& Byte : SeededDigest.Bytes) Byte = 0xA5;
	TestEqual(TEXT("seeded digest rejects duplicate"),
		FURLabRequestEnvelopeCodec::Digest(
			TEXT("render_frame"), Duplicate, SeededDigest).Code,
		EURLabWireError::DuplicateKey);
	bool bDigestUnchanged = true;
	for (const uint8 Byte : SeededDigest.Bytes) bDigestUnchanged &= Byte == 0xA5;
	TestTrue(TEXT("failed digest leaves output unchanged"), bDigestUnchanged);

	FURLabWireValue NonFinite = OneEntryBody(TEXT("x"),
		FURLabWireValue::MakeFloat(std::numeric_limits<double>::infinity()));
	TestEqual(TEXT("programmatic infinity rejects"),
		FURLabRequestEnvelopeCodec::CanonicalBytes(TEXT("render_frame"), NonFinite, Output).Code,
		EURLabWireError::NonFiniteFloat);

	FURLabWireValue Nested = FURLabWireValue::MakeNull();
	for (int32 Index = 0; Index < 65; ++Index)
	{
		TArray<FURLabWireValue> Item;
		Item.Add(MoveTemp(Nested));
		Nested = FURLabWireValue::MakeArray(MoveTemp(Item));
	}
	TestEqual(TEXT("programmatic depth rejects"),
		FURLabRequestEnvelopeCodec::CanonicalBytes(TEXT("render_frame"),
			OneEntryBody(TEXT("x"), MoveTemp(Nested)), Output).Code,
		EURLabWireError::DepthLimit);

	FURLabWireLimits TinyOutput;
	TinyOutput.MaxOutputBytes = 20;
	TestEqual(TEXT("canonical output bound rejects"),
		FURLabRequestEnvelopeCodec::CanonicalBytes(TEXT("render_frame"),
			OneEntryBody(TEXT("x"), FURLabWireValue::MakeString(TEXT("some text"))),
			Output, TinyOutput).Code,
		EURLabWireError::OutputTooLarge);

	FURLabWireLimits TinyItems;
	TinyItems.MaxContainerItems = 4;
	TArray<FURLabWireValue> FiveNulls;
	FiveNulls.Init(FURLabWireValue::MakeNull(), 5);
	TestEqual(TEXT("programmatic item bound rejects before encoding"),
		FURLabRequestEnvelopeCodec::CanonicalBytes(TEXT("render_frame"),
			OneEntryBody(TEXT("x"), FURLabWireValue::MakeArray(MoveTemp(FiveNulls))),
			Output, TinyItems).Code,
		EURLabWireError::ContainerLimit);

	TArray<uint8> Oversized;
	Oversized.SetNumZeroed(4 * 1024 * 1024 + 1);
	FURLabRequestEnvelope Unchanged;
	Unchanged.RequestId = 918;
	TestEqual(TEXT("input bound rejects before parsing"),
		FURLabRequestEnvelopeCodec::Decode(Oversized, Unchanged).Code,
		EURLabWireError::InputTooLarge);
	TestEqual(TEXT("oversize failure leaves output unchanged"), Unchanged.RequestId, uint64(918));

	const TCHAR EmbeddedChars[] = {'a', 0, 'b'};
	const FString Embedded = FString::ConstructFromPtrSize(
		EmbeddedChars, UE_ARRAY_COUNT(EmbeddedChars));
	const FURLabWireValue EmbeddedBody = OneEntryBody(TEXT("x"),
		FURLabWireValue::MakeString(Embedded));
	const FURLabWireStatus EmbeddedStatus = FURLabRequestEnvelopeCodec::CanonicalBytes(
		TEXT("render_frame"), EmbeddedBody, Output);
	TestTrue(TEXT("body strings preserve embedded NUL"), EmbeddedStatus.IsOk());
	TestTrue(TEXT("canonical bytes retain embedded NUL"), Output.Contains(0));

	FString BadOp(TEXT("bad"));
	BadOp.AppendChar('\n');
	TestEqual(TEXT("control in operation rejects"),
		FURLabRequestEnvelopeCodec::CanonicalBytes(BadOp, EmbeddedBody, Output).Code,
		EURLabWireError::EnvelopeFieldValue);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjWireMessageSharedVectors,
	"URLab.Wire.RequestEnvelope.SharedMessageVectors",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjWireMessageSharedVectors::RunTest(const FString& Parameters)
{
	const TSharedPtr<FJsonObject> Root = LoadVectors(*this);
	if (!Root) return false;
	const TSharedPtr<FJsonObject>* Messages = nullptr;
	if (!TestTrue(TEXT("message vector object exists"),
		Root->TryGetObjectField(TEXT("messages"), Messages)
		&& Messages && Messages->IsValid()))
	{
		return false;
	}
	auto LoadMessage = [this, &Messages](const TCHAR* Name, TArray<uint8>& Out)
	{
		return TestTrue(*FString::Printf(TEXT("%s hex decodes"), Name),
			HexBytes((*Messages)->GetStringField(Name), Out));
	};
	auto TestExact = [this](const TCHAR* Name, const TArray<uint8>& Actual,
		const TArray<uint8>& Expected)
	{
		TestTrue(*FString::Printf(TEXT("%s re-encodes exactly"), Name), Actual == Expected);
	};

	TArray<uint8> Packed;
	TArray<uint8> Encoded;
	FURLabWireStatus Status;
	if (LoadMessage(TEXT("connect_request"), Packed))
	{
		FURLabConnectRequest Request;
		Status = FURLabRequestEnvelopeCodec::DecodeConnectRequest(Packed, Request);
		TestTrue(TEXT("connect request decodes"), Status.IsOk());
		if (Status)
		{
			TestEqual(TEXT("connect request carries exact network policy"),
				Request.DataPolicy, FString(TEXT("network")));
			Status = FURLabRequestEnvelopeCodec::EncodeConnectRequest(Request, Encoded);
			TestTrue(TEXT("connect request encodes"), Status.IsOk());
			TestExact(TEXT("connect request"), Encoded, Packed);
		}
	}

	if (LoadMessage(TEXT("connect_reply"), Packed))
	{
		FURLabConnectReply Reply;
		Status = FURLabRequestEnvelopeCodec::DecodeConnectReply(Packed, Reply);
		TestTrue(TEXT("connect reply decodes"), Status.IsOk());
		if (Status)
		{
			TestTrue(TEXT("connect reply carries canonical permission"),
				Reply.Permissions == TArray<FString>{TEXT("camera_rpc")});
			TestEqual(TEXT("connect reply carries selected network data"),
				Reply.DataNegotiation.Transfer, FString(TEXT("network")));
			TestNotNull(TEXT("connect reply carries negotiation timeout"),
				Reply.Limits.FindMapValue(TEXT("data_negotiation_timeout_s")));
			Status = FURLabRequestEnvelopeCodec::EncodeConnectReply(Reply, Encoded);
			TestTrue(TEXT("connect reply encodes"), Status.IsOk());
			TestExact(TEXT("connect reply"), Encoded, Packed);
		}
	}

	if (LoadMessage(TEXT("connect_reply_shm"), Packed))
	{
		FURLabConnectReply Reply;
		Status = FURLabRequestEnvelopeCodec::DecodeConnectReply(Packed, Reply);
		TestTrue(TEXT("SHM connect reply decodes"), Status.IsOk());
		if (Status)
		{
			TestEqual(TEXT("SHM negotiation transfer"), Reply.DataNegotiation.Transfer,
				FString(TEXT("shm")));
			TestEqual(TEXT("SHM negotiation generation"),
				Reply.DataNegotiation.RegionGeneration, uint64(7));
			TestEqual(TEXT("SHM negotiation proof length"),
				Reply.DataNegotiation.ChallengeByteLength, int64(32));
			Status = FURLabRequestEnvelopeCodec::EncodeConnectReply(Reply, Encoded);
			TestTrue(TEXT("SHM connect reply encodes"), Status.IsOk());
			TestExact(TEXT("connect_reply_shm"), Encoded, Packed);
		}
	}

	if (LoadMessage(TEXT("disconnect_request"), Packed))
	{
		FURLabDisconnectRequest Request;
		Status = FURLabRequestEnvelopeCodec::DecodeDisconnectRequest(Packed, Request);
		TestTrue(TEXT("disconnect request decodes"), Status.IsOk());
		if (Status)
		{
			Status = FURLabRequestEnvelopeCodec::EncodeDisconnectRequest(Request, Encoded);
			TestTrue(TEXT("disconnect request encodes"), Status.IsOk());
			TestExact(TEXT("disconnect request"), Encoded, Packed);
		}
	}

	if (LoadMessage(TEXT("disconnect_reply"), Packed))
	{
		FURLabDisconnectReply Reply;
		Status = FURLabRequestEnvelopeCodec::DecodeDisconnectReply(Packed, Reply);
		TestTrue(TEXT("disconnect reply decodes"), Status.IsOk());
		if (Status)
		{
			Status = FURLabRequestEnvelopeCodec::EncodeDisconnectReply(Reply, Encoded);
			TestTrue(TEXT("disconnect reply encodes"), Status.IsOk());
			TestExact(TEXT("disconnect reply"), Encoded, Packed);
		}
	}

	if (LoadMessage(TEXT("request"), Packed))
	{
		FURLabRequestEnvelope Request;
		Status = FURLabRequestEnvelopeCodec::Decode(Packed, Request);
		TestTrue(TEXT("regular request decodes"), Status.IsOk());
		if (Status)
		{
			Status = FURLabRequestEnvelopeCodec::Encode(Request, Encoded);
			TestTrue(TEXT("regular request encodes"), Status.IsOk());
			TestExact(TEXT("regular request"), Encoded, Packed);
		}
	}

	if (LoadMessage(TEXT("data_select_request_shm"), Packed))
	{
		FURLabRequestEnvelope Request;
		Status = FURLabRequestEnvelopeCodec::Decode(Packed, Request);
		TestTrue(TEXT("SHM selection request decodes"), Status.IsOk());
		if (Status)
		{
			TestTrue(TEXT("SHM selection body has exact generated shape"),
				FURLabOperationSchema::ValidateRequestBody(Request.Op, Request.Body).IsOk());
			Status = FURLabRequestEnvelopeCodec::Encode(Request, Encoded);
			TestTrue(TEXT("SHM selection request encodes"), Status.IsOk());
			TestExact(TEXT("data_select_request_shm"), Encoded, Packed);
		}
	}

	for (const TCHAR* Name : {TEXT("result_reply"), TEXT("pending_reply"), TEXT("error_reply")})
	{
		if (!LoadMessage(Name, Packed)) continue;
		FURLabReplyEnvelope Reply;
		Status = FURLabRequestEnvelopeCodec::DecodeReply(Packed, Reply);
		TestTrue(*FString::Printf(TEXT("%s decodes"), Name), Status.IsOk());
		if (!Status) continue;
		Status = FURLabRequestEnvelopeCodec::EncodeReply(Reply, Encoded);
		TestTrue(*FString::Printf(TEXT("%s encodes"), Name), Status.IsOk());
		TestExact(Name, Encoded, Packed);
	}

	if (LoadMessage(TEXT("protocol_error"), Packed))
	{
		FURLabProtocolError Error;
		Status = FURLabRequestEnvelopeCodec::DecodeProtocolError(Packed, Error);
		TestTrue(TEXT("protocol error decodes"), Status.IsOk());
		if (Status)
		{
			Status = FURLabRequestEnvelopeCodec::EncodeProtocolError(Error, Encoded);
			TestTrue(TEXT("protocol error encodes"), Status.IsOk());
			TestExact(TEXT("protocol error"), Encoded, Packed);
		}
	}

	if (LoadMessage(TEXT("value"), Packed))
	{
		FURLabWireValue Value;
		Status = FURLabRequestEnvelopeCodec::DecodeValue(Packed, Value);
		TestTrue(TEXT("generic value decodes"), Status.IsOk());
		if (Status)
		{
			const TCHAR NulKeyChars[] = {'n', 'u', 'l', 0, 'k', 'e', 'y'};
			const FString NulKey = FString::ConstructFromPtrSize(
				NulKeyChars, UE_ARRAY_COUNT(NulKeyChars));
			const FURLabWireValue* NulValue = Value.FindMapValue(NulKey);
			TestTrue(TEXT("generic value preserves embedded-NUL key and suffix"),
				NulValue && NulValue->GetType() == EURLabWireValueType::String
				&& NulValue->GetString().Len() == 12 && NulValue->GetString()[6] == 0
				&& NulValue->GetString()[7] == 'v');
			Status = FURLabRequestEnvelopeCodec::EncodeValue(Value, Encoded);
			TestTrue(TEXT("generic value encodes"), Status.IsOk());
			TestExact(TEXT("generic value"), Encoded, Packed);
		}
	}

	TArray<uint8> Seeded{7, 8, 9};
	FURLabWireLimits Tiny;
	Tiny.MaxOutputBytes = 1;
	Status = FURLabRequestEnvelopeCodec::EncodeValue(
		FURLabWireValue::MakeString(TEXT("too large")), Seeded, Tiny);
	TestEqual(TEXT("generic encoder reports output bound"), Status.Code,
		EURLabWireError::OutputTooLarge);
	TestTrue(TEXT("generic encoder leaves output unchanged"), Seeded == TArray<uint8>{7, 8, 9});
	return true;
}
