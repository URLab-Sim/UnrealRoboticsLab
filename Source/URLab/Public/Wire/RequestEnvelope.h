// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Containers/ArrayView.h"
#include "Wire/RequestDigest.h"

enum class EURLabWireValueType : uint8
{
	Null,
	Bool,
	Int64,
	Float64,
	String,
	Binary,
	Array,
	Map
};

/** Owned MessagePack value. Maps retain entry order and cannot erase duplicates. */
class URLAB_API FURLabWireValue
{
public:
	static FURLabWireValue MakeNull();
	static FURLabWireValue MakeBool(bool Value);
	static FURLabWireValue MakeInt(int64 Value);
	static FURLabWireValue MakeFloat(double Value);
	static FURLabWireValue MakeString(FString Value);
	static FURLabWireValue MakeBinary(TArray<uint8> Value);
	static FURLabWireValue MakeArray(TArray<FURLabWireValue> Value);
	static FURLabWireValue MakeMap(TArray<FString> Keys, TArray<FURLabWireValue> Values);

	EURLabWireValueType GetType() const { return Type; }
	bool GetBool() const;
	int64 GetInt() const;
	double GetFloat() const;
	const FString& GetString() const;
	const TArray<uint8>& GetBinary() const;
	const TArray<FURLabWireValue>& GetArray() const;
	int32 GetMapSize() const;
	bool HasValidMapStorage() const;
	const FString& GetMapKey(int32 Index) const;
	const FURLabWireValue& GetMapValue(int32 Index) const;
	const FURLabWireValue* FindMapValue(const FString& Key) const;

private:
	EURLabWireValueType Type = EURLabWireValueType::Null;
	bool BoolValue = false;
	int64 IntValue = 0;
	double FloatValue = 0.0;
	FString StringValue;
	TArray<uint8> BinaryValue;
	TArray<FURLabWireValue> ArrayValue;
	TArray<FString> MapKeys;
	TArray<FURLabWireValue> MapValues;

	friend class FURLabRequestEnvelopeCodec;
};

struct URLAB_API FURLabRequestEnvelope
{
	int32 ProtocolVersion = 0;
	FString BootId;
	FString SessionId;
	uint64 RequestId = 0;
	FString Op;
	FURLabWireValue Body;
};

enum class EURLabReplyKind : uint8
{
	Result,
	Pending,
	Error
};

enum class EURLabExecution : uint8
{
	Rejected,
	Terminal,
	Unknown
};

struct URLAB_API FURLabReplyError
{
	FString Code;
	FString Message;
	EURLabExecution Execution = EURLabExecution::Rejected;
};

struct URLAB_API FURLabReplyEnvelope
{
	int32 ProtocolVersion = 0;
	FString BootId;
	FString SessionId;
	uint64 RequestId = 0;
	FString Op;
	EURLabReplyKind Kind = EURLabReplyKind::Pending;
	FURLabWireValue Result;
	FURLabReplyError Error;

	static FURLabReplyEnvelope MakeResult(const FURLabRequestEnvelope& Request,
		FURLabWireValue Result);
	static FURLabReplyEnvelope MakePending(const FURLabRequestEnvelope& Request);
	static FURLabReplyEnvelope MakeError(const FURLabRequestEnvelope& Request,
		FString Code, FString Message, EURLabExecution Execution);
};

struct URLAB_API FURLabConnectRequest
{
	int32 ProtocolVersion = 0;
	FString DataPolicy = TEXT("network");
};

struct URLAB_API FURLabDataNegotiation
{
	FString Transfer = TEXT("network");
	FString RegionId;
	uint64 RegionGeneration = 0;
	FString DirectoryPath;
	FString ChallengeName;
	int64 ChallengeByteLength = 0;
	double ExpiresInSeconds = 0.0;
};

struct URLAB_API FURLabConnectReply
{
	int32 ProtocolVersion = 0;
	FString InstanceId;
	FString BootId;
	FString SessionId;
	FString URLabVersion;
	TArray<FString> CommandEndpoints;
	TArray<FString> Services;
	TArray<FString> Permissions;
	TArray<FString> DataOffers;
	FURLabDataNegotiation DataNegotiation;
	FURLabWireValue Limits;
};

struct URLAB_API FURLabDisconnectRequest
{
	int32 ProtocolVersion = 0;
	FString BootId;
	FString SessionId;
};

struct URLAB_API FURLabDisconnectReply
{
	int32 ProtocolVersion = 0;
	FString BootId;
	FString SessionId;
	bool bDisconnected = false;
};

struct URLAB_API FURLabProtocolError
{
	FString Code;
	FString Message;
};

struct URLAB_API FURLabWireLimits
{
	int32 MaxInputBytes = 4 * 1024 * 1024;
	int32 MaxDepth = 64;
	int32 MaxContainerItems = 65'536;
	int32 MaxOutputBytes = 4 * 1024 * 1024;
	int32 MaxIdentifierBytes = 256;
};

enum class EURLabWireError : uint8
{
	None,
	InputTooLarge,
	OutputTooLarge,
	Truncated,
	InvalidMsgpack,
	TrailingData,
	DepthLimit,
	ContainerLimit,
	DuplicateKey,
	InvalidUtf8,
	NonStringMapKey,
	UnsupportedType,
	IntegerOutOfRange,
	NonFiniteFloat,
	EnvelopeKeys,
	EnvelopeFieldType,
	EnvelopeFieldValue,
	InvalidValue
};

struct URLAB_API FURLabWireStatus
{
	EURLabWireError Code = EURLabWireError::None;
	int32 ByteOffset = INDEX_NONE;
	FString Detail;

	bool IsOk() const { return Code == EURLabWireError::None; }
	explicit operator bool() const { return IsOk(); }

	static FURLabWireStatus Ok() { return {}; }
	static FURLabWireStatus Error(EURLabWireError Code, FString Detail,
		int32 ByteOffset = INDEX_NONE);
};

class URLAB_API FURLabRequestEnvelopeCodec
{
public:
	/** Decode/encode one complete bounded MessagePack value. Outputs are transactional. */
	static FURLabWireStatus DecodeValue(TConstArrayView<uint8> Bytes,
		FURLabWireValue& OutValue, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus EncodeValue(const FURLabWireValue& Value,
		TArray<uint8>& OutBytes, const FURLabWireLimits& Limits = {});

	/** Decode raw UTF-8 protobuf text using the same identifier rules as envelope op. */
	static FURLabWireStatus DecodeIdentifierUtf8(TConstArrayView<uint8> Bytes,
		FString& OutIdentifier, const FURLabWireLimits& Limits = {});

	/** Decode one complete MessagePack object. OutEnvelope is unchanged on failure. */
	static FURLabWireStatus Decode(TConstArrayView<uint8> Bytes,
		FURLabRequestEnvelope& OutEnvelope, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus Encode(const FURLabRequestEnvelope& Envelope,
		TArray<uint8>& OutBytes, const FURLabWireLimits& Limits = {});

	static FURLabWireStatus DecodeReply(TConstArrayView<uint8> Bytes,
		FURLabReplyEnvelope& OutReply, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus EncodeReply(const FURLabReplyEnvelope& Reply,
		TArray<uint8>& OutBytes, const FURLabWireLimits& Limits = {});

	static FURLabWireStatus DecodeConnectRequest(TConstArrayView<uint8> Bytes,
		FURLabConnectRequest& OutRequest, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus EncodeConnectRequest(const FURLabConnectRequest& Request,
		TArray<uint8>& OutBytes, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus DecodeConnectReply(TConstArrayView<uint8> Bytes,
		FURLabConnectReply& OutReply, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus EncodeConnectReply(const FURLabConnectReply& Reply,
		TArray<uint8>& OutBytes, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus DecodeDisconnectRequest(TConstArrayView<uint8> Bytes,
		FURLabDisconnectRequest& OutRequest, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus EncodeDisconnectRequest(const FURLabDisconnectRequest& Request,
		TArray<uint8>& OutBytes, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus DecodeDisconnectReply(TConstArrayView<uint8> Bytes,
		FURLabDisconnectReply& OutReply, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus EncodeDisconnectReply(const FURLabDisconnectReply& Reply,
		TArray<uint8>& OutBytes, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus DecodeProtocolError(TConstArrayView<uint8> Bytes,
		FURLabProtocolError& OutError, const FURLabWireLimits& Limits = {});
	static FURLabWireStatus EncodeProtocolError(const FURLabProtocolError& Error,
		TArray<uint8>& OutBytes, const FURLabWireLimits& Limits = {});

	/** Canonical map {"body": Body, "op": Op}, excluding routing identity. */
	static FURLabWireStatus CanonicalBytes(const FString& Op, const FURLabWireValue& Body,
		TArray<uint8>& OutBytes, const FURLabWireLimits& Limits = {});

	static FURLabWireStatus Digest(const FString& Op, const FURLabWireValue& Body,
		FURLabRequestDigest& OutDigest, const FURLabWireLimits& Limits = {});

	/** Stable snake-case spelling used by shared rejection fixtures. */
	static const TCHAR* ErrorCodeString(EURLabWireError Code);
};
