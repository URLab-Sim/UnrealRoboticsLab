// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Wire/RequestEnvelope.h"

#include "Algo/Sort.h"
#include "Containers/StringConv.h"
#include "Math/UnrealMathUtility.h"
#include "Misc/Crc.h"
#include "Utils/AssetCache.h"

#include <initializer_list>

namespace
{
FURLabWireStatus MakeError(EURLabWireError Code, const TCHAR* Detail, int32 Offset = INDEX_NONE)
{
	return FURLabWireStatus::Error(Code, FString(Detail), Offset);
}

FURLabWireStatus ValidateLimits(const FURLabWireLimits& Limits)
{
	if (Limits.MaxInputBytes <= 0 || Limits.MaxDepth <= 0
		|| Limits.MaxContainerItems <= 0 || Limits.MaxOutputBytes <= 0
		|| Limits.MaxIdentifierBytes <= 0)
	{
		return MakeError(EURLabWireError::InvalidValue,
			TEXT("wire limits must all be positive"));
	}
	return FURLabWireStatus::Ok();
}

bool IsContinuation(uint8 Byte)
{
	return (Byte & 0xC0) == 0x80;
}

/** Strict RFC 3629 validation: no overlong forms, surrogates, or > U+10ffff. */
bool IsValidUtf8(TConstArrayView<uint8> Bytes, int32& OutBadOffset)
{
	for (int32 Index = 0; Index < Bytes.Num();)
	{
		const uint8 Lead = Bytes[Index];
		if (Lead <= 0x7F)
		{
			++Index;
			continue;
		}

		int32 Width = 0;
		if (Lead >= 0xC2 && Lead <= 0xDF)
		{
			Width = 2;
		}
		else if (Lead >= 0xE0 && Lead <= 0xEF)
		{
			Width = 3;
		}
		else if (Lead >= 0xF0 && Lead <= 0xF4)
		{
			Width = 4;
		}
		else
		{
			OutBadOffset = Index;
			return false;
		}

		if (Width > Bytes.Num() - Index)
		{
			OutBadOffset = Index;
			return false;
		}
		for (int32 Continuation = 1; Continuation < Width; ++Continuation)
		{
			if (!IsContinuation(Bytes[Index + Continuation]))
			{
				OutBadOffset = Index + Continuation;
				return false;
			}
		}

		const uint8 Second = Bytes[Index + 1];
		if ((Lead == 0xE0 && Second < 0xA0)
			|| (Lead == 0xED && Second > 0x9F)
			|| (Lead == 0xF0 && Second < 0x90)
			|| (Lead == 0xF4 && Second > 0x8F))
		{
			OutBadOffset = Index;
			return false;
		}
		Index += Width;
	}
	return true;
}

bool HasValidUnicode(const FString& Value)
{
	for (int32 Index = 0; Index < Value.Len(); ++Index)
	{
		const uint32 Unit = static_cast<uint32>(Value[Index]);
		if constexpr (sizeof(TCHAR) == 2)
		{
			if (Unit >= 0xD800 && Unit <= 0xDBFF)
			{
				if (++Index >= Value.Len())
					return false;
				const uint32 Low = static_cast<uint32>(Value[Index]);
				if (Low < 0xDC00 || Low > 0xDFFF)
					return false;
			}
			else if (Unit >= 0xDC00 && Unit <= 0xDFFF)
			{
				return false;
			}
		}
		else if (Unit > 0x10FFFF || (Unit >= 0xD800 && Unit <= 0xDFFF))
		{
			return false;
		}
	}
	return true;
}

bool StringToUtf8(const FString& Value, TArray<uint8>& OutBytes)
{
	if (!HasValidUnicode(Value))
		return false;
	// Explicit-length TStringConversion treats a final NUL as a terminator.
	// A non-NUL guard keeps every FString code unit, including a trailing NUL.
	FString Guarded = Value;
	Guarded.AppendChar(1);
	FTCHARToUTF8 Converted(*Guarded, Guarded.Len());
	const int32 PayloadLength = Converted.Length() - 1;
	OutBytes.Reset(PayloadLength);
	if (PayloadLength > 0)
	{
		OutBytes.Append(reinterpret_cast<const uint8*>(Converted.Get()), PayloadLength);
	}
	return true;
}

bool IsControlCharacter(TCHAR Character)
{
	const uint32 Value = static_cast<uint32>(Character);
	return Value <= 0x1F || (Value >= 0x7F && Value <= 0x9F);
}

bool ExactStringEquals(const FString& A, const FString& B)
{
	return A.Len() == B.Len()
		&& (A.IsEmpty() || FMemory::Memcmp(*A, *B, A.Len() * sizeof(TCHAR)) == 0);
}

struct FExactStringKeyFuncs : BaseKeyFuncs<FString, FString>
{
	static const FString& GetSetKey(const FString& Element)
	{
		return Element;
	}

	static bool Matches(const FString& A, const FString& B)
	{
		return ExactStringEquals(A, B);
	}

	static uint32 GetKeyHash(const FString& Key)
	{
		return FCrc::MemCrc32(*Key, Key.Len() * sizeof(TCHAR));
	}
};

FURLabWireStatus ValidateIdentifier(const FString& Value, const TCHAR* Field,
	const FURLabWireLimits& Limits)
{
	if (Value.Len() > Limits.MaxIdentifierBytes)
	{
		return FURLabWireStatus::Error(EURLabWireError::EnvelopeFieldValue,
			FString::Printf(TEXT("%s must contain 1..%d UTF-8 bytes"), Field,
				Limits.MaxIdentifierBytes));
	}
	TArray<uint8> Utf8;
	if (!StringToUtf8(Value, Utf8))
		return FURLabWireStatus::Error(EURLabWireError::InvalidUtf8,
			FString::Printf(TEXT("%s is not valid Unicode"), Field));
	if (Utf8.IsEmpty() || Utf8.Num() > Limits.MaxIdentifierBytes)
	{
		return FURLabWireStatus::Error(EURLabWireError::EnvelopeFieldValue,
			FString::Printf(TEXT("%s must contain 1..%d UTF-8 bytes"), Field,
				Limits.MaxIdentifierBytes));
	}
	for (const TCHAR Character : Value)
	{
		if (IsControlCharacter(Character))
		{
			return FURLabWireStatus::Error(EURLabWireError::EnvelopeFieldValue,
				FString::Printf(TEXT("%s must not contain control characters"), Field));
		}
	}
	return FURLabWireStatus::Ok();
}

class FBoundedMsgpackParser
{
public:
	FBoundedMsgpackParser(TConstArrayView<uint8> InBytes, const FURLabWireLimits& InLimits)
		: Bytes(InBytes), Limits(InLimits)
	{
	}

	FURLabWireStatus Parse(FURLabWireValue& OutValue)
	{
		FURLabWireValue Local;
		if (!ParseValue(0, Local))
			return Status;
		if (Offset != Bytes.Num())
			return MakeError(EURLabWireError::TrailingData,
				TEXT("request contains a trailing MessagePack object"), Offset);
		OutValue = MoveTemp(Local);
		return FURLabWireStatus::Ok();
	}

private:
	bool Fail(EURLabWireError Code, const TCHAR* Detail, int32 At = INDEX_NONE)
	{
		if (Status.IsOk())
			Status = MakeError(Code, Detail, At == INDEX_NONE ? Offset : At);
		return false;
	}

	bool ReadByte(uint8& Out)
	{
		if (Offset >= Bytes.Num())
			return Fail(EURLabWireError::Truncated, TEXT("MessagePack value is truncated"));
		Out = Bytes[Offset++];
		return true;
	}

	bool ReadUnsigned(int32 Width, uint64& Out)
	{
		if (Width < 0 || Width > Bytes.Num() - Offset)
			return Fail(EURLabWireError::Truncated, TEXT("MessagePack value is truncated"));
		Out = 0;
		for (int32 Index = 0; Index < Width; ++Index)
			Out = (Out << 8) | Bytes[Offset++];
		return true;
	}

	bool ReadSigned(int32 Width, int64& Out)
	{
		uint64 Raw = 0;
		if (!ReadUnsigned(Width, Raw))
			return false;
		const int32 Bits = Width * 8;
		const uint64 Sign = uint64(1) << (Bits - 1);
		if ((Raw & Sign) == 0)
		{
			Out = static_cast<int64>(Raw);
		}
		else
		{
			const uint64 Mask = Width == 8 ? MAX_uint64 : ((uint64(1) << Bits) - 1);
			Out = -1 - static_cast<int64>((~Raw) & Mask);
		}
		return true;
	}

	bool ChargeContainer(uint64 Count, int32 Depth)
	{
		if (Depth > Limits.MaxDepth)
			return Fail(EURLabWireError::DepthLimit,
				TEXT("MessagePack nesting depth exceeds the limit"));
		if (Count > static_cast<uint64>(Limits.MaxContainerItems)
			|| Count > static_cast<uint64>(Limits.MaxContainerItems - ContainerItems))
		{
			return Fail(EURLabWireError::ContainerLimit,
				TEXT("MessagePack container items exceed the limit"));
		}
		ContainerItems += static_cast<int32>(Count);
		return true;
	}

	bool ReadString(uint64 Count, FString& Out)
	{
		if (Count > static_cast<uint64>(Bytes.Num() - Offset))
			return Fail(EURLabWireError::Truncated, TEXT("MessagePack string is truncated"));
		const int32 Start = Offset;
		const int32 Size = static_cast<int32>(Count);
		const TConstArrayView<uint8> Utf8(Bytes.GetData() + Offset, Size);
		int32 BadOffset = INDEX_NONE;
		if (!IsValidUtf8(Utf8, BadOffset))
			return Fail(EURLabWireError::InvalidUtf8, TEXT("string is not valid UTF-8"),
				Start + BadOffset);
		if (Size == 0)
		{
			Out.Reset();
		}
		else
		{
			TArray<ANSICHAR> Guarded;
			Guarded.Reserve(Size + 1);
			Guarded.Append(reinterpret_cast<const ANSICHAR*>(Utf8.GetData()), Size);
			Guarded.Add(1);
			FUTF8ToTCHAR Converted(Guarded.GetData(), Guarded.Num());
			Out = FString::ConstructFromPtrSize(Converted.Get(), Converted.Length() - 1);
		}
		Offset += Size;
		return true;
	}

	bool ReadBinary(uint64 Count, TArray<uint8>& Out)
	{
		if (Count > static_cast<uint64>(Bytes.Num() - Offset))
			return Fail(EURLabWireError::Truncated, TEXT("MessagePack binary value is truncated"));
		const int32 Size = static_cast<int32>(Count);
		Out.Reset(Size);
		if (Size > 0)
			Out.Append(Bytes.GetData() + Offset, Size);
		Offset += Size;
		return true;
	}

	bool ParseArray(uint64 Count, int32 ParentDepth, FURLabWireValue& Out)
	{
		const int32 Depth = ParentDepth + 1;
		if (!ChargeContainer(Count, Depth))
			return false;
		TArray<FURLabWireValue> Values;
		Values.Reserve(static_cast<int32>(Count));
		for (uint64 Index = 0; Index < Count; ++Index)
		{
			FURLabWireValue Value;
			if (!ParseValue(Depth, Value))
				return false;
			Values.Add(MoveTemp(Value));
		}
		Out = FURLabWireValue::MakeArray(MoveTemp(Values));
		return true;
	}

	bool ParseMap(uint64 Count, int32 ParentDepth, FURLabWireValue& Out)
	{
		const int32 Depth = ParentDepth + 1;
		if (!ChargeContainer(Count, Depth))
			return false;
		TArray<FString> Keys;
		TArray<FURLabWireValue> Values;
		TSet<FString, FExactStringKeyFuncs, FDefaultSetAllocator> Seen;
		Keys.Reserve(static_cast<int32>(Count));
		Values.Reserve(static_cast<int32>(Count));
		Seen.Reserve(static_cast<int32>(Count));
		for (uint64 Index = 0; Index < Count; ++Index)
		{
			const int32 KeyOffset = Offset;
			FURLabWireValue Key;
			if (!ParseValue(Depth, Key))
				return false;
			if (Key.GetType() != EURLabWireValueType::String)
				return Fail(EURLabWireError::NonStringMapKey,
					TEXT("MessagePack map keys must be strings"), KeyOffset);
			if (Seen.Contains(Key.GetString()))
				return Fail(EURLabWireError::DuplicateKey,
					TEXT("MessagePack map contains a duplicate key"), KeyOffset);
			Seen.Add(Key.GetString());

			FURLabWireValue Value;
			if (!ParseValue(Depth, Value))
				return false;
			Keys.Add(Key.GetString());
			Values.Add(MoveTemp(Value));
		}
		Out = FURLabWireValue::MakeMap(MoveTemp(Keys), MoveTemp(Values));
		return true;
	}

	bool ParseValue(int32 ParentDepth, FURLabWireValue& Out)
	{
		const int32 TagOffset = Offset;
		uint8 Tag = 0;
		if (!ReadByte(Tag))
			return false;
		if (Tag <= 0x7F)
		{
			Out = FURLabWireValue::MakeInt(Tag);
			return true;
		}
		if (Tag >= 0xE0)
		{
			Out = FURLabWireValue::MakeInt(static_cast<int8>(Tag));
			return true;
		}
		if (Tag >= 0xA0 && Tag <= 0xBF)
		{
			FString Value;
			if (!ReadString(Tag & 0x1F, Value)) return false;
			Out = FURLabWireValue::MakeString(MoveTemp(Value));
			return true;
		}
		if (Tag >= 0x90 && Tag <= 0x9F)
			return ParseArray(Tag & 0x0F, ParentDepth, Out);
		if (Tag >= 0x80 && Tag <= 0x8F)
			return ParseMap(Tag & 0x0F, ParentDepth, Out);

		switch (Tag)
		{
		case 0xC0: Out = FURLabWireValue::MakeNull(); return true;
		case 0xC2: Out = FURLabWireValue::MakeBool(false); return true;
		case 0xC3: Out = FURLabWireValue::MakeBool(true); return true;
		case 0xCA:
		{
			uint64 Raw64 = 0;
			if (!ReadUnsigned(4, Raw64)) return false;
			const uint32 Raw = static_cast<uint32>(Raw64);
			float Value = 0.0f;
			FMemory::Memcpy(&Value, &Raw, sizeof(Value));
			if (!FMath::IsFinite(Value))
				return Fail(EURLabWireError::NonFiniteFloat,
					TEXT("floating-point values must be finite"), TagOffset);
			Out = FURLabWireValue::MakeFloat(static_cast<double>(Value));
			return true;
		}
		case 0xCB:
		{
			uint64 Raw = 0;
			if (!ReadUnsigned(8, Raw)) return false;
			double Value = 0.0;
			FMemory::Memcpy(&Value, &Raw, sizeof(Value));
			if (!FMath::IsFinite(Value))
				return Fail(EURLabWireError::NonFiniteFloat,
					TEXT("floating-point values must be finite"), TagOffset);
			Out = FURLabWireValue::MakeFloat(Value);
			return true;
		}
		case 0xCC: case 0xCD: case 0xCE: case 0xCF:
		{
			const int32 Width = Tag == 0xCC ? 1 : Tag == 0xCD ? 2 : Tag == 0xCE ? 4 : 8;
			uint64 Value = 0;
			if (!ReadUnsigned(Width, Value)) return false;
			if (Value > static_cast<uint64>(MAX_int64))
				return Fail(EURLabWireError::IntegerOutOfRange,
					TEXT("integer is outside signed 64-bit range"), TagOffset);
			Out = FURLabWireValue::MakeInt(static_cast<int64>(Value));
			return true;
		}
		case 0xD0: case 0xD1: case 0xD2: case 0xD3:
		{
			const int32 Width = Tag == 0xD0 ? 1 : Tag == 0xD1 ? 2 : Tag == 0xD2 ? 4 : 8;
			int64 Value = 0;
			if (!ReadSigned(Width, Value)) return false;
			Out = FURLabWireValue::MakeInt(Value);
			return true;
		}
		case 0xC4: case 0xC5: case 0xC6:
		{
			const int32 Width = Tag == 0xC4 ? 1 : Tag == 0xC5 ? 2 : 4;
			uint64 Count = 0;
			if (!ReadUnsigned(Width, Count)) return false;
			TArray<uint8> Value;
			if (!ReadBinary(Count, Value)) return false;
			Out = FURLabWireValue::MakeBinary(MoveTemp(Value));
			return true;
		}
		case 0xD9: case 0xDA: case 0xDB:
		{
			const int32 Width = Tag == 0xD9 ? 1 : Tag == 0xDA ? 2 : 4;
			uint64 Count = 0;
			if (!ReadUnsigned(Width, Count)) return false;
			FString Value;
			if (!ReadString(Count, Value)) return false;
			Out = FURLabWireValue::MakeString(MoveTemp(Value));
			return true;
		}
		case 0xDC: case 0xDD:
		{
			uint64 Count = 0;
			if (!ReadUnsigned(Tag == 0xDC ? 2 : 4, Count)) return false;
			return ParseArray(Count, ParentDepth, Out);
		}
		case 0xDE: case 0xDF:
		{
			uint64 Count = 0;
			if (!ReadUnsigned(Tag == 0xDE ? 2 : 4, Count)) return false;
			return ParseMap(Count, ParentDepth, Out);
		}
		case 0xC1:
			return Fail(EURLabWireError::InvalidMsgpack,
				TEXT("reserved MessagePack tag 0xc1"), TagOffset);
		default:
			return Fail(EURLabWireError::UnsupportedType,
				TEXT("MessagePack extension types are unsupported"), TagOffset);
		}
	}

	TConstArrayView<uint8> Bytes;
	const FURLabWireLimits& Limits;
	int32 Offset = 0;
	int32 ContainerItems = 0;
	FURLabWireStatus Status;
};

class FBoundedMsgpackEncoder
{
public:
	explicit FBoundedMsgpackEncoder(const FURLabWireLimits& InLimits) : Limits(InLimits) {}

	FURLabWireStatus Encode(const FURLabWireValue& Value, TArray<uint8>& OutBytes)
	{
		if (!EncodeValue(Value, 0))
			return Status;
		OutBytes = MoveTemp(Output);
		return FURLabWireStatus::Ok();
	}

private:
	bool Fail(EURLabWireError Code, const TCHAR* Detail)
	{
		if (Status.IsOk()) Status = MakeError(Code, Detail);
		return false;
	}

	bool EnsureOutput(int64 Additional)
	{
		if (Additional < 0 || Additional > Limits.MaxOutputBytes - Output.Num())
			return Fail(EURLabWireError::OutputTooLarge,
				TEXT("MessagePack value exceeds the output byte limit"));
		return true;
	}

	bool AppendByte(uint8 Byte)
	{
		if (!EnsureOutput(1)) return false;
		Output.Add(Byte);
		return true;
	}

	bool AppendBytes(TConstArrayView<uint8> Bytes)
	{
		if (!EnsureOutput(Bytes.Num())) return false;
		Output.Append(Bytes.GetData(), Bytes.Num());
		return true;
	}

	bool AppendUnsigned(uint64 Value, int32 Width)
	{
		if (!EnsureOutput(Width)) return false;
		for (int32 Shift = (Width - 1) * 8; Shift >= 0; Shift -= 8)
			Output.Add(static_cast<uint8>(Value >> Shift));
		return true;
	}

	bool ChargeContainer(int32 Count, int32 Depth)
	{
		if (Depth > Limits.MaxDepth)
			return Fail(EURLabWireError::DepthLimit,
				TEXT("value nesting depth exceeds the limit"));
		if (Count < 0 || Count > Limits.MaxContainerItems - ContainerItems)
			return Fail(EURLabWireError::ContainerLimit,
				TEXT("container items exceed the total limit"));
		ContainerItems += Count;
		return true;
	}

	bool ConvertString(const FString& Value, TArray<uint8>& Utf8)
	{
		if (Value.Len() > Limits.MaxOutputBytes)
			return Fail(EURLabWireError::OutputTooLarge,
				TEXT("string exceeds the output byte limit"));
		if (!StringToUtf8(Value, Utf8))
			return Fail(EURLabWireError::InvalidUtf8, TEXT("string is not valid Unicode"));
		if (Utf8.Num() > Limits.MaxOutputBytes)
			return Fail(EURLabWireError::OutputTooLarge,
				TEXT("string exceeds the output byte limit"));
		return true;
	}

	bool AppendString(const FString& Value)
	{
		TArray<uint8> Utf8;
		if (!ConvertString(Value, Utf8)) return false;
		const uint64 Size = static_cast<uint64>(Utf8.Num());
		if (Size <= 31)
		{
			if (!AppendByte(static_cast<uint8>(0xA0 | Size))) return false;
		}
		else if (Size <= MAX_uint8)
		{
			if (!AppendByte(0xD9) || !AppendUnsigned(Size, 1)) return false;
		}
		else if (Size <= MAX_uint16)
		{
			if (!AppendByte(0xDA) || !AppendUnsigned(Size, 2)) return false;
		}
		else
		{
			if (!AppendByte(0xDB) || !AppendUnsigned(Size, 4)) return false;
		}
		return AppendBytes(Utf8);
	}

	bool AppendContainerHeader(int32 Count, uint8 FixBase, uint8 Tag16, uint8 Tag32)
	{
		if (Count <= 15) return AppendByte(static_cast<uint8>(FixBase | Count));
		if (Count <= MAX_uint16)
			return AppendByte(Tag16) && AppendUnsigned(static_cast<uint64>(Count), 2);
		return AppendByte(Tag32) && AppendUnsigned(static_cast<uint64>(Count), 4);
	}

	bool EncodeValue(const FURLabWireValue& Value, int32 ParentDepth)
	{
		switch (Value.GetType())
		{
		case EURLabWireValueType::Null:
			return AppendByte(0xC0);
		case EURLabWireValueType::Bool:
			return AppendByte(Value.GetBool() ? 0xC3 : 0xC2);
		case EURLabWireValueType::Int64:
		{
			const int64 Signed = Value.GetInt();
			if (Signed >= 0 && Signed <= 0x7F) return AppendByte(static_cast<uint8>(Signed));
			if (Signed >= -32 && Signed < 0) return AppendByte(static_cast<uint8>(Signed));
			uint64 Raw = 0;
			FMemory::Memcpy(&Raw, &Signed, sizeof(Raw));
			if (Signed >= 0)
			{
				if (Signed <= MAX_uint8) return AppendByte(0xCC) && AppendUnsigned(Raw, 1);
				if (Signed <= MAX_uint16) return AppendByte(0xCD) && AppendUnsigned(Raw, 2);
				if (Signed <= MAX_uint32) return AppendByte(0xCE) && AppendUnsigned(Raw, 4);
				return AppendByte(0xCF) && AppendUnsigned(Raw, 8);
			}
			if (Signed >= MIN_int8) return AppendByte(0xD0) && AppendUnsigned(Raw, 1);
			if (Signed >= MIN_int16) return AppendByte(0xD1) && AppendUnsigned(Raw, 2);
			if (Signed >= MIN_int32) return AppendByte(0xD2) && AppendUnsigned(Raw, 4);
			return AppendByte(0xD3) && AppendUnsigned(Raw, 8);
		}
		case EURLabWireValueType::Float64:
		{
			const double Number = Value.GetFloat();
			if (!FMath::IsFinite(Number))
				return Fail(EURLabWireError::NonFiniteFloat,
					TEXT("floating-point values must be finite"));
			uint64 Raw = 0;
			FMemory::Memcpy(&Raw, &Number, sizeof(Raw));
			return AppendByte(0xCB) && AppendUnsigned(Raw, 8);
		}
		case EURLabWireValueType::String:
			return AppendString(Value.GetString());
		case EURLabWireValueType::Binary:
		{
			const uint64 Size = static_cast<uint64>(Value.GetBinary().Num());
			if (Size <= MAX_uint8)
			{
				if (!AppendByte(0xC4) || !AppendUnsigned(Size, 1)) return false;
			}
			else if (Size <= MAX_uint16)
			{
				if (!AppendByte(0xC5) || !AppendUnsigned(Size, 2)) return false;
			}
			else if (!AppendByte(0xC6) || !AppendUnsigned(Size, 4)) return false;
			return AppendBytes(Value.GetBinary());
		}
		case EURLabWireValueType::Array:
		{
			const int32 Depth = ParentDepth + 1;
			if (!ChargeContainer(Value.GetArray().Num(), Depth)
				|| !AppendContainerHeader(Value.GetArray().Num(), 0x90, 0xDC, 0xDD))
				return false;
			for (const FURLabWireValue& Item : Value.GetArray())
				if (!EncodeValue(Item, Depth)) return false;
			return true;
		}
		case EURLabWireValueType::Map:
		{
			if (!Value.HasValidMapStorage())
				return Fail(EURLabWireError::InvalidValue,
					TEXT("programmatic map key/value counts differ"));
			const int32 Depth = ParentDepth + 1;
			if (!ChargeContainer(Value.GetMapSize(), Depth)
				|| !AppendContainerHeader(Value.GetMapSize(), 0x80, 0xDE, 0xDF))
				return false;
			TSet<FString, FExactStringKeyFuncs, FDefaultSetAllocator> Seen;
			Seen.Reserve(Value.GetMapSize());
			for (int32 Index = 0; Index < Value.GetMapSize(); ++Index)
			{
				const FString& Key = Value.GetMapKey(Index);
				if (Seen.Contains(Key))
					return Fail(EURLabWireError::DuplicateKey,
						TEXT("programmatic map contains a duplicate key"));
				Seen.Add(Key);
				if (!AppendString(Key) || !EncodeValue(Value.GetMapValue(Index), Depth))
					return false;
			}
			return true;
		}
		default:
			return Fail(EURLabWireError::UnsupportedType,
				TEXT("unsupported wire value type"));
		}
	}

	const FURLabWireLimits& Limits;
	int32 ContainerItems = 0;
	TArray<uint8> Output;
	FURLabWireStatus Status;
};

class FCanonicalEncoder
{
public:
	explicit FCanonicalEncoder(const FURLabWireLimits& InLimits) : Limits(InLimits) {}

	FURLabWireStatus EncodeRequest(const FString& Op, const FURLabWireValue& Body,
		TArray<uint8>& OutBytes)
	{
		if (Body.GetType() != EURLabWireValueType::Map)
			return MakeError(EURLabWireError::EnvelopeFieldType, TEXT("body must be a map"));
		Status = ValidateIdentifier(Op, TEXT("op"), Limits);
		if (!Status) return Status;

		if (!ChargeContainer(2, 1)) return Status;
		if (!AppendByte(0x08) || !AppendUnsigned(2)) return Status;
		if (!AppendStringKey(TEXT("body")) || !EncodeValue(Body, 1)) return Status;
		if (!AppendStringKey(TEXT("op")) || !EncodeString(Op)) return Status;
		OutBytes = MoveTemp(Output);
		return FURLabWireStatus::Ok();
	}

private:
	bool Fail(EURLabWireError Code, const TCHAR* Detail)
	{
		if (Status.IsOk()) Status = MakeError(Code, Detail);
		return false;
	}

	bool EnsureOutput(int64 Additional)
	{
		if (Additional < 0 || Additional > Limits.MaxOutputBytes - Output.Num())
			return Fail(EURLabWireError::OutputTooLarge,
				TEXT("canonical request exceeds the output byte limit"));
		return true;
	}

	bool AppendByte(uint8 Byte)
	{
		if (!EnsureOutput(1)) return false;
		Output.Add(Byte);
		return true;
	}

	bool AppendBytes(TConstArrayView<uint8> Bytes)
	{
		if (!EnsureOutput(Bytes.Num())) return false;
		Output.Append(Bytes.GetData(), Bytes.Num());
		return true;
	}

	bool AppendUnsigned(uint64 Value)
	{
		if (!EnsureOutput(8)) return false;
		for (int32 Shift = 56; Shift >= 0; Shift -= 8)
			Output.Add(static_cast<uint8>(Value >> Shift));
		return true;
	}

	bool AppendSigned(int64 Value)
	{
		uint64 Raw = 0;
		FMemory::Memcpy(&Raw, &Value, sizeof(Raw));
		return AppendUnsigned(Raw);
	}

	bool ChargeContainer(int32 Count, int32 Depth)
	{
		if (Depth > Limits.MaxDepth)
			return Fail(EURLabWireError::DepthLimit,
				TEXT("canonical value nesting depth exceeds the limit"));
		if (Count < 0 || Count > Limits.MaxContainerItems - ContainerItems)
			return Fail(EURLabWireError::ContainerLimit,
				TEXT("canonical container items exceed the limit"));
		ContainerItems += Count;
		return true;
	}

	bool ConvertString(const FString& Value, TArray<uint8>& Utf8)
	{
		if (Value.Len() > Limits.MaxOutputBytes)
			return Fail(EURLabWireError::OutputTooLarge,
				TEXT("string exceeds the canonical output limit"));
		if (!StringToUtf8(Value, Utf8))
			return Fail(EURLabWireError::InvalidUtf8, TEXT("string is not valid Unicode"));
		if (Utf8.Num() > Limits.MaxOutputBytes)
			return Fail(EURLabWireError::OutputTooLarge,
				TEXT("string exceeds the canonical output limit"));
		return true;
	}

	bool AppendEncodedString(TConstArrayView<uint8> Utf8)
	{
		return AppendByte(0x05) && AppendUnsigned(static_cast<uint64>(Utf8.Num()))
			&& AppendBytes(Utf8);
	}

	bool EncodeString(const FString& Value)
	{
		TArray<uint8> Utf8;
		return ConvertString(Value, Utf8) && AppendEncodedString(Utf8);
	}

	bool AppendStringKey(const TCHAR* Value)
	{
		return EncodeString(FString(Value));
	}

	struct FEncodedMapEntry
	{
		int32 Index = 0;
		TArray<uint8> Key;
	};

	static bool Utf8Less(const TArray<uint8>& A, const TArray<uint8>& B)
	{
		const int32 Common = FMath::Min(A.Num(), B.Num());
		for (int32 Index = 0; Index < Common; ++Index)
		{
			if (A[Index] != B[Index]) return A[Index] < B[Index];
		}
		return A.Num() < B.Num();
	}

	bool EncodeMap(const FURLabWireValue& Value, int32 ParentDepth)
	{
		if (!Value.HasValidMapStorage())
			return Fail(EURLabWireError::InvalidValue,
				TEXT("programmatic map key/value counts differ"));
		const int32 Depth = ParentDepth + 1;
		if (!ChargeContainer(Value.GetMapSize(), Depth)) return false;

		TArray<FEncodedMapEntry> Entries;
		Entries.Reserve(Value.GetMapSize());
		int64 KeyBudget = 0;
		for (int32 Index = 0; Index < Value.GetMapSize(); ++Index)
		{
			FEncodedMapEntry& Entry = Entries.AddDefaulted_GetRef();
			Entry.Index = Index;
			if (!ConvertString(Value.GetMapKey(Index), Entry.Key)) return false;
			KeyBudget += 9 + Entry.Key.Num();
			if (KeyBudget > Limits.MaxOutputBytes)
				return Fail(EURLabWireError::OutputTooLarge,
					TEXT("canonical map keys exceed the output byte limit"));
		}
		Algo::Sort(Entries, [](const FEncodedMapEntry& A, const FEncodedMapEntry& B)
		{
			return FCanonicalEncoder::Utf8Less(A.Key, B.Key);
		});
		for (int32 Index = 1; Index < Entries.Num(); ++Index)
		{
			if (Entries[Index - 1].Key == Entries[Index].Key)
				return Fail(EURLabWireError::DuplicateKey,
					TEXT("canonical map contains a duplicate key"));
		}

		if (!AppendByte(0x08) || !AppendUnsigned(static_cast<uint64>(Entries.Num())))
			return false;
		for (const FEncodedMapEntry& Entry : Entries)
		{
			if (!AppendEncodedString(Entry.Key)
				|| !EncodeValue(Value.GetMapValue(Entry.Index), Depth))
				return false;
		}
		return true;
	}

	bool EncodeValue(const FURLabWireValue& Value, int32 ParentDepth)
	{
		switch (Value.GetType())
		{
		case EURLabWireValueType::Null:
			return AppendByte(0x00);
		case EURLabWireValueType::Bool:
			return AppendByte(Value.GetBool() ? 0x02 : 0x01);
		case EURLabWireValueType::Int64:
			return AppendByte(0x03) && AppendSigned(Value.GetInt());
		case EURLabWireValueType::Float64:
		{
			if (!FMath::IsFinite(Value.GetFloat()))
				return Fail(EURLabWireError::NonFiniteFloat,
					TEXT("floating-point values must be finite"));
			const double Normalized = Value.GetFloat() == 0.0 ? 0.0 : Value.GetFloat();
			uint64 Raw = 0;
			FMemory::Memcpy(&Raw, &Normalized, sizeof(Raw));
			return AppendByte(0x04) && AppendUnsigned(Raw);
		}
		case EURLabWireValueType::String:
			return EncodeString(Value.GetString());
		case EURLabWireValueType::Binary:
			return AppendByte(0x06)
				&& AppendUnsigned(static_cast<uint64>(Value.GetBinary().Num()))
				&& AppendBytes(Value.GetBinary());
		case EURLabWireValueType::Array:
		{
			const int32 Depth = ParentDepth + 1;
			if (!ChargeContainer(Value.GetArray().Num(), Depth)
				|| !AppendByte(0x07)
				|| !AppendUnsigned(static_cast<uint64>(Value.GetArray().Num())))
				return false;
			for (const FURLabWireValue& Item : Value.GetArray())
			{
				if (!EncodeValue(Item, Depth)) return false;
			}
			return true;
		}
		case EURLabWireValueType::Map:
			return EncodeMap(Value, ParentDepth);
		default:
			return Fail(EURLabWireError::UnsupportedType,
				TEXT("unsupported canonical value type"));
		}
	}

	const FURLabWireLimits& Limits;
	int32 ContainerItems = 0;
	TArray<uint8> Output;
	FURLabWireStatus Status;
};

FURLabWireValue MakeMap(
	std::initializer_list<TPair<const TCHAR*, FURLabWireValue>> Entries)
{
	TArray<FString> Keys;
	TArray<FURLabWireValue> Values;
	Keys.Reserve(static_cast<int32>(Entries.size()));
	Values.Reserve(static_cast<int32>(Entries.size()));
	for (const TPair<const TCHAR*, FURLabWireValue>& Entry : Entries)
	{
		Keys.Add(Entry.Key);
		Values.Add(Entry.Value);
	}
	return FURLabWireValue::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

FURLabWireStatus RequireExactKeys(const FURLabWireValue& Value,
	std::initializer_list<const TCHAR*> Expected, const TCHAR* Label)
{
	if (Value.GetType() != EURLabWireValueType::Map)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			*FString::Printf(TEXT("%s must be a map"), Label));
	if (Value.GetMapSize() != static_cast<int32>(Expected.size()))
		return MakeError(EURLabWireError::EnvelopeKeys,
			*FString::Printf(TEXT("%s has unexpected keys"), Label));
	for (const TCHAR* Key : Expected)
		if (!Value.FindMapValue(Key))
			return MakeError(EURLabWireError::EnvelopeKeys,
				*FString::Printf(TEXT("%s has unexpected keys"), Label));
	return FURLabWireStatus::Ok();
}

FURLabWireStatus RequireProtocol2(const FURLabWireValue* Value)
{
	if (!Value || Value->GetType() != EURLabWireValueType::Int64)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			TEXT("protocol_version must be an integer"));
	if (Value->GetInt() != 2)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("protocol_version must equal 2"));
	return FURLabWireStatus::Ok();
}

FURLabWireStatus RequireLiteral(const FURLabWireValue* Value,
	const TCHAR* Field, const TCHAR* Literal)
{
	if (!Value || Value->GetType() != EURLabWireValueType::String)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			*FString::Printf(TEXT("%s must be a string"), Field));
	if (!ExactStringEquals(Value->GetString(), FString(Literal)))
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			*FString::Printf(TEXT("%s has an unexpected value"), Field));
	return FURLabWireStatus::Ok();
}

FURLabWireStatus ReadIdentifier(const FURLabWireValue* Value, const TCHAR* Field,
	FString& Out, const FURLabWireLimits& Limits)
{
	if (!Value || Value->GetType() != EURLabWireValueType::String)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			*FString::Printf(TEXT("%s must be a string"), Field));
	const FURLabWireStatus Status = ValidateIdentifier(Value->GetString(), Field, Limits);
	if (!Status) return Status;
	Out = Value->GetString();
	return FURLabWireStatus::Ok();
}

FURLabWireStatus ReadStringArray(const FURLabWireValue* Value, const TCHAR* Field,
	TArray<FString>& Out)
{
	if (!Value || Value->GetType() != EURLabWireValueType::Array)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			*FString::Printf(TEXT("%s must be an array of strings"), Field));
	TArray<FString> Local;
	Local.Reserve(Value->GetArray().Num());
	for (const FURLabWireValue& Item : Value->GetArray())
	{
		if (Item.GetType() != EURLabWireValueType::String)
			return MakeError(EURLabWireError::EnvelopeFieldType,
				*FString::Printf(TEXT("%s must be an array of strings"), Field));
		Local.Add(Item.GetString());
	}
	Out = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireValue MakeStringArray(const TArray<FString>& Values)
{
	TArray<FURLabWireValue> Items;
	Items.Reserve(Values.Num());
	for (const FString& Value : Values)
		Items.Add(FURLabWireValue::MakeString(Value));
	return FURLabWireValue::MakeArray(MoveTemp(Items));
}

bool IsDataPolicy(const FString& Value)
{
	return Value == TEXT("network") || Value == TEXT("auto") || Value == TEXT("shm");
}

FURLabWireStatus DecodeDataNegotiation(const FURLabWireValue* Value,
	FURLabDataNegotiation& Out, const FURLabWireLimits& Limits)
{
	if (!Value || Value->GetType() != EURLabWireValueType::Map)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			TEXT("data_negotiation must be a map"));
	const FURLabWireValue* Transfer = Value->FindMapValue(TEXT("transfer"));
	if (!Transfer || Transfer->GetType() != EURLabWireValueType::String)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			TEXT("data_negotiation.transfer must be a string"));
	FURLabDataNegotiation Local;
	Local.Transfer = Transfer->GetString();
	FURLabWireStatus Status;
	if (Local.Transfer == TEXT("network"))
	{
		if (!(Status = RequireExactKeys(*Value, {TEXT("transfer")},
			TEXT("network data negotiation")))) return Status;
	}
	else if (Local.Transfer == TEXT("shm"))
	{
		if (!(Status = RequireExactKeys(*Value,
			{TEXT("transfer"), TEXT("region_id"), TEXT("region_generation"),
			 TEXT("directory_path"), TEXT("challenge_name"),
			 TEXT("challenge_byte_length"), TEXT("expires_in_s")},
			TEXT("SHM data negotiation")))) return Status;
		if (!(Status = ReadIdentifier(Value->FindMapValue(TEXT("region_id")),
			TEXT("region_id"), Local.RegionId, Limits))) return Status;
		if (!(Status = ReadIdentifier(Value->FindMapValue(TEXT("directory_path")),
			TEXT("directory_path"), Local.DirectoryPath, Limits))) return Status;
		if (!(Status = ReadIdentifier(Value->FindMapValue(TEXT("challenge_name")),
			TEXT("challenge_name"), Local.ChallengeName, Limits))) return Status;
		const FURLabWireValue* Generation = Value->FindMapValue(TEXT("region_generation"));
		const FURLabWireValue* Length = Value->FindMapValue(TEXT("challenge_byte_length"));
		const FURLabWireValue* Expiry = Value->FindMapValue(TEXT("expires_in_s"));
		if (!Generation || Generation->GetType() != EURLabWireValueType::Int64
			|| Generation->GetInt() <= 0)
			return MakeError(EURLabWireError::EnvelopeFieldValue,
				TEXT("region_generation must be a positive int64"));
		if (!Length || Length->GetType() != EURLabWireValueType::Int64
			|| Length->GetInt() <= 0)
			return MakeError(EURLabWireError::EnvelopeFieldValue,
				TEXT("challenge_byte_length must be a positive int64"));
		if (!Expiry || Expiry->GetType() != EURLabWireValueType::Float64
			|| !FMath::IsFinite(Expiry->GetFloat()) || Expiry->GetFloat() <= 0.0)
			return MakeError(EURLabWireError::EnvelopeFieldValue,
				TEXT("expires_in_s must be a positive finite float64"));
		Local.RegionGeneration = static_cast<uint64>(Generation->GetInt());
		Local.ChallengeByteLength = Length->GetInt();
		Local.ExpiresInSeconds = Expiry->GetFloat();
	}
	else
	{
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("data_negotiation.transfer must be network or shm"));
	}
	Out = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus EncodeDataNegotiation(const FURLabDataNegotiation& Value,
	FURLabWireValue& Out, const FURLabWireLimits& Limits)
{
	if (Value.Transfer == TEXT("network"))
	{
		Out = MakeMap({{TEXT("transfer"), FURLabWireValue::MakeString(TEXT("network"))}});
		return FURLabWireStatus::Ok();
	}
	if (Value.Transfer != TEXT("shm") || Value.RegionGeneration == 0
		|| Value.RegionGeneration > static_cast<uint64>(MAX_int64)
		|| Value.ChallengeByteLength <= 0 || !FMath::IsFinite(Value.ExpiresInSeconds)
		|| Value.ExpiresInSeconds <= 0.0)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("invalid SHM data negotiation"));
	FURLabWireStatus Status = ValidateIdentifier(Value.RegionId, TEXT("region_id"), Limits);
	if (!Status) return Status;
	if (!(Status = ValidateIdentifier(Value.DirectoryPath, TEXT("directory_path"), Limits))) return Status;
	if (!(Status = ValidateIdentifier(Value.ChallengeName, TEXT("challenge_name"), Limits))) return Status;
	Out = MakeMap({
		{TEXT("transfer"), FURLabWireValue::MakeString(TEXT("shm"))},
		{TEXT("region_id"), FURLabWireValue::MakeString(Value.RegionId)},
		{TEXT("region_generation"), FURLabWireValue::MakeInt(static_cast<int64>(Value.RegionGeneration))},
		{TEXT("directory_path"), FURLabWireValue::MakeString(Value.DirectoryPath)},
		{TEXT("challenge_name"), FURLabWireValue::MakeString(Value.ChallengeName)},
		{TEXT("challenge_byte_length"), FURLabWireValue::MakeInt(Value.ChallengeByteLength)},
		{TEXT("expires_in_s"), FURLabWireValue::MakeFloat(Value.ExpiresInSeconds)}});
	return FURLabWireStatus::Ok();
}
} // namespace

FURLabWireValue FURLabWireValue::MakeNull()
{
	return {};
}

FURLabWireValue FURLabWireValue::MakeBool(bool Value)
{
	FURLabWireValue Result;
	Result.Type = EURLabWireValueType::Bool;
	Result.BoolValue = Value;
	return Result;
}

FURLabWireValue FURLabWireValue::MakeInt(int64 Value)
{
	FURLabWireValue Result;
	Result.Type = EURLabWireValueType::Int64;
	Result.IntValue = Value;
	return Result;
}

FURLabWireValue FURLabWireValue::MakeFloat(double Value)
{
	FURLabWireValue Result;
	Result.Type = EURLabWireValueType::Float64;
	Result.FloatValue = Value;
	return Result;
}

FURLabWireValue FURLabWireValue::MakeString(FString Value)
{
	FURLabWireValue Result;
	Result.Type = EURLabWireValueType::String;
	Result.StringValue = MoveTemp(Value);
	return Result;
}

FURLabWireValue FURLabWireValue::MakeBinary(TArray<uint8> Value)
{
	FURLabWireValue Result;
	Result.Type = EURLabWireValueType::Binary;
	Result.BinaryValue = MoveTemp(Value);
	return Result;
}

FURLabWireValue FURLabWireValue::MakeArray(TArray<FURLabWireValue> Value)
{
	FURLabWireValue Result;
	Result.Type = EURLabWireValueType::Array;
	Result.ArrayValue = MoveTemp(Value);
	return Result;
}

FURLabWireValue FURLabWireValue::MakeMap(TArray<FString> Keys, TArray<FURLabWireValue> Values)
{
	FURLabWireValue Result;
	Result.Type = EURLabWireValueType::Map;
	Result.MapKeys = MoveTemp(Keys);
	Result.MapValues = MoveTemp(Values);
	return Result;
}

bool FURLabWireValue::GetBool() const
{
	check(Type == EURLabWireValueType::Bool);
	return BoolValue;
}

int64 FURLabWireValue::GetInt() const
{
	check(Type == EURLabWireValueType::Int64);
	return IntValue;
}

double FURLabWireValue::GetFloat() const
{
	check(Type == EURLabWireValueType::Float64);
	return FloatValue;
}

const FString& FURLabWireValue::GetString() const
{
	check(Type == EURLabWireValueType::String);
	return StringValue;
}

const TArray<uint8>& FURLabWireValue::GetBinary() const
{
	check(Type == EURLabWireValueType::Binary);
	return BinaryValue;
}

const TArray<FURLabWireValue>& FURLabWireValue::GetArray() const
{
	check(Type == EURLabWireValueType::Array);
	return ArrayValue;
}

int32 FURLabWireValue::GetMapSize() const
{
	check(Type == EURLabWireValueType::Map);
	return MapKeys.Num();
}

bool FURLabWireValue::HasValidMapStorage() const
{
	return Type == EURLabWireValueType::Map && MapKeys.Num() == MapValues.Num();
}

const FString& FURLabWireValue::GetMapKey(int32 Index) const
{
	check(Type == EURLabWireValueType::Map && MapKeys.IsValidIndex(Index));
	return MapKeys[Index];
}

const FURLabWireValue& FURLabWireValue::GetMapValue(int32 Index) const
{
	check(Type == EURLabWireValueType::Map && MapValues.IsValidIndex(Index));
	return MapValues[Index];
}

const FURLabWireValue* FURLabWireValue::FindMapValue(const FString& Key) const
{
	check(Type == EURLabWireValueType::Map);
	for (int32 Index = 0; Index < MapKeys.Num(); ++Index)
	{
		if (ExactStringEquals(MapKeys[Index], Key))
			return MapValues.IsValidIndex(Index) ? &MapValues[Index] : nullptr;
	}
	return nullptr;
}

FURLabReplyEnvelope FURLabReplyEnvelope::MakeResult(
	const FURLabRequestEnvelope& Request, FURLabWireValue InResult)
{
	FURLabReplyEnvelope Reply;
	Reply.ProtocolVersion = Request.ProtocolVersion;
	Reply.BootId = Request.BootId;
	Reply.SessionId = Request.SessionId;
	Reply.RequestId = Request.RequestId;
	Reply.Op = Request.Op;
	Reply.Kind = EURLabReplyKind::Result;
	Reply.Result = MoveTemp(InResult);
	return Reply;
}

FURLabReplyEnvelope FURLabReplyEnvelope::MakePending(const FURLabRequestEnvelope& Request)
{
	FURLabReplyEnvelope Reply;
	Reply.ProtocolVersion = Request.ProtocolVersion;
	Reply.BootId = Request.BootId;
	Reply.SessionId = Request.SessionId;
	Reply.RequestId = Request.RequestId;
	Reply.Op = Request.Op;
	Reply.Kind = EURLabReplyKind::Pending;
	return Reply;
}

FURLabReplyEnvelope FURLabReplyEnvelope::MakeError(const FURLabRequestEnvelope& Request,
	FString Code, FString Message, EURLabExecution Execution)
{
	FURLabReplyEnvelope Reply;
	Reply.ProtocolVersion = Request.ProtocolVersion;
	Reply.BootId = Request.BootId;
	Reply.SessionId = Request.SessionId;
	Reply.RequestId = Request.RequestId;
	Reply.Op = Request.Op;
	Reply.Kind = EURLabReplyKind::Error;
	Reply.Error.Code = MoveTemp(Code);
	Reply.Error.Message = MoveTemp(Message);
	Reply.Error.Execution = Execution;
	return Reply;
}

FURLabWireStatus FURLabWireStatus::Error(EURLabWireError InCode, FString InDetail,
	int32 InByteOffset)
{
	FURLabWireStatus Result;
	Result.Code = InCode;
	Result.ByteOffset = InByteOffset;
	Result.Detail = MoveTemp(InDetail);
	return Result;
}

FString FURLabRequestDigest::ToHex() const
{
	static constexpr TCHAR Hex[] = TEXT("0123456789abcdef");
	FString Result;
	Result.Reserve(NumBytes * 2);
	for (const uint8 Byte : Bytes)
	{
		Result.AppendChar(Hex[Byte >> 4]);
		Result.AppendChar(Hex[Byte & 0x0F]);
	}
	return Result;
}

FURLabWireStatus FURLabRequestEnvelopeCodec::DecodeValue(TConstArrayView<uint8> Bytes,
	FURLabWireValue& OutValue, const FURLabWireLimits& Limits)
{
	const FURLabWireStatus LimitStatus = ValidateLimits(Limits);
	if (!LimitStatus) return LimitStatus;
	if (Bytes.Num() > Limits.MaxInputBytes)
		return MakeError(EURLabWireError::InputTooLarge,
			TEXT("input exceeds the byte limit"), 0);
	if (Bytes.IsEmpty())
		return MakeError(EURLabWireError::Truncated, TEXT("input is empty"), 0);
	FURLabWireValue Local;
	FBoundedMsgpackParser Parser(Bytes, Limits);
	const FURLabWireStatus Status = Parser.Parse(Local);
	if (!Status) return Status;
	OutValue = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::EncodeValue(const FURLabWireValue& Value,
	TArray<uint8>& OutBytes, const FURLabWireLimits& Limits)
{
	const FURLabWireStatus LimitStatus = ValidateLimits(Limits);
	if (!LimitStatus) return LimitStatus;
	TArray<uint8> Local;
	FBoundedMsgpackEncoder Encoder(Limits);
	const FURLabWireStatus Status = Encoder.Encode(Value, Local);
	if (!Status) return Status;
	OutBytes = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::DecodeIdentifierUtf8(
	TConstArrayView<uint8> Bytes, FString& OutIdentifier, const FURLabWireLimits& Limits)
{
	const FURLabWireStatus LimitStatus = ValidateLimits(Limits);
	if (!LimitStatus) return LimitStatus;
	if (Bytes.IsEmpty() || Bytes.Num() > Limits.MaxIdentifierBytes)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("identifier must contain 1..256 UTF-8 bytes"));
	int32 BadOffset = INDEX_NONE;
	if (!IsValidUtf8(Bytes, BadOffset))
		return MakeError(EURLabWireError::InvalidUtf8,
			TEXT("identifier is not valid UTF-8"), BadOffset);
	TArray<ANSICHAR> Guarded;
	Guarded.Append(reinterpret_cast<const ANSICHAR*>(Bytes.GetData()), Bytes.Num());
	Guarded.Add(1);
	FUTF8ToTCHAR Converted(Guarded.GetData(), Guarded.Num());
	FString Local = FString::ConstructFromPtrSize(Converted.Get(), Converted.Length() - 1);
	const FURLabWireStatus Status = ValidateIdentifier(Local, TEXT("identifier"), Limits);
	if (!Status) return Status;
	OutIdentifier = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::Decode(TConstArrayView<uint8> Bytes,
	FURLabRequestEnvelope& OutEnvelope, const FURLabWireLimits& Limits)
{
	FURLabWireValue Root;
	FURLabWireStatus Status = DecodeValue(Bytes, Root, Limits);
	if (!Status) return Status;
	if (Root.GetType() != EURLabWireValueType::Map)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			TEXT("request envelope must be a map"));
	if (Root.GetMapSize() != 6)
		return MakeError(EURLabWireError::EnvelopeKeys,
			TEXT("request envelope keys must exactly match protocol version 2"));

	static const TCHAR* ExpectedKeys[] = {
		TEXT("protocol_version"), TEXT("boot_id"), TEXT("session_id"),
		TEXT("request_id"), TEXT("op"), TEXT("body")
	};
	for (int32 Index = 0; Index < Root.GetMapSize(); ++Index)
	{
		bool bKnown = false;
		for (const TCHAR* Expected : ExpectedKeys)
			bKnown |= ExactStringEquals(Root.GetMapKey(Index), FString(Expected));
		if (!bKnown)
			return MakeError(EURLabWireError::EnvelopeKeys,
				TEXT("request envelope keys must exactly match protocol version 2"));
	}

	const FURLabWireValue* ProtocolVersion = Root.FindMapValue(TEXT("protocol_version"));
	const FURLabWireValue* BootId = Root.FindMapValue(TEXT("boot_id"));
	const FURLabWireValue* SessionId = Root.FindMapValue(TEXT("session_id"));
	const FURLabWireValue* RequestId = Root.FindMapValue(TEXT("request_id"));
	const FURLabWireValue* Op = Root.FindMapValue(TEXT("op"));
	const FURLabWireValue* Body = Root.FindMapValue(TEXT("body"));
	if (!ProtocolVersion || !BootId || !SessionId || !RequestId || !Op || !Body)
		return MakeError(EURLabWireError::EnvelopeKeys,
			TEXT("request envelope keys must exactly match protocol version 2"));
	if (ProtocolVersion->GetType() != EURLabWireValueType::Int64)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			TEXT("protocol_version must be an integer"));
	if (ProtocolVersion->GetInt() != 2)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("protocol_version must equal 2"));
	if (RequestId->GetType() != EURLabWireValueType::Int64)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			TEXT("request_id must be an integer"));
	if (RequestId->GetInt() < 1)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("request_id must be in 1..INT64_MAX"));
	if (BootId->GetType() != EURLabWireValueType::String
		|| SessionId->GetType() != EURLabWireValueType::String
		|| Op->GetType() != EURLabWireValueType::String)
	{
		return MakeError(EURLabWireError::EnvelopeFieldType,
			TEXT("boot_id, session_id, and op must be strings"));
	}
	if (Body->GetType() != EURLabWireValueType::Map)
		return MakeError(EURLabWireError::EnvelopeFieldType, TEXT("body must be a map"));
	if (!(Status = ValidateIdentifier(BootId->GetString(), TEXT("boot_id"), Limits))) return Status;
	if (!(Status = ValidateIdentifier(SessionId->GetString(), TEXT("session_id"), Limits))) return Status;
	if (!(Status = ValidateIdentifier(Op->GetString(), TEXT("op"), Limits))) return Status;

	FURLabRequestEnvelope Local;
	Local.ProtocolVersion = 2;
	Local.BootId = BootId->GetString();
	Local.SessionId = SessionId->GetString();
	Local.RequestId = static_cast<uint64>(RequestId->GetInt());
	Local.Op = Op->GetString();
	Local.Body = *Body;
	OutEnvelope = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::Encode(const FURLabRequestEnvelope& Envelope,
	TArray<uint8>& OutBytes, const FURLabWireLimits& Limits)
{
	if (Envelope.ProtocolVersion != 2)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("protocol_version must equal 2"));
	if (Envelope.RequestId < 1 || Envelope.RequestId > static_cast<uint64>(MAX_int64))
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("request_id must be in 1..INT64_MAX"));
	FURLabWireStatus Status = ValidateIdentifier(Envelope.BootId, TEXT("boot_id"), Limits);
	if (!Status) return Status;
	if (!(Status = ValidateIdentifier(Envelope.SessionId, TEXT("session_id"), Limits))) return Status;
	if (!(Status = ValidateIdentifier(Envelope.Op, TEXT("op"), Limits))) return Status;
	if (Envelope.Body.GetType() != EURLabWireValueType::Map)
		return MakeError(EURLabWireError::EnvelopeFieldType, TEXT("body must be a map"));
	const FURLabWireValue Root = MakeMap({
		{TEXT("protocol_version"), FURLabWireValue::MakeInt(2)},
		{TEXT("boot_id"), FURLabWireValue::MakeString(Envelope.BootId)},
		{TEXT("session_id"), FURLabWireValue::MakeString(Envelope.SessionId)},
		{TEXT("request_id"), FURLabWireValue::MakeInt(static_cast<int64>(Envelope.RequestId))},
		{TEXT("op"), FURLabWireValue::MakeString(Envelope.Op)},
		{TEXT("body"), Envelope.Body}
	});
	return EncodeValue(Root, OutBytes, Limits);
}

FURLabWireStatus FURLabRequestEnvelopeCodec::DecodeReply(TConstArrayView<uint8> Bytes,
	FURLabReplyEnvelope& OutReply, const FURLabWireLimits& Limits)
{
	FURLabWireValue Root;
	FURLabWireStatus Status = DecodeValue(Bytes, Root, Limits);
	if (!Status) return Status;
	if (Root.GetType() != EURLabWireValueType::Map || Root.GetMapSize() != 6)
		return MakeError(EURLabWireError::EnvelopeKeys,
			TEXT("reply must contain routing fields and exactly one outcome"));
	const FURLabWireValue* Result = Root.FindMapValue(TEXT("result"));
	const FURLabWireValue* Pending = Root.FindMapValue(TEXT("pending"));
	const FURLabWireValue* Error = Root.FindMapValue(TEXT("error"));
	if (static_cast<int32>(Result != nullptr) + static_cast<int32>(Pending != nullptr)
		+ static_cast<int32>(Error != nullptr) != 1)
	{
		return MakeError(EURLabWireError::EnvelopeKeys,
			TEXT("reply must contain routing fields and exactly one outcome"));
	}
	const TCHAR* OutcomeKey = Result ? TEXT("result") : Pending ? TEXT("pending") : TEXT("error");
	if (!(Status = RequireExactKeys(Root,
		{TEXT("protocol_version"), TEXT("boot_id"), TEXT("session_id"),
		 TEXT("request_id"), TEXT("op"), OutcomeKey}, TEXT("reply")))) return Status;
	if (!(Status = RequireProtocol2(Root.FindMapValue(TEXT("protocol_version"))))) return Status;
	const FURLabWireValue* RequestId = Root.FindMapValue(TEXT("request_id"));
	if (!RequestId || RequestId->GetType() != EURLabWireValueType::Int64)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			TEXT("request_id must be an integer"));
	if (RequestId->GetInt() < 1)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("request_id must be in 1..INT64_MAX"));
	FURLabReplyEnvelope Local;
	Local.ProtocolVersion = 2;
	Local.RequestId = static_cast<uint64>(RequestId->GetInt());
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("boot_id")), TEXT("boot_id"),
		Local.BootId, Limits))) return Status;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("session_id")), TEXT("session_id"),
		Local.SessionId, Limits))) return Status;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("op")), TEXT("op"),
		Local.Op, Limits))) return Status;
	if (Result)
	{
		if (Result->GetType() != EURLabWireValueType::Map)
			return MakeError(EURLabWireError::EnvelopeFieldType, TEXT("result must be a map"));
		Local.Kind = EURLabReplyKind::Result;
		Local.Result = *Result;
	}
	else if (Pending)
	{
		if (Pending->GetType() != EURLabWireValueType::Map || Pending->GetMapSize() != 0)
			return MakeError(EURLabWireError::EnvelopeFieldValue,
				TEXT("pending must be an empty map"));
		Local.Kind = EURLabReplyKind::Pending;
	}
	else
	{
		if (!(Status = RequireExactKeys(*Error,
			{TEXT("code"), TEXT("message"), TEXT("execution")}, TEXT("reply error"))))
			return Status;
		Local.Kind = EURLabReplyKind::Error;
		if (!(Status = ReadIdentifier(Error->FindMapValue(TEXT("code")), TEXT("code"),
			Local.Error.Code, Limits))) return Status;
		const FURLabWireValue* Message = Error->FindMapValue(TEXT("message"));
		const FURLabWireValue* Execution = Error->FindMapValue(TEXT("execution"));
		if (!Message || Message->GetType() != EURLabWireValueType::String
			|| Message->GetString().IsEmpty())
			return MakeError(EURLabWireError::EnvelopeFieldValue,
				TEXT("error message must be a non-empty string"));
		if (!Execution || Execution->GetType() != EURLabWireValueType::String)
			return MakeError(EURLabWireError::EnvelopeFieldType,
				TEXT("error execution must be a string"));
		Local.Error.Message = Message->GetString();
		if (ExactStringEquals(Execution->GetString(), TEXT("rejected")))
			Local.Error.Execution = EURLabExecution::Rejected;
		else if (ExactStringEquals(Execution->GetString(), TEXT("terminal")))
			Local.Error.Execution = EURLabExecution::Terminal;
		else if (ExactStringEquals(Execution->GetString(), TEXT("unknown")))
			Local.Error.Execution = EURLabExecution::Unknown;
		else
			return MakeError(EURLabWireError::EnvelopeFieldValue,
				TEXT("error execution has an unknown value"));
	}
	OutReply = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::EncodeReply(const FURLabReplyEnvelope& Reply,
	TArray<uint8>& OutBytes, const FURLabWireLimits& Limits)
{
	if (Reply.ProtocolVersion != 2)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("protocol_version must equal 2"));
	if (Reply.RequestId < 1 || Reply.RequestId > static_cast<uint64>(MAX_int64))
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("request_id must be in 1..INT64_MAX"));
	FURLabWireStatus Status = ValidateIdentifier(Reply.BootId, TEXT("boot_id"), Limits);
	if (!Status) return Status;
	if (!(Status = ValidateIdentifier(Reply.SessionId, TEXT("session_id"), Limits))) return Status;
	if (!(Status = ValidateIdentifier(Reply.Op, TEXT("op"), Limits))) return Status;
	const TCHAR* OutcomeKey = nullptr;
	FURLabWireValue Outcome;
	if (Reply.Kind == EURLabReplyKind::Result)
	{
		if (Reply.Result.GetType() != EURLabWireValueType::Map)
			return MakeError(EURLabWireError::EnvelopeFieldType, TEXT("result must be a map"));
		OutcomeKey = TEXT("result");
		Outcome = Reply.Result;
	}
	else if (Reply.Kind == EURLabReplyKind::Pending)
	{
		OutcomeKey = TEXT("pending");
		Outcome = FURLabWireValue::MakeMap(TArray<FString>{}, TArray<FURLabWireValue>{});
	}
	else if (Reply.Kind == EURLabReplyKind::Error)
	{
		if (!(Status = ValidateIdentifier(Reply.Error.Code, TEXT("code"), Limits))) return Status;
		if (Reply.Error.Message.IsEmpty())
			return MakeError(EURLabWireError::EnvelopeFieldValue,
				TEXT("error message must be a non-empty string"));
		const TCHAR* Execution = nullptr;
		switch (Reply.Error.Execution)
		{
		case EURLabExecution::Rejected: Execution = TEXT("rejected"); break;
		case EURLabExecution::Terminal: Execution = TEXT("terminal"); break;
		case EURLabExecution::Unknown: Execution = TEXT("unknown"); break;
		default:
			return MakeError(EURLabWireError::EnvelopeFieldValue,
				TEXT("unknown error execution"));
		}
		OutcomeKey = TEXT("error");
		Outcome = MakeMap({
			{TEXT("code"), FURLabWireValue::MakeString(Reply.Error.Code)},
			{TEXT("message"), FURLabWireValue::MakeString(Reply.Error.Message)},
			{TEXT("execution"), FURLabWireValue::MakeString(Execution)}
		});
	}
	else
	{
		return MakeError(EURLabWireError::EnvelopeFieldValue, TEXT("unknown reply outcome"));
	}
	const FURLabWireValue Root = MakeMap({
		{TEXT("protocol_version"), FURLabWireValue::MakeInt(2)},
		{TEXT("boot_id"), FURLabWireValue::MakeString(Reply.BootId)},
		{TEXT("session_id"), FURLabWireValue::MakeString(Reply.SessionId)},
		{TEXT("request_id"), FURLabWireValue::MakeInt(static_cast<int64>(Reply.RequestId))},
		{TEXT("op"), FURLabWireValue::MakeString(Reply.Op)},
		{OutcomeKey, MoveTemp(Outcome)}
	});
	return EncodeValue(Root, OutBytes, Limits);
}

FURLabWireStatus FURLabRequestEnvelopeCodec::DecodeConnectRequest(
	TConstArrayView<uint8> Bytes, FURLabConnectRequest& OutRequest,
	const FURLabWireLimits& Limits)
{
	FURLabWireValue Root;
	FURLabWireStatus Status = DecodeValue(Bytes, Root, Limits);
	if (!Status) return Status;
	if (!(Status = RequireExactKeys(Root,
		{TEXT("op"), TEXT("protocol_version"), TEXT("data_policy")},
		TEXT("connect request")))) return Status;
	if (!(Status = RequireLiteral(Root.FindMapValue(TEXT("op")), TEXT("op"), TEXT("connect"))))
		return Status;
	if (!(Status = RequireProtocol2(Root.FindMapValue(TEXT("protocol_version"))))) return Status;
	FURLabConnectRequest Local;
	Local.ProtocolVersion = 2;
	const FURLabWireValue* Policy = Root.FindMapValue(TEXT("data_policy"));
	if (!Policy || Policy->GetType() != EURLabWireValueType::String)
		return MakeError(EURLabWireError::EnvelopeFieldType,
			TEXT("data_policy must be a string"));
	if (!IsDataPolicy(Policy->GetString()))
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("data_policy must be network, auto, or shm"));
	Local.DataPolicy = Policy->GetString();
	OutRequest = Local;
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::EncodeConnectRequest(
	const FURLabConnectRequest& Request, TArray<uint8>& OutBytes,
	const FURLabWireLimits& Limits)
{
	if (Request.ProtocolVersion != 2)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("protocol_version must equal 2"));
	if (!IsDataPolicy(Request.DataPolicy))
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("data_policy must be network, auto, or shm"));
	return EncodeValue(MakeMap({
		{TEXT("op"), FURLabWireValue::MakeString(TEXT("connect"))},
		{TEXT("protocol_version"), FURLabWireValue::MakeInt(2)},
		{TEXT("data_policy"), FURLabWireValue::MakeString(Request.DataPolicy)}
	}), OutBytes, Limits);
}

FURLabWireStatus FURLabRequestEnvelopeCodec::DecodeConnectReply(
	TConstArrayView<uint8> Bytes, FURLabConnectReply& OutReply,
	const FURLabWireLimits& Limits)
{
	FURLabWireValue Root;
	FURLabWireStatus Status = DecodeValue(Bytes, Root, Limits);
	if (!Status) return Status;
	if (!(Status = RequireExactKeys(Root,
		{TEXT("op"), TEXT("protocol_version"), TEXT("instance_id"), TEXT("boot_id"),
			 TEXT("session_id"), TEXT("urlab_version"), TEXT("command_endpoints"),
			 TEXT("services"), TEXT("permissions"), TEXT("data_offers"),
			 TEXT("data_negotiation"), TEXT("limits")},
		TEXT("connect reply")))) return Status;
	if (!(Status = RequireLiteral(Root.FindMapValue(TEXT("op")), TEXT("op"),
		TEXT("connect_ok")))) return Status;
	if (!(Status = RequireProtocol2(Root.FindMapValue(TEXT("protocol_version"))))) return Status;
	FURLabConnectReply Local;
	Local.ProtocolVersion = 2;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("instance_id")), TEXT("instance_id"),
		Local.InstanceId, Limits))) return Status;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("boot_id")), TEXT("boot_id"),
		Local.BootId, Limits))) return Status;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("session_id")), TEXT("session_id"),
		Local.SessionId, Limits))) return Status;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("urlab_version")), TEXT("urlab_version"),
		Local.URLabVersion, Limits))) return Status;
	if (!(Status = ReadStringArray(Root.FindMapValue(TEXT("command_endpoints")),
		TEXT("command_endpoints"), Local.CommandEndpoints))) return Status;
	if (!(Status = ReadStringArray(Root.FindMapValue(TEXT("services")),
		TEXT("services"), Local.Services))) return Status;
	if (!(Status = ReadStringArray(Root.FindMapValue(TEXT("permissions")),
		TEXT("permissions"), Local.Permissions))) return Status;
	if (!(Status = ReadStringArray(Root.FindMapValue(TEXT("data_offers")),
		TEXT("data_offers"), Local.DataOffers))) return Status;
	if (!(Status = DecodeDataNegotiation(Root.FindMapValue(TEXT("data_negotiation")),
		Local.DataNegotiation, Limits))) return Status;
	const FURLabWireValue* LimitMap = Root.FindMapValue(TEXT("limits"));
	if (!LimitMap || LimitMap->GetType() != EURLabWireValueType::Map)
		return MakeError(EURLabWireError::EnvelopeFieldType, TEXT("limits must be a map"));
	Local.Limits = *LimitMap;
	OutReply = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::EncodeConnectReply(
	const FURLabConnectReply& Reply, TArray<uint8>& OutBytes,
	const FURLabWireLimits& Limits)
{
	if (Reply.ProtocolVersion != 2)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("protocol_version must equal 2"));
	FURLabWireStatus Status = ValidateIdentifier(Reply.InstanceId, TEXT("instance_id"), Limits);
	if (!Status) return Status;
	if (!(Status = ValidateIdentifier(Reply.BootId, TEXT("boot_id"), Limits))) return Status;
	if (!(Status = ValidateIdentifier(Reply.SessionId, TEXT("session_id"), Limits))) return Status;
	if (!(Status = ValidateIdentifier(Reply.URLabVersion, TEXT("urlab_version"), Limits))) return Status;
	if (Reply.Limits.GetType() != EURLabWireValueType::Map)
		return MakeError(EURLabWireError::EnvelopeFieldType, TEXT("limits must be a map"));
	FURLabWireValue DataNegotiation;
	if (!(Status = EncodeDataNegotiation(Reply.DataNegotiation, DataNegotiation, Limits)))
		return Status;
	return EncodeValue(MakeMap({
		{TEXT("op"), FURLabWireValue::MakeString(TEXT("connect_ok"))},
		{TEXT("protocol_version"), FURLabWireValue::MakeInt(2)},
		{TEXT("instance_id"), FURLabWireValue::MakeString(Reply.InstanceId)},
		{TEXT("boot_id"), FURLabWireValue::MakeString(Reply.BootId)},
		{TEXT("session_id"), FURLabWireValue::MakeString(Reply.SessionId)},
		{TEXT("urlab_version"), FURLabWireValue::MakeString(Reply.URLabVersion)},
		{TEXT("command_endpoints"), MakeStringArray(Reply.CommandEndpoints)},
		{TEXT("services"), MakeStringArray(Reply.Services)},
		{TEXT("permissions"), MakeStringArray(Reply.Permissions)},
		{TEXT("data_offers"), MakeStringArray(Reply.DataOffers)},
		{TEXT("data_negotiation"), MoveTemp(DataNegotiation)},
		{TEXT("limits"), Reply.Limits}
	}), OutBytes, Limits);
}

FURLabWireStatus FURLabRequestEnvelopeCodec::DecodeDisconnectRequest(
	TConstArrayView<uint8> Bytes, FURLabDisconnectRequest& OutRequest,
	const FURLabWireLimits& Limits)
{
	FURLabWireValue Root;
	FURLabWireStatus Status = DecodeValue(Bytes, Root, Limits);
	if (!Status) return Status;
	if (!(Status = RequireExactKeys(Root,
		{TEXT("op"), TEXT("protocol_version"), TEXT("boot_id"), TEXT("session_id")},
		TEXT("disconnect request")))) return Status;
	if (!(Status = RequireLiteral(Root.FindMapValue(TEXT("op")), TEXT("op"),
		TEXT("disconnect")))) return Status;
	if (!(Status = RequireProtocol2(Root.FindMapValue(TEXT("protocol_version"))))) return Status;
	FURLabDisconnectRequest Local;
	Local.ProtocolVersion = 2;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("boot_id")), TEXT("boot_id"),
		Local.BootId, Limits))) return Status;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("session_id")), TEXT("session_id"),
		Local.SessionId, Limits))) return Status;
	OutRequest = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::EncodeDisconnectRequest(
	const FURLabDisconnectRequest& Request, TArray<uint8>& OutBytes,
	const FURLabWireLimits& Limits)
{
	if (Request.ProtocolVersion != 2)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("protocol_version must equal 2"));
	FURLabWireStatus Status = ValidateIdentifier(Request.BootId, TEXT("boot_id"), Limits);
	if (!Status) return Status;
	if (!(Status = ValidateIdentifier(Request.SessionId, TEXT("session_id"), Limits))) return Status;
	return EncodeValue(MakeMap({
		{TEXT("op"), FURLabWireValue::MakeString(TEXT("disconnect"))},
		{TEXT("protocol_version"), FURLabWireValue::MakeInt(2)},
		{TEXT("boot_id"), FURLabWireValue::MakeString(Request.BootId)},
		{TEXT("session_id"), FURLabWireValue::MakeString(Request.SessionId)}
	}), OutBytes, Limits);
}

FURLabWireStatus FURLabRequestEnvelopeCodec::DecodeDisconnectReply(
	TConstArrayView<uint8> Bytes, FURLabDisconnectReply& OutReply,
	const FURLabWireLimits& Limits)
{
	FURLabWireValue Root;
	FURLabWireStatus Status = DecodeValue(Bytes, Root, Limits);
	if (!Status) return Status;
	if (!(Status = RequireExactKeys(Root,
		{TEXT("op"), TEXT("protocol_version"), TEXT("boot_id"), TEXT("session_id"),
		 TEXT("disconnected")}, TEXT("disconnect reply")))) return Status;
	if (!(Status = RequireLiteral(Root.FindMapValue(TEXT("op")), TEXT("op"),
		TEXT("disconnect_ok")))) return Status;
	if (!(Status = RequireProtocol2(Root.FindMapValue(TEXT("protocol_version"))))) return Status;
	const FURLabWireValue* Disconnected = Root.FindMapValue(TEXT("disconnected"));
	if (!Disconnected || Disconnected->GetType() != EURLabWireValueType::Bool
		|| !Disconnected->GetBool())
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("disconnected must equal true"));
	FURLabDisconnectReply Local;
	Local.ProtocolVersion = 2;
	Local.bDisconnected = true;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("boot_id")), TEXT("boot_id"),
		Local.BootId, Limits))) return Status;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("session_id")), TEXT("session_id"),
		Local.SessionId, Limits))) return Status;
	OutReply = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::EncodeDisconnectReply(
	const FURLabDisconnectReply& Reply, TArray<uint8>& OutBytes,
	const FURLabWireLimits& Limits)
{
	if (Reply.ProtocolVersion != 2 || !Reply.bDisconnected)
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("disconnect reply requires protocol 2 and disconnected true"));
	FURLabWireStatus Status = ValidateIdentifier(Reply.BootId, TEXT("boot_id"), Limits);
	if (!Status) return Status;
	if (!(Status = ValidateIdentifier(Reply.SessionId, TEXT("session_id"), Limits))) return Status;
	return EncodeValue(MakeMap({
		{TEXT("op"), FURLabWireValue::MakeString(TEXT("disconnect_ok"))},
		{TEXT("protocol_version"), FURLabWireValue::MakeInt(2)},
		{TEXT("boot_id"), FURLabWireValue::MakeString(Reply.BootId)},
		{TEXT("session_id"), FURLabWireValue::MakeString(Reply.SessionId)},
		{TEXT("disconnected"), FURLabWireValue::MakeBool(true)}
	}), OutBytes, Limits);
}

FURLabWireStatus FURLabRequestEnvelopeCodec::DecodeProtocolError(
	TConstArrayView<uint8> Bytes, FURLabProtocolError& OutError,
	const FURLabWireLimits& Limits)
{
	FURLabWireValue Root;
	FURLabWireStatus Status = DecodeValue(Bytes, Root, Limits);
	if (!Status) return Status;
	if (!(Status = RequireExactKeys(Root, {TEXT("op"), TEXT("code"), TEXT("message")},
		TEXT("protocol error")))) return Status;
	if (!(Status = RequireLiteral(Root.FindMapValue(TEXT("op")), TEXT("op"), TEXT("error"))))
		return Status;
	FURLabProtocolError Local;
	if (!(Status = ReadIdentifier(Root.FindMapValue(TEXT("code")), TEXT("code"),
		Local.Code, Limits))) return Status;
	const FURLabWireValue* Message = Root.FindMapValue(TEXT("message"));
	if (!Message || Message->GetType() != EURLabWireValueType::String
		|| Message->GetString().IsEmpty())
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("message must be a non-empty string"));
	Local.Message = Message->GetString();
	OutError = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::EncodeProtocolError(
	const FURLabProtocolError& Error, TArray<uint8>& OutBytes,
	const FURLabWireLimits& Limits)
{
	FURLabWireStatus Status = ValidateIdentifier(Error.Code, TEXT("code"), Limits);
	if (!Status) return Status;
	if (Error.Message.IsEmpty())
		return MakeError(EURLabWireError::EnvelopeFieldValue,
			TEXT("message must be a non-empty string"));
	return EncodeValue(MakeMap({
		{TEXT("op"), FURLabWireValue::MakeString(TEXT("error"))},
		{TEXT("code"), FURLabWireValue::MakeString(Error.Code)},
		{TEXT("message"), FURLabWireValue::MakeString(Error.Message)}
	}), OutBytes, Limits);
}

FURLabWireStatus FURLabRequestEnvelopeCodec::CanonicalBytes(const FString& Op,
	const FURLabWireValue& Body, TArray<uint8>& OutBytes, const FURLabWireLimits& Limits)
{
	const FURLabWireStatus LimitStatus = ValidateLimits(Limits);
	if (!LimitStatus) return LimitStatus;
	TArray<uint8> Local;
	FCanonicalEncoder Encoder(Limits);
	const FURLabWireStatus Status = Encoder.EncodeRequest(Op, Body, Local);
	if (!Status) return Status;
	OutBytes = MoveTemp(Local);
	return FURLabWireStatus::Ok();
}

FURLabWireStatus FURLabRequestEnvelopeCodec::Digest(const FString& Op,
	const FURLabWireValue& Body, FURLabRequestDigest& OutDigest,
	const FURLabWireLimits& Limits)
{
	TArray<uint8> Canonical;
	const FURLabWireStatus Status = CanonicalBytes(Op, Body, Canonical, Limits);
	if (!Status) return Status;
	const FString Hex = FURLabAssetCache::Sha256Hex(Canonical);
	if (Hex.Len() != FURLabRequestDigest::NumBytes * 2)
		return MakeError(EURLabWireError::InvalidValue, TEXT("SHA-256 returned an invalid digest"));

	FURLabRequestDigest Local;
	for (int32 Index = 0; Index < FURLabRequestDigest::NumBytes; ++Index)
	{
		auto Nibble = [](TCHAR Character) -> int32
		{
			if (Character >= '0' && Character <= '9') return Character - '0';
			if (Character >= 'a' && Character <= 'f') return Character - 'a' + 10;
			return INDEX_NONE;
		};
		const int32 High = Nibble(Hex[Index * 2]);
		const int32 Low = Nibble(Hex[Index * 2 + 1]);
		if (High < 0 || Low < 0)
			return MakeError(EURLabWireError::InvalidValue, TEXT("SHA-256 returned invalid hex"));
		Local.Bytes[Index] = static_cast<uint8>((High << 4) | Low);
	}
	OutDigest = Local;
	return FURLabWireStatus::Ok();
}

const TCHAR* FURLabRequestEnvelopeCodec::ErrorCodeString(EURLabWireError Code)
{
	switch (Code)
	{
	case EURLabWireError::None: return TEXT("none");
	case EURLabWireError::InputTooLarge: return TEXT("input_too_large");
	case EURLabWireError::OutputTooLarge: return TEXT("output_too_large");
	case EURLabWireError::Truncated: return TEXT("truncated");
	case EURLabWireError::InvalidMsgpack: return TEXT("invalid_msgpack");
	case EURLabWireError::TrailingData: return TEXT("trailing_data");
	case EURLabWireError::DepthLimit: return TEXT("depth_limit");
	case EURLabWireError::ContainerLimit: return TEXT("container_limit");
	case EURLabWireError::DuplicateKey: return TEXT("duplicate_key");
	case EURLabWireError::InvalidUtf8: return TEXT("invalid_utf8");
	case EURLabWireError::NonStringMapKey: return TEXT("non_string_map_key");
	case EURLabWireError::UnsupportedType: return TEXT("unsupported_type");
	case EURLabWireError::IntegerOutOfRange: return TEXT("integer_out_of_range");
	case EURLabWireError::NonFiniteFloat: return TEXT("non_finite_float");
	case EURLabWireError::EnvelopeKeys: return TEXT("envelope_keys");
	case EURLabWireError::EnvelopeFieldType: return TEXT("envelope_field_type");
	case EURLabWireError::EnvelopeFieldValue: return TEXT("envelope_field_value");
	case EURLabWireError::InvalidValue: return TEXT("invalid_value");
	default: return TEXT("invalid_value");
	}
}
