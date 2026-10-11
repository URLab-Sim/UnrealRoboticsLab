// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

// Primitive MJCF text values for generated UE field thunks. XML traversal,
// element meaning and spatial normalization live elsewhere.

#include "CoreMinimal.h"
#include "String/LexFromString.h"
#include "MuJoCo/Gen/MjKeywords.gen.h"
#include "MuJoCo/Spec/MjFrameTypes.h"

#include <string_view>
#include <string>
#include <type_traits>

namespace urlab::spec::fieldtext
{
inline std::string ToUtf8(FStringView Text)
{
	FTCHARToUTF8 Converted(Text.GetData(), Text.Len());
	return std::string(Converted.Get(), Converted.Length());
}

inline FString FromUtf8(std::string_view Text)
{
	FUTF8ToTCHAR Converted(Text.data(), static_cast<int32>(Text.size()));
	return FString::ConstructFromPtrSize(Converted.Get(), Converted.Length());
}

template <class T> struct TIsArray : std::false_type {};
template <class T> struct TIsArray<TArray<T>> : std::true_type { using Item = T; };

template <class T>
bool ParseAtom(FStringView Text, T& Out)
{
	if constexpr (std::is_same_v<T, FString>)
	{
		Out = FString(Text);
		return true;
	}
	else if constexpr (std::is_same_v<T, bool>)
	{
		if (Text.Equals(TEXT("true"), ESearchCase::CaseSensitive)) { Out = true; return true; }
		if (Text.Equals(TEXT("false"), ESearchCase::CaseSensitive)) { Out = false; return true; }
		return false;
	}
	else if constexpr (std::is_enum_v<T>)
	{
		const std::string Keyword = ToUtf8(Text);
		return ps::ue::FromMjcf(Keyword, Out);
	}
	else
	{
		return LexTryParseString(Out, *FString(Text));
	}
}

template <class T>
FString FormatAtom(const T& Value)
{
	if constexpr (std::is_same_v<T, FString>) return Value;
	else if constexpr (std::is_same_v<T, bool>) return Value ? TEXT("true") : TEXT("false");
	else if constexpr (std::is_enum_v<T>) return FromUtf8(ps::ue::ToMjcf(Value));
	else if constexpr (std::is_floating_point_v<T>)
	{
		if constexpr (std::is_same_v<T, float>) return FString::Printf(TEXT("%.9g"), static_cast<double>(Value));
		else return FString::Printf(TEXT("%.17g"), Value);
	}
	else return LexToString(Value);
}

inline void Tokens(FStringView Text, TArray<FString>& Out)
{
	FString(Text).ParseIntoArrayWS(Out);
}

template <class T>
bool Parse(FStringView Text, T& Out, int32 MinCount = 1, int32 MaxCount = 1)
{
	if constexpr (std::is_same_v<T, FString>) return ParseAtom(Text, Out);
	else if constexpr (TIsArray<T>::value)
	{
		using Item = typename TIsArray<T>::Item;
		TArray<FString> Parts;
		Tokens(Text, Parts);
		if (Parts.Num() < MinCount || (MaxCount >= 0 && Parts.Num() > MaxCount)) return false;
		T Parsed;
		Parsed.Reserve(Parts.Num());
		for (const FString& Part : Parts)
		{
			Item Value{};
			if (!ParseAtom<Item>(FStringView(Part), Value)) return false;
			Parsed.Add(Value);
		}
		Out = MoveTemp(Parsed);
		return true;
	}
	else if constexpr (std::is_same_v<T, FVector2D> || std::is_same_v<T, FMjPosition3>
		|| std::is_same_v<T, FMjDirection3> || std::is_same_v<T, FMjVec3>
		|| std::is_same_v<T, FMjQuatRot> || std::is_same_v<T, FLinearColor>)
	{
		constexpr int32 Count = std::is_same_v<T, FVector2D> ? 2
			: (std::is_same_v<T, FMjQuatRot> || std::is_same_v<T, FLinearColor> ? 4 : 3);
		TArray<FString> Parts;
		Tokens(Text, Parts);
		if (Parts.Num() != Count) return false;
		double V[4]{};
		for (int32 I = 0; I < Count; ++I)
			if (!ParseAtom<double>(FStringView(Parts[I]), V[I])) return false;
		if constexpr (std::is_same_v<T, FVector2D>) Out = FVector2D(V[0], V[1]);
		else if constexpr (std::is_same_v<T, FMjQuatRot>) Out = FMjQuatRot(V[0], V[1], V[2], V[3]);
		else if constexpr (std::is_same_v<T, FLinearColor>) Out = FLinearColor(V[0], V[1], V[2], V[3]);
		else Out = T(V[0], V[1], V[2]);
		return true;
	}
	else return ParseAtom(Text, Out);
}

template <class T>
FString Format(const T& Value)
{
	if constexpr (TIsArray<T>::value)
	{
		FString Out;
		for (const auto& Item : Value)
		{
			if (!Out.IsEmpty()) Out += TEXT(" ");
			Out += FormatAtom(Item);
		}
		return Out;
	}
	else if constexpr (std::is_same_v<T, FVector2D>)
		return FormatAtom(Value.X) + TEXT(" ") + FormatAtom(Value.Y);
	else if constexpr (std::is_same_v<T, FMjQuatRot>)
		return FormatAtom(Value.W) + TEXT(" ") + FormatAtom(Value.X) + TEXT(" ")
			+ FormatAtom(Value.Y) + TEXT(" ") + FormatAtom(Value.Z);
	else if constexpr (std::is_same_v<T, FMjPosition3> || std::is_same_v<T, FMjDirection3>
		|| std::is_same_v<T, FMjVec3>)
		return FormatAtom(Value.X) + TEXT(" ") + FormatAtom(Value.Y) + TEXT(" ") + FormatAtom(Value.Z);
	else if constexpr (std::is_same_v<T, FLinearColor>)
		return FormatAtom(Value.R) + TEXT(" ") + FormatAtom(Value.G) + TEXT(" ")
			+ FormatAtom(Value.B) + TEXT(" ") + FormatAtom(Value.A);
	else return FormatAtom(Value);
}
} // namespace urlab::spec::fieldtext
