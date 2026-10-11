// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Wire/OperationSchema.h"

#include "Math/UnrealMathUtility.h"

namespace
{
FURLabWireStatus Invalid(const FString& Detail)
{
	return FURLabWireStatus::Error(EURLabWireError::InvalidValue, Detail);
}

bool Exact(const FString& A, const TCHAR* B)
{
	return B && A.Equals(B, ESearchCase::CaseSensitive);
}

const FURLabWireValue* FindExact(const FURLabWireValue& Map, const TCHAR* Key)
{
	if (Map.GetType() != EURLabWireValueType::Map || !Key) return nullptr;
	for (int32 Index = 0; Index < Map.GetMapSize(); ++Index)
	{
		if (Exact(Map.GetMapKey(Index), Key)) return &Map.GetMapValue(Index);
	}
	return nullptr;
}

enum class EGeneratedSchemaKind : uint8
{
	Request,
	Result
};

struct FGeneratedField
{
	const TCHAR* Name = nullptr;
	const TCHAR* Type = nullptr;
	const TCHAR* LengthFrom = nullptr;
	const TCHAR* Literal = nullptr;
	bool bRequired = true;
};

struct FGeneratedShape
{
	FString Name;
	TArray<FGeneratedField> Fields;
};

bool ReadShape(EGeneratedSchemaKind Kind, const FString& Name, FGeneratedShape& Out)
{
	Out = {};
	Out.Name = Name;
	if (Kind == EGeneratedSchemaKind::Request)
	{
		const URLabWire::TypedRequest::FShape* Shape = URLabWire::TypedRequest::Find(Name);
		if (!Shape) return false;
		Out.Fields.Reserve(Shape->Num);
		for (int32 Index = 0; Index < Shape->Num; ++Index)
		{
			const URLabWire::TypedRequest::FField& Field = Shape->Fields[Index];
			Out.Fields.Add({Field.Name, Field.Type, Field.LengthFrom, Field.Literal,
				Field.Required});
		}
		return true;
	}
	const URLabWire::TypedResult::FShape* Shape = URLabWire::TypedResult::Find(Name);
	if (!Shape) return false;
	Out.Fields.Reserve(Shape->Num);
	for (int32 Index = 0; Index < Shape->Num; ++Index)
	{
		const URLabWire::TypedResult::FField& Field = Shape->Fields[Index];
		Out.Fields.Add({Field.Name, Field.Type, Field.LengthFrom, Field.Literal,
			Field.Required});
	}
	return true;
}

FURLabWireStatus ResolveTaggedShape(EGeneratedSchemaKind Kind,
	const FString& UnionName, const FURLabWireValue& Value,
	FString& OutTarget, bool& bOutTagged, const FString& Path)
{
	bOutTagged = false;
	OutTarget.Reset();
	const TCHAR* Discriminator = nullptr;
	auto Consider = [&](const TCHAR* Union, const TCHAR* Field,
		const TCHAR* Tag, const TCHAR* Shape)
	{
		if (!Exact(UnionName, Union)) return;
		bOutTagged = true;
		Discriminator = Field;
	};
	if (Kind == EGeneratedSchemaKind::Request)
	{
		for (int32 Index = 0; Index < URLabWire::TypedRequest::TaggedVariantCount; ++Index)
		{
			const URLabWire::TypedRequest::FTaggedVariant& Variant =
				URLabWire::TypedRequest::TaggedVariants[Index];
			Consider(Variant.Union, Variant.Field, Variant.Tag, Variant.Shape);
		}
	}
	else
	{
		for (int32 Index = 0; Index < URLabWire::TypedResult::TaggedVariantCount; ++Index)
		{
			const URLabWire::TypedResult::FTaggedVariant& Variant =
				URLabWire::TypedResult::TaggedVariants[Index];
			Consider(Variant.Union, Variant.Field, Variant.Tag, Variant.Shape);
		}
	}
	if (!bOutTagged) return FURLabWireStatus::Ok();
	if (Value.GetType() != EURLabWireValueType::Map || !Value.HasValidMapStorage())
		return Invalid(FString::Printf(TEXT("%s must be a tagged map"), *Path));
	const FURLabWireValue* TagValue = FindExact(Value, Discriminator);
	if (!TagValue || TagValue->GetType() != EURLabWireValueType::String)
	{
		return Invalid(FString::Printf(TEXT("%s.%s must be a string discriminator"),
			*Path, Discriminator));
	}
	auto Resolve = [&](const TCHAR* Union, const TCHAR* Tag, const TCHAR* Shape)
	{
		if (Exact(UnionName, Union) && Exact(TagValue->GetString(), Tag))
			OutTarget = Shape;
	};
	if (Kind == EGeneratedSchemaKind::Request)
	{
		for (int32 Index = 0; Index < URLabWire::TypedRequest::TaggedVariantCount; ++Index)
		{
			const auto& Variant = URLabWire::TypedRequest::TaggedVariants[Index];
			Resolve(Variant.Union, Variant.Tag, Variant.Shape);
		}
	}
	else
	{
		for (int32 Index = 0; Index < URLabWire::TypedResult::TaggedVariantCount; ++Index)
		{
			const auto& Variant = URLabWire::TypedResult::TaggedVariants[Index];
			Resolve(Variant.Union, Variant.Tag, Variant.Shape);
		}
	}
	if (OutTarget.IsEmpty())
	{
		return Invalid(FString::Printf(TEXT("%s.%s has unknown tag %s"),
			*Path, Discriminator, *TagValue->GetString()));
	}
	return FURLabWireStatus::Ok();
}

const FGeneratedField* FindField(const FGeneratedShape& Shape, const FString& Name)
{
	for (const FGeneratedField& Field : Shape.Fields)
	{
		if (Exact(Name, Field.Name)) return &Field;
	}
	return nullptr;
}

FURLabWireStatus ValidateNamedShape(const FURLabWireValue& Value,
	EGeneratedSchemaKind Kind, const FString& ShapeName, const FString& Path);

FURLabWireStatus ValidateValue(const FURLabWireValue& Value,
	EGeneratedSchemaKind Kind, const FGeneratedField& Field, const FString& Path)
{
	const FString TypeName(Field.Type ? Field.Type : TEXT(""));
	bool bValid = false;
	if (TypeName == TEXT("string"))
		bValid = Value.GetType() == EURLabWireValueType::String;
	else if (TypeName == TEXT("nonempty_string"))
		bValid = Value.GetType() == EURLabWireValueType::String
			&& !Value.GetString().IsEmpty();
	else if (TypeName == TEXT("bool"))
	{
		bValid = Value.GetType() == EURLabWireValueType::Bool;
		if (bValid && Field.Literal)
		{
			const bool bExpected = FCString::Strcmp(Field.Literal, TEXT("true")) == 0;
			const bool bKnown = bExpected
				|| FCString::Strcmp(Field.Literal, TEXT("false")) == 0;
			bValid = bKnown && Value.GetBool() == bExpected;
		}
	}
	else if (TypeName == TEXT("finite_float"))
		bValid = Value.GetType() == EURLabWireValueType::Float64
			&& FMath::IsFinite(Value.GetFloat());
	else if (TypeName == TEXT("finite_vec3"))
	{
		bValid = Value.GetType() == EURLabWireValueType::Array
			&& Value.GetArray().Num() == 3;
		if (bValid) for (const FURLabWireValue& Item : Value.GetArray())
		{
			if (Item.GetType() != EURLabWireValueType::Float64
				|| !FMath::IsFinite(Item.GetFloat())) { bValid = false; break; }
		}
	}
	else if (TypeName == TEXT("int64"))
		bValid = Value.GetType() == EURLabWireValueType::Int64;
	else if (TypeName == TEXT("nonnegative_int64"))
		bValid = Value.GetType() == EURLabWireValueType::Int64 && Value.GetInt() >= 0;
	else if (TypeName == TEXT("positive_int64"))
		bValid = Value.GetType() == EURLabWireValueType::Int64 && Value.GetInt() > 0;
	else if (TypeName == TEXT("positive_uint32"))
		bValid = Value.GetType() == EURLabWireValueType::Int64 && Value.GetInt() > 0
			&& Value.GetInt() <= MAX_uint32;
	else if (TypeName == TEXT("binary"))
		bValid = Value.GetType() == EURLabWireValueType::Binary;
	else if (TypeName == TEXT("literal"))
		bValid = Field.Literal && Value.GetType() == EURLabWireValueType::String
			&& Exact(Value.GetString(), Field.Literal);
	else if (TypeName.StartsWith(TEXT("array<")) && TypeName.EndsWith(TEXT(">")))
	{
		if (Value.GetType() != EURLabWireValueType::Array)
			return Invalid(FString::Printf(TEXT("%s must have type %s"), *Path, Field.Type));
		const FString ElementType = TypeName.Mid(6, TypeName.Len() - 7);
		for (int32 Index = 0; Index < Value.GetArray().Num(); ++Index)
		{
			FGeneratedField Item;
			Item.Type = *ElementType;
			const FURLabWireStatus Status = ValidateValue(Value.GetArray()[Index],
				Kind, Item, FString::Printf(TEXT("%s[%d]"), *Path, Index));
			if (!Status) return Status;
		}
		return FURLabWireStatus::Ok();
	}
	else
	{
		FGeneratedShape Nested;
		if (ReadShape(Kind, TypeName, Nested))
			return ValidateNamedShape(Value, Kind, TypeName, Path);
		return Invalid(FString::Printf(TEXT("%s has unsupported schema type %s"),
			*Path, *TypeName));
	}
	return bValid ? FURLabWireStatus::Ok()
		: Invalid(FString::Printf(TEXT("%s must have type %s"), *Path, *TypeName));
}

FURLabWireStatus ResolveLength(const FURLabWireValue& Scope, const TCHAR* DottedPath,
	int64& OutLength, const FString& FieldPath)
{
	if (!DottedPath || !*DottedPath)
		return Invalid(FString::Printf(TEXT("%s has an empty length source"), *FieldPath));
	const FURLabWireValue* Current = &Scope;
	TArray<FString> Components;
	FString(DottedPath).ParseIntoArray(Components, TEXT("."), false);
	for (const FString& Component : Components)
	{
		if (Current->GetType() != EURLabWireValueType::Map)
			return Invalid(FString::Printf(TEXT("%s length source %s is not a map path"),
				*FieldPath, DottedPath));
		Current = FindExact(*Current, *Component);
		if (!Current)
			return Invalid(FString::Printf(TEXT("%s length source %s is missing"),
				*FieldPath, DottedPath));
	}
	if (Current->GetType() != EURLabWireValueType::Int64 || Current->GetInt() < 0)
		return Invalid(FString::Printf(TEXT("%s length source %s is not nonnegative"),
			*FieldPath, DottedPath));
	OutLength = Current->GetInt();
	return FURLabWireStatus::Ok();
}

FURLabWireStatus ValidateConcreteShape(const FURLabWireValue& Value,
	EGeneratedSchemaKind Kind, const FGeneratedShape& Shape, const FString& Path)
{
	if (Value.GetType() != EURLabWireValueType::Map || !Value.HasValidMapStorage())
		return Invalid(FString::Printf(TEXT("%s must be a map"), *Path));
	for (int32 Index = 0; Index < Value.GetMapSize(); ++Index)
	{
		const FString& Key = Value.GetMapKey(Index);
		for (int32 Prior = 0; Prior < Index; ++Prior)
		{
			if (Key.Equals(Value.GetMapKey(Prior), ESearchCase::CaseSensitive))
				return Invalid(FString::Printf(TEXT("%s contains duplicate field %s"),
					*Path, *Key));
		}
		if (!FindField(Shape, Key))
			return Invalid(FString::Printf(TEXT("%s contains unknown field %s"),
				*Path, *Key));
	}
	for (const FGeneratedField& Field : Shape.Fields)
	{
		const FURLabWireValue* Child = FindExact(Value, Field.Name);
		if (!Child)
		{
			if (Field.bRequired)
				return Invalid(FString::Printf(TEXT("%s is missing field %s"),
					*Path, Field.Name));
			continue;
		}
		const FString ChildPath = FString::Printf(TEXT("%s.%s"), *Path, Field.Name);
		FURLabWireStatus Status = ValidateValue(*Child, Kind, Field, ChildPath);
		if (!Status) return Status;
		if (Field.LengthFrom)
		{
			int64 ExpectedLength = 0;
			Status = ResolveLength(Value, Field.LengthFrom, ExpectedLength, ChildPath);
			if (!Status) return Status;
			if (Child->GetType() != EURLabWireValueType::Array
				|| ExpectedLength != Child->GetArray().Num())
				return Invalid(FString::Printf(TEXT("%s length does not match %s"),
					*ChildPath, Field.LengthFrom));
		}
	}
	return FURLabWireStatus::Ok();
}

FURLabWireStatus ValidateNamedShape(const FURLabWireValue& Value,
	EGeneratedSchemaKind Kind, const FString& ShapeName, const FString& Path)
{
	FGeneratedShape Shape;
	if (!ReadShape(Kind, ShapeName, Shape))
		return Invalid(FString::Printf(TEXT("%s has unknown generated shape %s"),
			*Path, *ShapeName));
	FString Target;
	bool bTagged = false;
	FURLabWireStatus Status = ResolveTaggedShape(Kind, ShapeName, Value,
		Target, bTagged, Path);
	if (!Status) return Status;
	if (bTagged) return ValidateNamedShape(Value, Kind, Target, Path);
	return ValidateConcreteShape(Value, Kind, Shape, Path);
}
} // namespace

FURLabWireStatus FURLabOperationSchema::ValidateRequestBody(const FString& Op,
	const FURLabWireValue& Body)
{
	const URLabWire::TypedOp::FDecl* Operation = URLabWire::TypedOp::Find(Op);
	if (!Operation)
		return Invalid(FString::Printf(TEXT("operation %s has no active request schema"), *Op));
	return ValidateNamedShape(Body, EGeneratedSchemaKind::Request,
		Operation->RequestShape, TEXT("body"));
}

FURLabWireStatus FURLabOperationSchema::ValidateResultBody(const FString& Op,
	const FURLabWireValue& Result)
{
	if (!URLabWire::TypedOp::Find(Op))
		return Invalid(FString::Printf(TEXT("operation %s has no active result schema"), *Op));
	return ValidateNamedShape(Result, EGeneratedSchemaKind::Result, Op, TEXT("result"));
}

FURLabWireStatus FURLabOperationSchema::ValidateShape(const FURLabWireValue& Value,
	const URLabWire::TypedRequest::FShape& Shape, const FString& Path)
{
	FGeneratedShape Supplied;
	Supplied.Name = Shape.Name;
	Supplied.Fields.Reserve(Shape.Num);
	for (int32 Index = 0; Index < Shape.Num; ++Index)
	{
		const URLabWire::TypedRequest::FField& Field = Shape.Fields[Index];
		Supplied.Fields.Add({Field.Name, Field.Type, Field.LengthFrom, Field.Literal,
			Field.Required});
	}
	FString Target;
	bool bTagged = false;
	FURLabWireStatus Status = ResolveTaggedShape(EGeneratedSchemaKind::Request,
		Supplied.Name, Value, Target, bTagged, Path);
	if (!Status) return Status;
	return bTagged
		? ValidateNamedShape(Value, EGeneratedSchemaKind::Request, Target, Path)
		: ValidateConcreteShape(Value, EGeneratedSchemaKind::Request, Supplied, Path);
}

FURLabWireStatus FURLabOperationSchema::ValidateShape(const FURLabWireValue& Value,
	const URLabWire::TypedResult::FShape& Shape, const FString& Path)
{
	FGeneratedShape Supplied;
	Supplied.Name = Shape.Name;
	Supplied.Fields.Reserve(Shape.Num);
	for (int32 Index = 0; Index < Shape.Num; ++Index)
	{
		const URLabWire::TypedResult::FField& Field = Shape.Fields[Index];
		Supplied.Fields.Add({Field.Name, Field.Type, Field.LengthFrom, Field.Literal,
			Field.Required});
	}
	FString Target;
	bool bTagged = false;
	FURLabWireStatus Status = ResolveTaggedShape(EGeneratedSchemaKind::Result,
		Supplied.Name, Value, Target, bTagged, Path);
	if (!Status) return Status;
	return bTagged
		? ValidateNamedShape(Value, EGeneratedSchemaKind::Result, Target, Path)
		: ValidateConcreteShape(Value, EGeneratedSchemaKind::Result, Supplied, Path);
}
