// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Wire/RequestEnvelope.h"
#include "Wire/UrlabWire.gen.h"

/** Recursive validator for generated protocol-v2 operation body shapes. */
class URLAB_API FURLabOperationSchema
{
public:
	/** Validate an active operation's body against its generated request shape. */
	static FURLabWireStatus ValidateRequestBody(const FString& Op,
		const FURLabWireValue& Body);
	/** Validate a handler-produced result against its generated result shape. */
	static FURLabWireStatus ValidateResultBody(const FString& Op,
		const FURLabWireValue& Result);

	/** Validate against a supplied generated-shape declaration (also used by tests). */
	static FURLabWireStatus ValidateShape(const FURLabWireValue& Value,
		const URLabWire::TypedRequest::FShape& Shape, const FString& Path = TEXT("body"));
	static FURLabWireStatus ValidateShape(const FURLabWireValue& Value,
		const URLabWire::TypedResult::FShape& Shape, const FString& Path = TEXT("result"));
};
