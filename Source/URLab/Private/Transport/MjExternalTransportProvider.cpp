// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "Transport/MjExternalTransportProvider.h"

#include "Transport/TransportBase.h"

#include "HAL/PlatformProperties.h"

TArray<FMjExternalRpcTransportFactory> FMjExternalTransportProvider::ControlRpcTransportFactories;
TMap<FString, FMjMakeExternalClientSubscribeTransport> FMjExternalTransportProvider::ClientSubscribeTransportFactories;
TMap<FString, FMjMakeExternalRpcClientTransport> FMjExternalTransportProvider::RpcClientTransportFactories;
FMjMakeExternalCameraPublishTransport FMjExternalTransportProvider::MakeCameraPublishTransport;

bool FMjExternalTransportProvider::HasControlRpcTransport()
{
	return ControlRpcTransportFactories.Num() > 0;
}

FString FMjExternalTransportProvider::SchemeOf(const FString& Endpoint)
{
	const int32 SchemeEnd = Endpoint.Find(TEXT("://"), ESearchCase::CaseSensitive, ESearchDir::FromStart);
	return (SchemeEnd != INDEX_NONE) ? Endpoint.Left(SchemeEnd) : FString();
}

bool FMjExternalTransportProvider::IsBuiltInZmqScheme(const FString& Scheme)
{
	return Scheme.IsEmpty() || Scheme.Equals(TEXT("tcp"), ESearchCase::IgnoreCase);
}

bool FMjExternalTransportProvider::HasClientSubscribeScheme(const FString& Scheme)
{
	const FMjMakeExternalClientSubscribeTransport* Factory = ClientSubscribeTransportFactories.Find(Scheme);
	return Factory != nullptr && Factory->IsBound();
}

bool FMjExternalTransportProvider::HasRpcClientScheme(const FString& Scheme)
{
	const FMjMakeExternalRpcClientTransport* Factory = RpcClientTransportFactories.Find(Scheme);
	return Factory != nullptr && Factory->IsBound();
}

FString FMjExternalTransportProvider::DescribeUnavailableScheme(
	const FString& Scheme, const FString& Endpoint)
{
	// Which optional module owns each non-ZMQ scheme, and where that module ships.
	// URLabDmEnvRpc carries "PlatformAllowList": ["Linux"] in the .uplugin, so on any
	// other platform it is not merely unloaded, it is not built at all -- the operator
	// needs to be told that rather than left to read a zmq_connect error.
	FString Provider;
	if (Scheme.Equals(TEXT("grpc"), ESearchCase::IgnoreCase))
	{
		Provider = TEXT("the optional URLabDmEnvRpc module, which ships on Linux only");
	}
	else if (Scheme.Equals(TEXT("shm"), ESearchCase::IgnoreCase))
	{
		Provider = TEXT("no command module: shm:// is reserved for the built-in "
						"latest-render subscription reader");
	}
	else
	{
		Provider = TEXT("no known module");
	}

	return FString::Printf(
		TEXT("no client transport registered for scheme '%s' (endpoint '%s'): it would come from %s; "
			 "this build is running on %s. Use a 'tcp://host:port' endpoint for the built-in ZMQ backend."),
		*Scheme, *Endpoint, *Provider, UTF8_TO_TCHAR(FPlatformProperties::PlatformName()));
}
