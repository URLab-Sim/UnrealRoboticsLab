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

#include "URLabDmEnvRpc.h"
#include "Transport/MjExternalTransportProvider.h"
#include "Transport/DmEnvRpcTransport.h"
#include "Transport/DmEnvRpcClientTransports.h"
#include "Bridge/BridgeServer.h"

DEFINE_LOG_CATEGORY(LogURLabDmEnvRpc);

namespace
{
/** Factory for the dm_env_rpc control RPC transport, installed as the core's external
 *  control-RPC hook. */
UURLabRpcTransport* MakeDmEnvRpcControlTransport(UURLabBridgeServer* Bridge)
{
	UURLabDmEnvRpcTransport* Transport = NewObject<UURLabDmEnvRpcTransport>(Bridge, NAME_None);
	Transport->SetOwningBridge(Bridge);
	return Transport;
}
} // namespace

// The transport name this module's control RPC transport reports via
// GetTransportName() (UURLabDmEnvRpcTransport). Used as the registration-list key
// so EnsureExternalTransportsBound dedups by name and this module coexists with
// ROS instead of evicting it (H1).
static const FName GDmEnvRpcControlName(TEXT("dm_env_rpc"));
// The endpoint scheme this module's CLIENT transports answer (grpc://host:port).
static const FString GDmEnvRpcScheme(TEXT("grpc"));

void FURLabDmEnvRpcModule::StartupModule()
{
	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] Module starting up. Registering external transport provider."));
	// APPEND (not overwrite) our control-RPC factory into the registration list so
	// ROS and gRPC bind side by side (H1). Remove any stale same-name entry first so
	// a module hot-reload does not double-register.
	FMjExternalTransportProvider::ControlRpcTransportFactories.RemoveAll(
		[](const FMjExternalRpcTransportFactory& R) { return R.TransportName == GDmEnvRpcControlName; });
	FMjExternalTransportProvider::ControlRpcTransportFactories.Add(
		{ GDmEnvRpcControlName, FMjMakeExternalRpcTransport::CreateStatic(&MakeDmEnvRpcControlTransport) });

	// CLIENT-side gRPC transports (the mirror/peek direction): a Mirror dials an
	// owner over gRPC for the model (fastpath_hello), perturbs, and the transform
	// view stream. Registered under the "grpc" scheme so the core's Create selects
	// them for a "grpc://" endpoint -- side by side with any other scheme's factory.
	FMjExternalTransportProvider::RpcClientTransportFactories.Add(
		GDmEnvRpcScheme, FMjMakeExternalRpcClientTransport::CreateStatic(&MakeDmEnvRpcRpcClientTransport));
	FMjExternalTransportProvider::ClientSubscribeTransportFactories.Add(
		GDmEnvRpcScheme, FMjMakeExternalClientSubscribeTransport::CreateStatic(&MakeDmEnvRpcClientSubscribeTransport));
}

void FURLabDmEnvRpcModule::ShutdownModule()
{
	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] Module shutting down."));
	FMjExternalTransportProvider::ControlRpcTransportFactories.RemoveAll(
		[](const FMjExternalRpcTransportFactory& R) { return R.TransportName == GDmEnvRpcControlName; });
	FMjExternalTransportProvider::RpcClientTransportFactories.Remove(GDmEnvRpcScheme);
	FMjExternalTransportProvider::ClientSubscribeTransportFactories.Remove(GDmEnvRpcScheme);
}

IMPLEMENT_MODULE(FURLabDmEnvRpcModule, URLabDmEnvRpc)
