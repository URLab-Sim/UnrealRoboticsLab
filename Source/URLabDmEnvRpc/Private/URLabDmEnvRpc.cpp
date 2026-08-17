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

void FURLabDmEnvRpcModule::StartupModule()
{
	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] Module starting up. Registering external transport provider."));
	FMjExternalTransportProvider::MakeControlRpcTransport.BindStatic(&MakeDmEnvRpcControlTransport);
}

void FURLabDmEnvRpcModule::ShutdownModule()
{
	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] Module shutting down."));
	if (FMjExternalTransportProvider::MakeControlRpcTransport.IsBound())
	{
		FMjExternalTransportProvider::MakeControlRpcTransport.Unbind();
	}
}

IMPLEMENT_MODULE(FURLabDmEnvRpcModule, URLabDmEnvRpc)
