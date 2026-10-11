#pragma once

#include "CoreMinimal.h"
#include "Wire/RequestEnvelope.h"
#include "ViewerProducerSession.generated.h"

class UURLabRpcClientTransport;
class FURLabShmRegionAccess;

enum class EURLabViewerProducerDataPolicy : uint8 { Network, Auto, Shm };

struct URLAB_API FURLabViewerProducerConnection
{
	FString CommandUri;
	EURLabViewerProducerDataPolicy DataPolicy = EURLabViewerProducerDataPolicy::Network;
};

struct URLAB_API FURLabViewerProducerRequest
{
	FString Op;
	uint64 RequestId = 0;
	TArray<uint8> Bytes;
	int32 Attempts = 0;
};

enum class EURLabViewerProducerRequestState : uint8 { Pending, Terminal, Failed };

/** Independent read-only typed protocol-v2 session owned by one viewer binding. */
UCLASS()
class URLAB_API UURLabViewerProducerSession : public UObject
{
	GENERATED_BODY()
public:
	/** Game-thread UObject transport construction, separated from blocking I/O. */
	bool Initialize(const FURLabViewerProducerConnection& Connection, FString& OutError);
	/** Blocking protocol connect/negotiation; safe on a worker after Initialize. */
	bool CompleteConnect(int32 TimeoutMs, FString& OutError);
	bool Connect(const FURLabViewerProducerConnection& Connection, int32 TimeoutMs,
		FString& OutError);
	bool Request(const FString& Op, FURLabWireValue Body, int32 TimeoutMs,
		FURLabReplyEnvelope& OutReply, FString& OutError);
	bool BeginRequest(const FString& Op, FURLabWireValue Body,
		FURLabViewerProducerRequest& OutRequest, FString& OutError);
	EURLabViewerProducerRequestState PollRequest(FURLabViewerProducerRequest& Request,
		int32 TimeoutMs, FURLabReplyEnvelope& OutReply, FString& OutError);
	bool Disconnect(int32 TimeoutMs, FString& OutError);
	/** Retire a failed peer without attempting another command round trip. */
	void Abort();
	bool IsConnected() const { return Transport != nullptr && !BootId.IsEmpty() && !SessionId.IsEmpty(); }
	const FString& GetBootId() const { return BootId; }
	const FString& GetSessionId() const { return SessionId; }
	const FString& GetSelectedTransfer() const { return SelectedTransfer; }
	const FURLabConnectReply& GetConnectReply() const { return ConnectReply; }
	TSharedPtr<FURLabShmRegionAccess, ESPMode::ThreadSafe> GetShmAccess() const { return ShmAccess; }
	virtual void BeginDestroy() override;

	static bool ValidateCommandUri(const FString& Uri, FString& OutError);
private:
	bool RoundTrip(const TArray<uint8>& Bytes, int32 TimeoutMs, TArray<uint8>& Out,
		FString& OutError);
	void Reset();

	UPROPERTY(Transient) TObjectPtr<UURLabRpcClientTransport> Transport;
	FURLabConnectReply ConnectReply;
	TSharedPtr<FURLabShmRegionAccess, ESPMode::ThreadSafe> ShmAccess;
	FString BootId;
	FString SessionId;
	FString SelectedTransfer;
	uint64 NextRequestId = 1;
	EURLabViewerProducerDataPolicy PendingPolicy = EURLabViewerProducerDataPolicy::Network;
};
