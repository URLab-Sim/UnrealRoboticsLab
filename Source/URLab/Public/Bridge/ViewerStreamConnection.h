#pragma once
#include "CoreMinimal.h"
#include "Bridge/ViewerProducerSession.h"
#include "Templates/SharedPointer.h"
#include "UObject/StrongObjectPtr.h"
#include "Transport/WorkerThread.h"
#include "Wire/RequestEnvelope.h"
#include "Containers/Queue.h"

struct FURLabViewerStreamDescriptor;
class FURLabLockedLatestFrameReader;
class FURLabPoseStreamZmqReader;

/** Prepared producer command session plus one negotiated latest-frame reader. */
class URLAB_API FURLabViewerStreamConnection : public TSharedFromThis<FURLabViewerStreamConnection,ESPMode::ThreadSafe>
{
public:
	static TSharedPtr<FURLabViewerStreamConnection,ESPMode::ThreadSafe> Create(
		UObject* Outer,const FURLabViewerProducerConnection& Connection,FString& OutError);
	~FURLabViewerStreamConnection();
	/** Blocking worker-thread connect, describe, exact identity match and reader creation. */
	bool Prepare(const FURLabViewerStreamDescriptor& Expected,int32 TimeoutMs,FString& OutError);
	bool PopLatest(TArray<uint8>& OutBytes);
	using FInteractionCallback = TUniqueFunction<void(FURLabReplyEnvelope&&, FString&&)>;
	/** Bounded asynchronous producer command; completion runs on the command worker. */
	bool ForwardInteraction(FURLabWireValue Body, int32 TimeoutMs,
		FInteractionCallback&& Completion, FString& OutError);
	enum class EHealth : uint8 { Healthy, Failed, Successor };
	EHealth GetHealth() const { return static_cast<EHealth>(Health.load(std::memory_order_acquire)); }
	void BeginRetirement();
	bool CanFinalizeRetirement() const;
	void FinalizeRetirement();
	void Shutdown();
private:
	explicit FURLabViewerStreamConnection(TStrongObjectPtr<UURLabViewerProducerSession>&& InSession);
	void StoreLatest(TArray<uint8>&& Bytes);
	void RunShm();
	void RunCommands();
	bool ProbeProducer();
	bool StartCommandWorker(FString& OutError);
	struct FCommand
	{
		FURLabWireValue Body;
		int32 TimeoutMs = 0;
		FInteractionCallback Completion;
	};
	TStrongObjectPtr<UURLabViewerProducerSession> Session;
	TUniquePtr<FURLabPoseStreamZmqReader> ZmqReader;
	TUniquePtr<FURLabLockedLatestFrameReader> ShmReader;
	FURLabWorkerThread ShmWorker;
	FURLabWorkerThread CommandWorker;
	TQueue<FCommand, EQueueMode::Mpsc> Commands;
	std::atomic<int32> PendingCommands{0};
	FCriticalSection InboxMutex;
	TArray<uint8> Latest;
	std::atomic<bool> bStop{false};
	std::atomic<bool> bCommandExited{false};
	std::atomic<bool> bShmExited{true};
	std::atomic<uint8> Health{static_cast<uint8>(EHealth::Healthy)};
	TUniquePtr<FURLabViewerStreamDescriptor> ExpectedDescriptor;
};
using FURLabViewerStreamConnectionPtr=TSharedPtr<FURLabViewerStreamConnection,ESPMode::ThreadSafe>;
