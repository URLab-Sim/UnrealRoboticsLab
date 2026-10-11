#pragma once
#include "CoreMinimal.h"
#include "Transport/WorkerThread.h"
#include <atomic>

/** Exact-binary-topic ZMQ pose reader. One worker owns its socket and drains to newest complete message. */
class URLAB_API FURLabPoseStreamZmqReader
{
public:
	using FOnFrame = TFunction<void(TArray<uint8>&&)>;
	static TUniquePtr<FURLabPoseStreamZmqReader> Create(const FString& Endpoint,
		TArray<uint8> Topic, int64 MaxFrameBytes, FOnFrame OnFrame, FString& OutError);
	~FURLabPoseStreamZmqReader();
	void Stop();
	void RequestStop() { bStop.store(true, std::memory_order_release); }
	bool HasExited() const { return bExited.load(std::memory_order_acquire); }
private:
	FURLabPoseStreamZmqReader(FString&& Endpoint,TArray<uint8>&& Topic,int64 MaxFrameBytes,FOnFrame&& OnFrame);
	void Run();
	FString Endpoint; TArray<uint8> Topic; int64 MaxFrameBytes=0; FOnFrame Callback;
	FURLabWorkerThread Worker; std::atomic<bool> bStop{false}; std::atomic<bool> bReady{false};
	std::atomic<bool> bExited{false}; FEvent* ReadyEvent=nullptr;
};
