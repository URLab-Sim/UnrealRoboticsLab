// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

/**
 * @class FURLabZmqContext
 * @brief The one libzmq context in the process, reference counted.
 *
 * A ZMQ context is a thread pool and a set of inproc endpoints, not a socket:
 * sockets on one context share its I/O threads, and there is no reason to have
 * more than one per process. Every transport used to call `zmq_ctx_new()` for
 * itself -- eight of them, one PER CAMERA -- so a scene with six streaming
 * cameras paid for eight contexts and their I/O threads to serve nine sockets.
 *
 * Shutdown ordering is the whole subtlety here. `zmq_ctx_term` blocks until
 * every socket on the context is closed, so the context must outlive its
 * sockets: acquire before creating a socket, close the socket, then release.
 * A holder that releases while its socket is open deadlocks the last releaser,
 * which is why Release takes the handle by reference and clears it -- a
 * released handle must never be used to open another socket.
 *
 * Thread safe: a ZMQ context is safe to share across threads, and the refcount
 * is taken under a lock so a camera worker and the game thread may acquire and
 * release concurrently.
 */
class URLAB_API FURLabZmqContext
{
public:
	/** Take a reference to the process context, creating it on the first
	 *  acquire. Returns nullptr only when libzmq cannot create a context, in
	 *  which case no reference was taken. */
	static void* Acquire();

	/** Drop a reference. Terminates the context when the last one goes,
	 *  which requires the caller to have closed its sockets first. `Ctx` is
	 *  cleared so a stale handle cannot be reused. Passing nullptr is a no-op,
	 *  so a failed init can release unconditionally. */
	static void Release(void*& Ctx);

	/** Live reference count. Exposed so a test can prove that N transports
	 *  share one context and that the count returns to zero. */
	static int32 GetRefCount();
};
