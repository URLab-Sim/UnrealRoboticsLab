#pragma once

#include "CoreMinimal.h"
#include "mujoco/mujoco.h"

/** RAII wrapper for mjSpec*. */
class URLAB_API FMJSpecHandle
{
public:
	FMJSpecHandle() = default;
	explicit FMJSpecHandle(mjSpec* InSpec) : Spec(InSpec) {}
	~FMJSpecHandle() { Reset(); }

	FMJSpecHandle(const FMJSpecHandle&) = delete;
	FMJSpecHandle& operator=(const FMJSpecHandle&) = delete;

	FMJSpecHandle(FMJSpecHandle&& Other) noexcept : Spec(Other.Spec) { Other.Spec = nullptr; }
	FMJSpecHandle& operator=(FMJSpecHandle&& Other) noexcept
	{
		if (this != &Other) { Reset(); Spec = Other.Spec; Other.Spec = nullptr; }
		return *this;
	}

	void Reset(mjSpec* NewSpec = nullptr)
	{
		if (Spec) { mj_deleteSpec(Spec); }
		Spec = NewSpec;
	}

	mjSpec* Get() const { return Spec; }
	mjSpec* Release() { mjSpec* Tmp = Spec; Spec = nullptr; return Tmp; }
	explicit operator bool() const { return Spec != nullptr; }

private:
	mjSpec* Spec = nullptr;
};

/** RAII wrapper for mjModel*. */
class URLAB_API FMJModelHandle
{
public:
	FMJModelHandle() = default;
	explicit FMJModelHandle(mjModel* InModel) : Model(InModel) {}
	~FMJModelHandle() { Reset(); }

	FMJModelHandle(const FMJModelHandle&) = delete;
	FMJModelHandle& operator=(const FMJModelHandle&) = delete;

	FMJModelHandle(FMJModelHandle&& Other) noexcept : Model(Other.Model) { Other.Model = nullptr; }
	FMJModelHandle& operator=(FMJModelHandle&& Other) noexcept
	{
		if (this != &Other) { Reset(); Model = Other.Model; Other.Model = nullptr; }
		return *this;
	}

	void Reset(mjModel* NewModel = nullptr)
	{
		if (Model) { mj_deleteModel(Model); }
		Model = NewModel;
	}

	mjModel* Get() const { return Model; }
	explicit operator bool() const { return Model != nullptr; }

private:
	mjModel* Model = nullptr;
};

/** RAII wrapper for mjData*. */
class URLAB_API FMJDataHandle
{
public:
	FMJDataHandle() = default;
	explicit FMJDataHandle(mjData* InData) : Data(InData) {}
	~FMJDataHandle() { Reset(); }

	FMJDataHandle(const FMJDataHandle&) = delete;
	FMJDataHandle& operator=(const FMJDataHandle&) = delete;

	FMJDataHandle(FMJDataHandle&& Other) noexcept : Data(Other.Data) { Other.Data = nullptr; }
	FMJDataHandle& operator=(FMJDataHandle&& Other) noexcept
	{
		if (this != &Other) { Reset(); Data = Other.Data; Other.Data = nullptr; }
		return *this;
	}

	void Reset(mjData* NewData = nullptr)
	{
		if (Data) { mj_deleteData(Data); }
		Data = NewData;
	}

	mjData* Get() const { return Data; }
	explicit operator bool() const { return Data != nullptr; }

private:
	mjData* Data = nullptr;
};

/** RAII wrapper for mjVFS*. */
class URLAB_API FMJVFSHandle
{
public:
	FMJVFSHandle() = default;
	~FMJVFSHandle() { Reset(); }

	FMJVFSHandle(const FMJVFSHandle&) = delete;
	FMJVFSHandle& operator=(const FMJVFSHandle&) = delete;

	void Init()
	{
		Reset();
		Vfs = new mjVFS();
		mj_defaultVFS(Vfs);
	}

	void Reset()
	{
		if (Vfs)
		{
			mj_deleteVFS(Vfs);
			delete Vfs;
			Vfs = nullptr;
		}
	}

	mjVFS* Get() const { return Vfs; }
	explicit operator bool() const { return Vfs != nullptr; }

private:
	mjVFS* Vfs = nullptr;
};
