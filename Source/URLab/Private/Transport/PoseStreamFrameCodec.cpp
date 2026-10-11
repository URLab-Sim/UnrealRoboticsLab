// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "Transport/PoseStreamFrameCodec.h"

#include "Wire/RequestEnvelope.h"
#include "Misc/ByteSwap.h"

namespace
{
using V = FURLabWireValue;

bool Fail(FString& Error, const TCHAR* Message)
{
	Error = Message;
	return false;
}

bool IsSigned64(uint64 Value)
{
	return Value <= static_cast<uint64>(MAX_int64);
}

bool DynamicNameValid(const FString& Name)
{
	if(Name.IsEmpty())return false;
	FTCHARToUTF8 Utf8(*Name);
	return Utf8.Length()<=1024;
}

bool Validate(const FURLabPoseStreamFrame& Frame, FString& Error)
{
	if (Frame.ProducerBootId.IsEmpty() || Frame.StreamId.IsEmpty()
		|| Frame.SourceRevision.IsEmpty())
	{
		return Fail(Error, TEXT("pose-stream identity fields must be non-empty"));
	}
	if (Frame.StreamGeneration == 0 || Frame.Sequence == 0
		|| !IsSigned64(Frame.StreamGeneration) || !IsSigned64(Frame.Sequence)
		|| !IsSigned64(Frame.FrameId) || !IsSigned64(Frame.ClockEpoch))
	{
		return Fail(Error, TEXT("pose-stream integer identity is outside signed64 bounds"));
	}
	if (Frame.bHasSourceTime && !FMath::IsFinite(Frame.SourceTime))
	{
		return Fail(Error, TEXT("pose-stream source_time must be finite"));
	}
	if (Frame.Payload.IsEmpty() || Frame.Payload.Num() % (7 * sizeof(double)) != 0)
	{
		return Fail(Error, TEXT("pose-stream payload must contain whole 7*f64le transforms"));
	}
	return true;
}

bool ReadString(const V& Map, const TCHAR* Name, FString& Out)
{
	const V* Field = Map.FindMapValue(Name);
	if (!Field || Field->GetType() != EURLabWireValueType::String || Field->GetString().IsEmpty())
	{
		return false;
	}
	Out = Field->GetString();
	return true;
}

bool ReadNonnegative(const V& Map, const TCHAR* Name, uint64& Out, bool bPositive)
{
	const V* Field = Map.FindMapValue(Name);
	if (!Field || Field->GetType() != EURLabWireValueType::Int64
		|| Field->GetInt() < (bPositive ? 1 : 0))
	{
		return false;
	}
	Out = static_cast<uint64>(Field->GetInt());
	return true;
}

void WriteF32(TArray<uint8>& Bytes,float Value){uint32 Bits=0;FMemory::Memcpy(&Bits,&Value,4);Bits=INTEL_ORDER32(Bits);const int32 At=Bytes.AddUninitialized(4);FMemory::Memcpy(Bytes.GetData()+At,&Bits,4);}
bool ReadF32(const TArray<uint8>& Bytes,int32 Offset,float& Out){if(Offset<0||Offset>Bytes.Num()-4)return false;uint32 Bits=0;FMemory::Memcpy(&Bits,Bytes.GetData()+Offset,4);Bits=INTEL_ORDER32(Bits);FMemory::Memcpy(&Out,&Bits,4);return FMath::IsFinite(Out);}
void WriteU32(TArray<uint8>& Bytes,uint32 Value){Value=INTEL_ORDER32(Value);const int32 At=Bytes.AddUninitialized(4);FMemory::Memcpy(Bytes.GetData()+At,&Value,4);}
bool ReadU32(const TArray<uint8>& Bytes,int32 Offset,uint32& Out){if(Offset<0||Offset>Bytes.Num()-4)return false;FMemory::Memcpy(&Out,Bytes.GetData()+Offset,4);Out=INTEL_ORDER32(Out);return true;}

bool EncodeVisualization(const FURLabPoseStreamFrame& Frame,V& Out,FString& Error)
{
	TArray<FString> Keys;TArray<V> Values;
	if(Frame.Deformables.Num()>4096||Frame.DebugMeshes.Num()>4096)return Fail(Error,TEXT("dynamic mesh count exceeds limit"));
	TArray<V> Deformables;
	int64 Total=0;
	for(const auto& D:Frame.Deformables)
	{
		if(!DynamicNameValid(D.Name)||D.Positions.Num()<3||(!D.Normals.IsEmpty()&&D.Normals.Num()!=D.Positions.Num()))return Fail(Error,TEXT("stream deformable shape is invalid"));
		const int64 Bytes=(static_cast<int64>(D.Positions.Num())+D.Normals.Num())*12;
		if(Bytes>128ll*1024*1024-Total)return Fail(Error,TEXT("stream dynamic payload exceeds limit"));
		Total+=Bytes;
		TArray<uint8> P,N;P.Reserve(D.Positions.Num()*12);N.Reserve(D.Normals.Num()*12);
		for(const FVector& X:D.Positions){const FVector Rh(X.X,-X.Y,X.Z);WriteF32(P,Rh.X);WriteF32(P,Rh.Y);WriteF32(P,Rh.Z);}
		for(const FVector& X:D.Normals){const FVector Rh(X.X,-X.Y,X.Z);WriteF32(N,Rh.X);WriteF32(N,Rh.Y);WriteF32(N,Rh.Z);}
		TArray<FString> RK={TEXT("name"),TEXT("vertex_positions_cm")};TArray<V> RV={V::MakeString(D.Name),V::MakeBinary(MoveTemp(P))};
		if(!N.IsEmpty()){RK.Add(TEXT("vertex_normals"));RV.Add(V::MakeBinary(MoveTemp(N)));}
		Deformables.Add(V::MakeMap(MoveTemp(RK),MoveTemp(RV)));
	}
	if(!Deformables.IsEmpty()){Keys.Add(TEXT("deformables"));Values.Add(V::MakeArray(MoveTemp(Deformables)));}
	TArray<V> Meshes;
	for(const auto& M:Frame.DebugMeshes)
	{
		if(!DynamicNameValid(M.Name)||M.Positions.Num()<3||M.Triangles.Num()<3||M.Triangles.Num()%3)return Fail(Error,TEXT("stream debug mesh shape is invalid"));
		const int64 Bytes=static_cast<int64>(M.Positions.Num())*12+static_cast<int64>(M.Triangles.Num())*4;
		if(Bytes>128ll*1024*1024-Total)return Fail(Error,TEXT("stream dynamic payload exceeds limit"));
		Total+=Bytes;
		TArray<uint8>P,I;P.Reserve(M.Positions.Num()*12);I.Reserve(M.Triangles.Num()*4);
		for(const FVector& X:M.Positions){const FVector Rh(X.X,-X.Y,X.Z);WriteF32(P,Rh.X);WriteF32(P,Rh.Y);WriteF32(P,Rh.Z);}
		for(int32 Index:M.Triangles){if(Index<0||Index>=M.Positions.Num())return Fail(Error,TEXT("stream debug mesh index is invalid"));WriteU32(I,Index);}
		TArray<V> Color={V::MakeFloat(M.Color.R),V::MakeFloat(M.Color.G),V::MakeFloat(M.Color.B),V::MakeFloat(M.Color.A)};
		Meshes.Add(V::MakeMap({TEXT("name"),TEXT("rgba"),TEXT("positions_cm"),TEXT("triangles")},{V::MakeString(M.Name),V::MakeArray(MoveTemp(Color)),V::MakeBinary(MoveTemp(P)),V::MakeBinary(MoveTemp(I))}));
	}
	if(!Meshes.IsEmpty()){Keys.Add(TEXT("debug_meshes"));Values.Add(V::MakeArray(MoveTemp(Meshes)));}
	Out=V::MakeMap(MoveTemp(Keys),MoveTemp(Values));return true;
}

bool DecodeVisualization(const V& Wire,FURLabPoseStreamFrame& Frame,FString& Error)
{
	if(Wire.GetType()!=EURLabWireValueType::Map||!Wire.HasValidMapStorage())return Fail(Error,TEXT("stream visualization must be a map"));
	int64 Total=0;
	const V* Ds=Wire.FindMapValue(TEXT("deformables"));if(Ds){if(Ds->GetType()!=EURLabWireValueType::Array||Ds->GetArray().Num()>4096)return Fail(Error,TEXT("stream deformables are invalid"));for(const V& Row:Ds->GetArray()){
		const V* Name=Row.FindMapValue(TEXT("name"));const V* Pos=Row.FindMapValue(TEXT("vertex_positions_cm"));const V* Norm=Row.FindMapValue(TEXT("vertex_normals"));if(!Name||!DynamicNameValid(Name->GetString())||!Pos||Pos->GetType()!=EURLabWireValueType::Binary||Pos->GetBinary().Num()<36||Pos->GetBinary().Num()%12||(Norm&&(Norm->GetType()!=EURLabWireValueType::Binary||Norm->GetBinary().Num()!=Pos->GetBinary().Num())))return Fail(Error,TEXT("stream deformable payload is invalid"));
		const int64 Bytes=static_cast<int64>(Pos->GetBinary().Num())+(Norm?Norm->GetBinary().Num():0);
		if(Bytes>128ll*1024*1024-Total)return Fail(Error,TEXT("stream dynamic payload exceeds limit"));Total+=Bytes;
		FRenderFrame::FDeformable D;D.Name=Name->GetString();const TArray<uint8>& PB=Pos->GetBinary();for(int32 O=0;O<PB.Num();O+=12){float X,Y,Z;if(!ReadF32(PB,O,X)||!ReadF32(PB,O+4,Y)||!ReadF32(PB,O+8,Z))return Fail(Error,TEXT("stream deformable position is non-finite"));D.Positions.Emplace(X,-Y,Z);}if(Norm){const TArray<uint8>& NB=Norm->GetBinary();for(int32 O=0;O<NB.Num();O+=12){float X,Y,Z;if(!ReadF32(NB,O,X)||!ReadF32(NB,O+4,Y)||!ReadF32(NB,O+8,Z))return Fail(Error,TEXT("stream deformable normal is non-finite"));D.Normals.Emplace(X,-Y,Z);}}Frame.Deformables.Add(MoveTemp(D));}}
	const V* Ms=Wire.FindMapValue(TEXT("debug_meshes"));if(Ms){if(Ms->GetType()!=EURLabWireValueType::Array||Ms->GetArray().Num()>4096)return Fail(Error,TEXT("stream debug meshes are invalid"));for(const V& Row:Ms->GetArray()){
		const V* Name=Row.FindMapValue(TEXT("name"));const V* C=Row.FindMapValue(TEXT("rgba"));const V* Pos=Row.FindMapValue(TEXT("positions_cm"));const V* Tri=Row.FindMapValue(TEXT("triangles"));if(!Name||!DynamicNameValid(Name->GetString())||!C||C->GetArray().Num()!=4||!Pos||Pos->GetType()!=EURLabWireValueType::Binary||Pos->GetBinary().Num()<36||Pos->GetBinary().Num()%12||!Tri||Tri->GetType()!=EURLabWireValueType::Binary||Tri->GetBinary().Num()<12||Tri->GetBinary().Num()%12)return Fail(Error,TEXT("stream debug mesh payload is invalid"));
		const int64 Bytes=static_cast<int64>(Pos->GetBinary().Num())+Tri->GetBinary().Num();
		if(Bytes>128ll*1024*1024-Total)return Fail(Error,TEXT("stream dynamic payload exceeds limit"));Total+=Bytes;
		FRenderFrame::FDebugMesh M;M.Name=Name->GetString();float Color[4];for(int32 I=0;I<4;++I){Color[I]=C->GetArray()[I].GetFloat();if(!FMath::IsFinite(Color[I])||Color[I]<0||Color[I]>1)return Fail(Error,TEXT("stream debug mesh color is invalid"));}M.Color=FLinearColor(Color[0],Color[1],Color[2],Color[3]);const TArray<uint8>& PB=Pos->GetBinary();for(int32 O=0;O<PB.Num();O+=12){float X,Y,Z;if(!ReadF32(PB,O,X)||!ReadF32(PB,O+4,Y)||!ReadF32(PB,O+8,Z))return Fail(Error,TEXT("stream debug position is non-finite"));M.Positions.Emplace(X,-Y,Z);}const TArray<uint8>& IB=Tri->GetBinary();for(int32 O=0;O<IB.Num();O+=4){uint32 X;if(!ReadU32(IB,O,X)||X>=uint32(M.Positions.Num()))return Fail(Error,TEXT("stream debug index is invalid"));M.Triangles.Add(X);}Frame.DebugMeshes.Add(MoveTemp(M));}}
	return true;
}
}

bool FURLabPoseStreamFrameCodec::Encode(const FURLabPoseStreamFrame& Frame,
	TArray<uint8>& OutBytes, FString& OutError)
{
	OutError.Reset();
	if (!Validate(Frame, OutError)) return false;

	TArray<FString> Keys = {TEXT("producer_boot_id"), TEXT("stream_id"),
		TEXT("stream_generation"), TEXT("source_revision"), TEXT("sequence"),
		TEXT("frame_id"), TEXT("has_source_time")};
	TArray<V> Values = {V::MakeString(Frame.ProducerBootId), V::MakeString(Frame.StreamId),
		V::MakeInt(static_cast<int64>(Frame.StreamGeneration)), V::MakeString(Frame.SourceRevision),
		V::MakeInt(static_cast<int64>(Frame.Sequence)), V::MakeInt(static_cast<int64>(Frame.FrameId)),
		V::MakeBool(Frame.bHasSourceTime)};
	if (Frame.bHasSourceTime)
	{
		Keys.Add(TEXT("source_time"));
		Values.Add(V::MakeFloat(Frame.SourceTime));
	}
	Keys.Append({TEXT("clock_epoch"), TEXT("payload")});
	Values.Add(V::MakeInt(static_cast<int64>(Frame.ClockEpoch)));
	Values.Add(V::MakeBinary(Frame.Payload));
	if(!Frame.Deformables.IsEmpty()||!Frame.DebugMeshes.IsEmpty())
	{
		V Visualization;if(!EncodeVisualization(Frame,Visualization,OutError))return false;
		Keys.Add(TEXT("visualization"));Values.Add(MoveTemp(Visualization));
	}
	TArray<uint8> Encoded;
	const FURLabWireStatus Status = FURLabRequestEnvelopeCodec::EncodeValue(
		V::MakeMap(MoveTemp(Keys), MoveTemp(Values)), Encoded);
	if (!Status)
	{
		OutError = Status.Detail;
		return false;
	}
	OutBytes = MoveTemp(Encoded);
	return true;
}

bool FURLabPoseStreamFrameCodec::Decode(TConstArrayView<uint8> Bytes,
	FURLabPoseStreamFrame& OutFrame, FString& OutError, int64 ExpectedPayloadBytes)
{
	OutError.Reset();
	V Value;
	const FURLabWireStatus Status = FURLabRequestEnvelopeCodec::DecodeValue(Bytes, Value);
	if (!Status)
	{
		OutError = Status.Detail;
		return false;
	}
	if (Value.GetType() != EURLabWireValueType::Map)
	{
		return Fail(OutError, TEXT("pose-stream frame must be a map"));
	}
	const V* HasTime = Value.FindMapValue(TEXT("has_source_time"));
	if (!HasTime || HasTime->GetType() != EURLabWireValueType::Bool)
	{
		return Fail(OutError, TEXT("pose-stream has_source_time must be bool"));
	}
	const bool bHasTime = HasTime->GetBool();
	const V* Visualization=Value.FindMapValue(TEXT("visualization"));
	const int32 ExpectedFields = (bHasTime ? 10 : 9)+(Visualization?1:0);
	if (!Value.HasValidMapStorage() || Value.GetMapSize() != ExpectedFields
		|| (Value.FindMapValue(TEXT("source_time")) != nullptr) != bHasTime)
	{
		return Fail(OutError, TEXT("pose-stream fields or source_time presence are invalid"));
	}

	FURLabPoseStreamFrame Local;
	Local.bHasSourceTime = bHasTime;
	if (!ReadString(Value, TEXT("producer_boot_id"), Local.ProducerBootId)
		|| !ReadString(Value, TEXT("stream_id"), Local.StreamId)
		|| !ReadString(Value, TEXT("source_revision"), Local.SourceRevision)
		|| !ReadNonnegative(Value, TEXT("stream_generation"), Local.StreamGeneration, true)
		|| !ReadNonnegative(Value, TEXT("sequence"), Local.Sequence, true)
		|| !ReadNonnegative(Value, TEXT("frame_id"), Local.FrameId, false)
		|| !ReadNonnegative(Value, TEXT("clock_epoch"), Local.ClockEpoch, false))
	{
		return Fail(OutError, TEXT("pose-stream identity fields are invalid"));
	}
	if (bHasTime)
	{
		const V* Time = Value.FindMapValue(TEXT("source_time"));
		if (!Time || Time->GetType() != EURLabWireValueType::Float64
			|| !FMath::IsFinite(Time->GetFloat()))
		{
			return Fail(OutError, TEXT("pose-stream source_time must be finite float64"));
		}
		Local.SourceTime = Time->GetFloat();
	}
	const V* Payload = Value.FindMapValue(TEXT("payload"));
	if (!Payload || Payload->GetType() != EURLabWireValueType::Binary)
	{
		return Fail(OutError, TEXT("pose-stream payload must be binary"));
	}
	Local.Payload = Payload->GetBinary();
	if (ExpectedPayloadBytes != INDEX_NONE && Local.Payload.Num() != ExpectedPayloadBytes)
	{
		return Fail(OutError, TEXT("pose-stream payload length does not match source"));
	}
	if(Visualization&&!DecodeVisualization(*Visualization,Local,OutError))return false;
	if (!Validate(Local, OutError)) return false;
	OutFrame = MoveTemp(Local);
	return true;
}
