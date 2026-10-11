#include "Transport/CameraStreamFrameCodec.h"

#include "Wire/RequestEnvelope.h"

namespace
{
using V = FURLabWireValue;
bool Fail(FString& Error,const TCHAR* Message){Error=Message;return false;}
bool Positive(uint64 V){return V>0&&V<=static_cast<uint64>(MAX_int64);}
bool Nonnegative(uint64 V){return V<=static_cast<uint64>(MAX_int64);}
bool Validate(const FURLabCameraStreamFrame& F,FString& E)
{
	if(F.ProducerBootId.IsEmpty()||F.StreamId.IsEmpty()||F.SourceRevision.IsEmpty()
		||F.CameraName.IsEmpty()||F.Identity.SceneRevision.IsEmpty()||F.Encoding.IsEmpty())
		return Fail(E,TEXT("camera-stream identity fields must be non-empty"));
	if(F.Identity.SceneRevision!=F.SourceRevision||!Positive(F.StreamGeneration)
		||!Positive(F.Sequence)||!Positive(F.CameraConfigurationRevision)
		||!Positive(F.CaptureSerial)||!Nonnegative(F.Identity.FrameId)
		||!Nonnegative(F.Identity.ClockEpoch))return Fail(E,TEXT("camera-stream integer or scene identity is invalid"));
	if((F.Identity.bHasSourceTime&&!FMath::IsFinite(F.Identity.SourceTime))
		||!FMath::IsFinite(F.AcquisitionUnixTime)||F.AcquisitionUnixTime<0.0)
		return Fail(E,TEXT("camera-stream time metadata is invalid"));
	if(F.Width<=0||F.Height<=0||F.StrideBytes<=0||F.Pixels.IsEmpty()
		||int64(F.StrideBytes)*F.Height!=F.Pixels.Num())
		return Fail(E,TEXT("camera-stream dimensions, stride, or pixels are inconsistent"));
	const int32 BytesPerPixel=F.Encoding==TEXT("rgba8")?4
		:F.Encoding==TEXT("depth_f32le")?4:F.Encoding==TEXT("seg_i32x2le")?8:0;
	if(BytesPerPixel==0||F.StrideBytes!=F.Width*BytesPerPixel)
		return Fail(E,TEXT("camera-stream encoding or canonical stride is invalid"));
	return true;
}
const V* Field(const V& M,const TCHAR* N){return M.FindMapValue(N);}
bool String(const V& M,const TCHAR* N,FString& Out){const V* X=Field(M,N);if(!X||X->GetType()!=EURLabWireValueType::String||X->GetString().IsEmpty())return false;Out=X->GetString();return true;}
bool Integer(const V& M,const TCHAR* N,uint64& Out,bool bPositive){const V* X=Field(M,N);if(!X||X->GetType()!=EURLabWireValueType::Int64||X->GetInt()<(bPositive?1:0))return false;Out=static_cast<uint64>(X->GetInt());return true;}
}

bool FURLabCameraStreamFrameCodec::Encode(const FURLabCameraStreamFrame& F,TArray<uint8>& Out,FString& Error)
{
	Error.Reset();if(!Validate(F,Error))return false;
	TArray<FString>K={TEXT("producer_boot_id"),TEXT("stream_id"),TEXT("stream_generation"),TEXT("source_revision"),TEXT("sequence"),TEXT("camera_name"),TEXT("scene_revision"),TEXT("frame_id"),TEXT("clock_epoch"),TEXT("has_source_time")};
	TArray<V>X={V::MakeString(F.ProducerBootId),V::MakeString(F.StreamId),V::MakeInt(F.StreamGeneration),V::MakeString(F.SourceRevision),V::MakeInt(F.Sequence),V::MakeString(F.CameraName),V::MakeString(F.Identity.SceneRevision),V::MakeInt(F.Identity.FrameId),V::MakeInt(F.Identity.ClockEpoch),V::MakeBool(F.Identity.bHasSourceTime)};
	if(F.Identity.bHasSourceTime){K.Add(TEXT("source_time"));X.Add(V::MakeFloat(F.Identity.SourceTime));}
	K.Append({TEXT("acquisition_unix_time"),TEXT("encoding"),TEXT("width"),TEXT("height"),TEXT("stride_bytes"),TEXT("camera_configuration_revision"),TEXT("capture_serial"),TEXT("pixels")});
	X.Append({V::MakeFloat(F.AcquisitionUnixTime),V::MakeString(F.Encoding),V::MakeInt(F.Width),V::MakeInt(F.Height),V::MakeInt(F.StrideBytes),V::MakeInt(F.CameraConfigurationRevision),V::MakeInt(F.CaptureSerial),V::MakeBinary(F.Pixels)});
	const FURLabWireStatus S=FURLabRequestEnvelopeCodec::EncodeValue(V::MakeMap(MoveTemp(K),MoveTemp(X)),Out);if(!S){Error=S.Detail;return false;}return true;
}

bool FURLabCameraStreamFrameCodec::Decode(TConstArrayView<uint8> Bytes,FURLabCameraStreamFrame& Out,FString& Error,int64 MaxPixelBytes)
{
	Error.Reset();V M;const FURLabWireStatus S=FURLabRequestEnvelopeCodec::DecodeValue(Bytes,M);if(!S){Error=S.Detail;return false;}
	const V* Has=Field(M,TEXT("has_source_time"));if(M.GetType()!=EURLabWireValueType::Map||!Has||Has->GetType()!=EURLabWireValueType::Bool)return Fail(Error,TEXT("camera-stream frame header is invalid"));
	const bool bHas=Has->GetBool();if(!M.HasValidMapStorage()||M.GetMapSize()!=(bHas?19:18)||(Field(M,TEXT("source_time"))!=nullptr)!=bHas)return Fail(Error,TEXT("camera-stream field set is invalid"));
	FURLabCameraStreamFrame F;F.Identity.bHasSourceTime=bHas;
	if(!String(M,TEXT("producer_boot_id"),F.ProducerBootId)||!String(M,TEXT("stream_id"),F.StreamId)||!String(M,TEXT("source_revision"),F.SourceRevision)||!String(M,TEXT("camera_name"),F.CameraName)||!String(M,TEXT("scene_revision"),F.Identity.SceneRevision)||!String(M,TEXT("encoding"),F.Encoding)||!Integer(M,TEXT("stream_generation"),F.StreamGeneration,true)||!Integer(M,TEXT("sequence"),F.Sequence,true)||!Integer(M,TEXT("frame_id"),F.Identity.FrameId,false)||!Integer(M,TEXT("clock_epoch"),F.Identity.ClockEpoch,false)||!Integer(M,TEXT("camera_configuration_revision"),F.CameraConfigurationRevision,true)||!Integer(M,TEXT("capture_serial"),F.CaptureSerial,true))return Fail(Error,TEXT("camera-stream identity fields are invalid"));
	const V* Acquisition=Field(M,TEXT("acquisition_unix_time"));const V* Width=Field(M,TEXT("width"));const V* Height=Field(M,TEXT("height"));const V* Stride=Field(M,TEXT("stride_bytes"));const V* Pixels=Field(M,TEXT("pixels"));
	if(!Acquisition||Acquisition->GetType()!=EURLabWireValueType::Float64||!Width||!Height||!Stride||Width->GetType()!=EURLabWireValueType::Int64||Height->GetType()!=EURLabWireValueType::Int64||Stride->GetType()!=EURLabWireValueType::Int64||!Pixels||Pixels->GetType()!=EURLabWireValueType::Binary)return Fail(Error,TEXT("camera-stream image fields are invalid"));
	F.AcquisitionUnixTime=Acquisition->GetFloat();F.Width=Width->GetInt();F.Height=Height->GetInt();F.StrideBytes=Stride->GetInt();F.Pixels=Pixels->GetBinary();
	if(bHas){const V* Time=Field(M,TEXT("source_time"));if(!Time||Time->GetType()!=EURLabWireValueType::Float64)return Fail(Error,TEXT("camera-stream source_time is invalid"));F.Identity.SourceTime=Time->GetFloat();}
	if(MaxPixelBytes<0||F.Pixels.Num()>MaxPixelBytes)return Fail(Error,TEXT("camera-stream pixels exceed the decode bound"));
	if(!Validate(F,Error))return false;Out=MoveTemp(F);return true;
}
