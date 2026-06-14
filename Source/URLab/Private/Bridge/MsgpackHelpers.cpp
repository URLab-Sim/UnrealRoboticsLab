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

#include "Bridge/MsgpackHelpers.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Misc/Base64.h"

// rpclib's msgpack-cxx headers use member functions and templates named
// `check`, which collides with UE's `check(cond)` assertion macro. Save
// the macro, undef while including rpclib, and restore afterwards. We do
// the same for `verify` for safety.
#pragma push_macro("check")
#pragma push_macro("verify")
#undef check
#undef verify

THIRD_PARTY_INCLUDES_START
#include "rpc/msgpack.hpp"
THIRD_PARTY_INCLUDES_END

#pragma pop_macro("verify")
#pragma pop_macro("check")

// Keys ending in "__b64__" are base64-decoded and emitted as msgpack `bin`;
// the bridge-side decoder strips the suffix. SetBinaryField packs for callers.

namespace
{
    constexpr const TCHAR* kBinSuffix = TEXT("__b64__");
    constexpr int32 kBinSuffixLen = 7;

    template <typename Stream>
    void PackString(clmdep_msgpack::packer<Stream>& Packer, const FString& S)
    {
        FTCHARToUTF8 Conv(*S);
        Packer.pack_str(Conv.Length());
        Packer.pack_str_body(Conv.Get(), Conv.Length());
    }

    template <typename Stream>
    void PackJsonValue(clmdep_msgpack::packer<Stream>& Packer, const TSharedPtr<FJsonValue>& V);

    template <typename Stream>
    void PackJsonObjectInner(clmdep_msgpack::packer<Stream>& Packer,
                             const TSharedPtr<FJsonObject>& Obj)
    {
        if (!Obj.IsValid()) { Packer.pack_map(0); return; }
        Packer.pack_map(Obj->Values.Num());
        for (const auto& Kv : Obj->Values)
        {
            const FString& Key = Kv.Key;
            // Special-case keys with kBinSuffix: emit real msgpack bin rather
            // than the base64-as-string form the JSON tree carries.
            if (Key.EndsWith(kBinSuffix, ESearchCase::CaseSensitive))
            {
                FString StrippedKey = Key.LeftChop(kBinSuffixLen);
                PackString(Packer, StrippedKey);
                FString Encoded;
                if (Kv.Value.IsValid() && Kv.Value->TryGetString(Encoded))
                {
                    TArray<uint8> Decoded;
                    FBase64::Decode(Encoded, Decoded);
                    Packer.pack_bin(Decoded.Num());
                    if (Decoded.Num() > 0)
                        Packer.pack_bin_body(reinterpret_cast<const char*>(Decoded.GetData()), Decoded.Num());
                }
                else
                {
                    Packer.pack_bin(0);
                }
                continue;
            }

            PackString(Packer, Key);
            PackJsonValue(Packer, Kv.Value);
        }
    }

    template <typename Stream>
    void PackJsonValue(clmdep_msgpack::packer<Stream>& Packer, const TSharedPtr<FJsonValue>& V)
    {
        if (!V.IsValid()) { Packer.pack_nil(); return; }

        switch (V->Type)
        {
        case EJson::Null:
            Packer.pack_nil();
            return;
        case EJson::Boolean:
            Packer.pack(V->AsBool());
            return;
        case EJson::Number:
            // Always pack numbers as float64. Bridge side decodes via
            // msgpack and gets Python `float`, which is float64.
            Packer.pack(V->AsNumber());
            return;
        case EJson::String:
        {
            FString S; V->TryGetString(S);
            PackString(Packer, S);
            return;
        }
        case EJson::Array:
        {
            const TArray<TSharedPtr<FJsonValue>>& Arr = V->AsArray();
            Packer.pack_array(Arr.Num());
            for (const auto& E : Arr) PackJsonValue(Packer, E);
            return;
        }
        case EJson::Object:
        {
            PackJsonObjectInner(Packer, V->AsObject());
            return;
        }
        default:
            Packer.pack_nil();
            return;
        }
    }
}

void FURLabMsgpackUtil::PackJsonObject(const TSharedPtr<FJsonObject>& Object, TArray<uint8>& OutBuf)
{
    OutBuf.Reset();
    clmdep_msgpack::sbuffer SBuf;
    clmdep_msgpack::packer<clmdep_msgpack::sbuffer> Packer(SBuf);
    PackJsonObjectInner(Packer, Object);
    OutBuf.Append(reinterpret_cast<const uint8*>(SBuf.data()), (int32)SBuf.size());
}

void FURLabMsgpackUtil::SetBinaryField(TSharedPtr<FJsonObject>& Obj, const FString& Field,
                                  const uint8* Data, int32 Size)
{
    if (!Obj.IsValid()) return;
    FString Encoded;
    if (Size > 0 && Data)
        Encoded = FBase64::Encode(Data, Size);
    // The packer recognises the __b64__ suffix and converts back to msgpack bin.
    Obj->SetStringField(Field + kBinSuffix, Encoded);
}

// =============================================================================
// Unpack: msgpack -> FJsonObject
// =============================================================================

namespace
{
    TSharedPtr<FJsonValue> ConvertMsgpackObject(const clmdep_msgpack::object& O)
    {
        switch (O.type)
        {
        case clmdep_msgpack::type::NIL:
            return MakeShared<FJsonValueNull>();
        case clmdep_msgpack::type::BOOLEAN:
            return MakeShared<FJsonValueBoolean>(O.via.boolean);
        case clmdep_msgpack::type::POSITIVE_INTEGER:
            return MakeShared<FJsonValueNumber>((double)O.via.u64);
        case clmdep_msgpack::type::NEGATIVE_INTEGER:
            return MakeShared<FJsonValueNumber>((double)O.via.i64);
        case clmdep_msgpack::type::FLOAT32:
        case clmdep_msgpack::type::FLOAT64:
            return MakeShared<FJsonValueNumber>(O.via.f64);
        case clmdep_msgpack::type::STR:
        {
            FString OutStr;
            FUTF8ToTCHAR Conv(O.via.str.ptr, O.via.str.size);
            OutStr.AppendChars(Conv.Get(), Conv.Length());
            return MakeShared<FJsonValueString>(OutStr);
        }
        case clmdep_msgpack::type::BIN:
        {
            // Surface bin as base64 inside the JSON tree, with the same
            // __b64__ suffix convention so callers using FJsonObject
            // round-trip correctly. Note: only relevant when the binary
            // appears at a value position; binary at a map *value* gets the
            // suffix tagged onto its key by ConvertMsgpackMap.
            FString Encoded = FBase64::Encode(reinterpret_cast<const uint8*>(O.via.bin.ptr),
                                              (int32)O.via.bin.size);
            return MakeShared<FJsonValueString>(Encoded);
        }
        case clmdep_msgpack::type::ARRAY:
        {
            TArray<TSharedPtr<FJsonValue>> Out;
            Out.Reserve((int32)O.via.array.size);
            for (uint32 i = 0; i < O.via.array.size; ++i)
                Out.Add(ConvertMsgpackObject(O.via.array.ptr[i]));
            return MakeShared<FJsonValueArray>(Out);
        }
        case clmdep_msgpack::type::MAP:
        {
            TSharedPtr<FJsonObject> Inner = MakeShared<FJsonObject>();
            for (uint32 i = 0; i < O.via.map.size; ++i)
            {
                const auto& K = O.via.map.ptr[i].key;
                const auto& V = O.via.map.ptr[i].val;
                if (K.type != clmdep_msgpack::type::STR) continue;
                FString KeyStr;
                {
                    FUTF8ToTCHAR Conv(K.via.str.ptr, K.via.str.size);
                    KeyStr.AppendChars(Conv.Get(), Conv.Length());
                }
                if (V.type == clmdep_msgpack::type::BIN)
                {
                    // Tag the key with __b64__ so PackJsonObject re-emits
                    // it as binary on round-trip.
                    FString Encoded = FBase64::Encode(reinterpret_cast<const uint8*>(V.via.bin.ptr),
                                                      (int32)V.via.bin.size);
                    Inner->SetStringField(KeyStr + kBinSuffix, Encoded);
                }
                else
                {
                    Inner->SetField(KeyStr, ConvertMsgpackObject(V));
                }
            }
            return MakeShared<FJsonValueObject>(Inner);
        }
        case clmdep_msgpack::type::EXT:
        default:
            return MakeShared<FJsonValueNull>();
        }
    }
}

bool FURLabMsgpackUtil::UnpackToJsonObject(const uint8* Data, int32 Size,
                                      TSharedPtr<FJsonObject>& OutObject)
{
    if (!Data || Size <= 0) return false;
    try
    {
        clmdep_msgpack::object_handle Oh =
            clmdep_msgpack::unpack(reinterpret_cast<const char*>(Data), (size_t)Size);
        const clmdep_msgpack::object& Top = Oh.get();
        if (Top.type != clmdep_msgpack::type::MAP) return false;
        TSharedPtr<FJsonValue> V = ConvertMsgpackObject(Top);
        if (!V.IsValid()) return false;
        OutObject = V->AsObject();
        return OutObject.IsValid();
    }
    catch (...)
    {
        return false;
    }
}
