// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjModelSource.h"

#include "Misc/Compression.h"
#include "Misc/Crc.h"
#include "Misc/FileHelper.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjModelSource
{
namespace
{
constexpr int32 MaxMjzEntries = 4096;
constexpr int64 MaxMjzExpandedBytes = 2048ll * 1024 * 1024;
constexpr int32 RawDeflateWindowBits = -15;
const TCHAR* const MjzRootModel = TEXT("model.xml");

/** A MuJoCo error slot, or a fallback when it is empty. Never returns an empty string. */
FString ErrorText(const char* Slot, const TCHAR* Fallback)
{
	return (Slot != nullptr && Slot[0] != '\0') ? FString(UTF8_TO_TCHAR(Slot)) : FString(Fallback);
}

bool Fail(FString& OutError, const FString& Message)
{
	OutError = Message;
	return false;
}

bool ContainsNull(const FString& Value)
{
	for (int32 Index = 0; Index < Value.Len(); ++Index)
	{
		if (Value[Index] == TEXT('\0')) return true;
	}
	return false;
}

bool ContainsNull(const TArray<uint8>& Value)
{
	return Value.Contains(static_cast<uint8>(0));
}

/** Strict UTF-8 decode: false when the bytes do not round-trip, so names compare byte-exactly. */
bool DecodeUtf8(const uint8* Bytes, int32 Length, FString& Out)
{
	const FUTF8ToTCHAR Wide(reinterpret_cast<const ANSICHAR*>(Bytes), Length);
	Out = FString::ConstructFromPtrSize(Wide.Get(), Wide.Length());
	const FTCHARToUTF8 Narrow(*Out, Out.Len());
	return Narrow.Length() == Length && FMemory::Memcmp(Narrow.Get(), Bytes, Length) == 0;
}

/** ASCII-only lowercase, matching the byte-wise tolower MuJoCo's VFS uses for name fallback. */
FString AsciiLower(const FString& Value)
{
	FString Out = Value;
	for (int32 Index = 0; Index < Out.Len(); ++Index)
	{
		const TCHAR Char = Out[Index];
		if (Char >= TEXT('A') && Char <= TEXT('Z'))
		{
			Out[Index] = Char - TEXT('A') + TEXT('a');
		}
	}
	return Out;
}

bool IsSeparator(TCHAR Char)
{
	return Char == TEXT('/') || Char == TEXT('\\');
}

/** The final path component, split on both separators as MuJoCo's VFS does. */
FString BaseName(const FString& Path)
{
	for (int32 Index = Path.Len() - 1; Index >= 0; --Index)
	{
		if (IsSeparator(Path[Index])) return Path.RightChop(Index + 1);
	}
	return Path;
}

/**
 * True for a relative path that cannot leave the VFS root: no NUL, no colon (drive letters,
 * URI schemes, alternate data streams), no leading separator (POSIX root or UNC), no empty or
 * '..' segment, and a file name other than '.'. A directory may end in one separator.
 */
bool IsContainedRelativePath(const FString& Path, bool bDirectory)
{
	if (Path.IsEmpty() || ContainsNull(Path) || Path.Contains(TEXT(":"))) return false;
	int32 Start = 0;
	for (int32 Index = 0; Index <= Path.Len(); ++Index)
	{
		const bool bEnd = Index == Path.Len();
		if (!bEnd && !IsSeparator(Path[Index])) continue;
		const int32 Length = Index - Start;
		if (Length == 0 && !(bDirectory && bEnd && Index > 0)) return false;
		if (Length == 2 && Path[Start] == TEXT('.') && Path[Start + 1] == TEXT('.')) return false;
		if (bEnd && !bDirectory && Length == 1 && Path[Start] == TEXT('.')) return false;
		Start = Index + 1;
	}
	return true;
}

bool ValidateAssetNames(const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	for (const TPair<FString, TArray<uint8>>& Asset : Assets)
	{
		if (!IsContainedRelativePath(Asset.Key, false))
		{
			return Fail(OutError, FString::Printf(
				TEXT("asset name '%s' must be a relative path with no '..', root, drive or NUL"), *Asset.Key));
		}
		if (Asset.Value.Num() == 0)
		{
			return Fail(OutError, FString::Printf(TEXT("asset '%s' is empty"), *Asset.Key));
		}
	}
	return true;
}

/** Mount every named blob into the VFS. No supplied entry may be silently ignored. */
bool MountAssets(mjVFS& Vfs, const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	for (const TPair<FString, TArray<uint8>>& Asset : Assets)
	{
		const int Result = mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Key),
			Asset.Value.GetData(), Asset.Value.Num());
		if (Result != 0)
		{
			return Fail(OutError, FString::Printf(
				TEXT("asset '%s' could not be mounted in the MuJoCo VFS (error %d)"), *Asset.Key, Result));
		}
	}
	return true;
}

enum class EXmlPathKind : uint8
{
	File,
	Directory
};

struct FXmlPathReference
{
	FString Element;
	FString Attribute;
	FString Value;
	EXmlPathKind Kind = EXmlPathKind::File;
};

bool IsXmlSpace(uint8 Char)
{
	return Char == ' ' || Char == '\t' || Char == '\n' || Char == '\v' || Char == '\f' || Char == '\r';
}

bool HasPrefixAt(TConstArrayView<uint8> Text, int32 At, const char* Prefix)
{
	const int32 Length = FCStringAnsi::Strlen(Prefix);
	return At + Length <= Text.Num() && FMemory::Memcmp(Text.GetData() + At, Prefix, Length) == 0;
}

int32 FindAt(TConstArrayView<uint8> Text, int32 From, const char* Needle)
{
	const int32 Length = FCStringAnsi::Strlen(Needle);
	for (int32 At = From; At + Length <= Text.Num(); ++At)
	{
		if (FMemory::Memcmp(Text.GetData() + At, Needle, Length) == 0) return At;
	}
	return INDEX_NONE;
}

void AppendUtf8(TArray<uint8>& Out, uint32 CodePoint)
{
	if (CodePoint < 0x80)
	{
		Out.Add(static_cast<uint8>(CodePoint));
	}
	else if (CodePoint < 0x800)
	{
		Out.Add(static_cast<uint8>(0xC0 | (CodePoint >> 6)));
		Out.Add(static_cast<uint8>(0x80 | (CodePoint & 0x3F)));
	}
	else if (CodePoint < 0x10000)
	{
		Out.Add(static_cast<uint8>(0xE0 | (CodePoint >> 12)));
		Out.Add(static_cast<uint8>(0x80 | ((CodePoint >> 6) & 0x3F)));
		Out.Add(static_cast<uint8>(0x80 | (CodePoint & 0x3F)));
	}
	else
	{
		Out.Add(static_cast<uint8>(0xF0 | (CodePoint >> 18)));
		Out.Add(static_cast<uint8>(0x80 | ((CodePoint >> 12) & 0x3F)));
		Out.Add(static_cast<uint8>(0x80 | ((CodePoint >> 6) & 0x3F)));
		Out.Add(static_cast<uint8>(0x80 | (CodePoint & 0x3F)));
	}
}

/** Resolve one entity body (between '&' and ';') the way tinyxml2 does; false leaves it literal. */
bool DecodeEntity(TConstArrayView<uint8> Body, TArray<uint8>& Out)
{
	static const struct
	{
		const char* Name;
		uint8 Char;
	} Named[] = {{"amp", '&'}, {"lt", '<'}, {"gt", '>'}, {"quot", '"'}, {"apos", '\''}};
	for (const auto& Entry : Named)
	{
		const int32 Length = FCStringAnsi::Strlen(Entry.Name);
		if (Body.Num() == Length && FMemory::Memcmp(Body.GetData(), Entry.Name, Length) == 0)
		{
			Out.Add(Entry.Char);
			return true;
		}
	}
	if (Body.Num() < 2 || Body[0] != '#') return false;
	const bool bHex = Body[1] == 'x' || Body[1] == 'X';
	const int32 First = bHex ? 2 : 1;
	if (First >= Body.Num()) return false;
	uint32 CodePoint = 0;
	for (int32 Index = First; Index < Body.Num(); ++Index)
	{
		const uint8 Char = Body[Index];
		uint32 Digit = 0;
		if (Char >= '0' && Char <= '9') Digit = Char - '0';
		else if (bHex && Char >= 'a' && Char <= 'f') Digit = Char - 'a' + 10;
		else if (bHex && Char >= 'A' && Char <= 'F') Digit = Char - 'A' + 10;
		else return false;
		CodePoint = CodePoint * (bHex ? 16 : 10) + Digit;
		if (CodePoint > 0x10FFFF) return false;
	}
	AppendUtf8(Out, CodePoint);
	return true;
}

bool DecodeAttributeValue(TConstArrayView<uint8> Raw, FString& Out)
{
	TArray<uint8> Bytes;
	Bytes.Reserve(Raw.Num());
	for (int32 Index = 0; Index < Raw.Num();)
	{
		if (Raw[Index] == '&')
		{
			int32 Semicolon = INDEX_NONE;
			for (int32 Probe = Index + 1; Probe < Raw.Num() && Probe <= Index + 12; ++Probe)
			{
				if (Raw[Probe] == ';')
				{
					Semicolon = Probe;
					break;
				}
			}
			if (Semicolon != INDEX_NONE
				&& DecodeEntity(Raw.Slice(Index + 1, Semicolon - Index - 1), Bytes))
			{
				Index = Semicolon + 1;
				continue;
			}
		}
		Bytes.Add(Raw[Index]);
		++Index;
	}
	return DecodeUtf8(Bytes.GetData(), Bytes.Num(), Out);
}

bool ClassifyPathAttribute(const FString& Attribute, EXmlPathKind& OutKind)
{
	const FString Lower = AsciiLower(Attribute);
	if (Lower.StartsWith(TEXT("file"), ESearchCase::CaseSensitive))
	{
		OutKind = EXmlPathKind::File;
		return true;
	}
	if (Lower == TEXT("meshdir") || Lower == TEXT("texturedir") || Lower == TEXT("assetdir"))
	{
		OutKind = EXmlPathKind::Directory;
		return true;
	}
	return false;
}

/**
 * Collect every path-bearing attribute (file*, meshdir, texturedir, assetdir) of one XML
 * document without trusting MuJoCo. Comments, CDATA, declarations and DTD-style markup are
 * skipped exactly where tinyxml2 skips them; entity references are decoded as tinyxml2 does.
 */
bool ScanXmlPathReferences(TConstArrayView<uint8> Text, const FString& DocumentName,
	TArray<FXmlPathReference>& Out, FString& OutError)
{
	const auto Malformed = [&OutError, &DocumentName](const TCHAR* What)
	{
		return Fail(OutError, FString::Printf(TEXT("%s is not well-formed XML (%s)"), *DocumentName, What));
	};
	int32 At = 0;
	while (At < Text.Num())
	{
		if (Text[At] != '<')
		{
			++At;
			continue;
		}
		if (HasPrefixAt(Text, At, "<!--"))
		{
			const int32 End = FindAt(Text, At + 4, "-->");
			if (End == INDEX_NONE) return Malformed(TEXT("unterminated comment"));
			At = End + 3;
			continue;
		}
		if (HasPrefixAt(Text, At, "<![CDATA["))
		{
			const int32 End = FindAt(Text, At + 9, "]]>");
			if (End == INDEX_NONE) return Malformed(TEXT("unterminated CDATA"));
			At = End + 3;
			continue;
		}
		if (HasPrefixAt(Text, At, "<?"))
		{
			const int32 End = FindAt(Text, At + 2, "?>");
			if (End == INDEX_NONE) return Malformed(TEXT("unterminated declaration"));
			At = End + 2;
			continue;
		}
		if (HasPrefixAt(Text, At, "<!") || HasPrefixAt(Text, At, "</"))
		{
			const int32 End = FindAt(Text, At + 2, ">");
			if (End == INDEX_NONE) return Malformed(TEXT("unterminated markup"));
			At = End + 1;
			continue;
		}

		int32 Cursor = At + 1;
		const int32 NameStart = Cursor;
		while (Cursor < Text.Num() && !IsXmlSpace(Text[Cursor]) && Text[Cursor] != '/' && Text[Cursor] != '>')
		{
			++Cursor;
		}
		FString Element;
		if (Cursor == NameStart || !DecodeUtf8(Text.GetData() + NameStart, Cursor - NameStart, Element))
		{
			return Malformed(TEXT("bad element name"));
		}
		for (;;)
		{
			while (Cursor < Text.Num() && IsXmlSpace(Text[Cursor])) ++Cursor;
			if (Cursor >= Text.Num()) return Malformed(TEXT("unterminated element"));
			if (Text[Cursor] == '>')
			{
				At = Cursor + 1;
				break;
			}
			if (Text[Cursor] == '/')
			{
				if (Cursor + 1 >= Text.Num() || Text[Cursor + 1] != '>') return Malformed(TEXT("bad empty element"));
				At = Cursor + 2;
				break;
			}
			const int32 AttributeStart = Cursor;
			while (Cursor < Text.Num() && !IsXmlSpace(Text[Cursor]) && Text[Cursor] != '='
				&& Text[Cursor] != '>' && Text[Cursor] != '/')
			{
				++Cursor;
			}
			FString Attribute;
			if (Cursor == AttributeStart
				|| !DecodeUtf8(Text.GetData() + AttributeStart, Cursor - AttributeStart, Attribute))
			{
				return Malformed(TEXT("bad attribute name"));
			}
			while (Cursor < Text.Num() && IsXmlSpace(Text[Cursor])) ++Cursor;
			if (Cursor >= Text.Num() || Text[Cursor] != '=') return Malformed(TEXT("attribute without value"));
			++Cursor;
			while (Cursor < Text.Num() && IsXmlSpace(Text[Cursor])) ++Cursor;
			if (Cursor >= Text.Num() || (Text[Cursor] != '"' && Text[Cursor] != '\''))
			{
				return Malformed(TEXT("unquoted attribute value"));
			}
			const uint8 Quote = Text[Cursor++];
			const int32 ValueStart = Cursor;
			while (Cursor < Text.Num() && Text[Cursor] != Quote) ++Cursor;
			if (Cursor >= Text.Num()) return Malformed(TEXT("unterminated attribute value"));
			const TConstArrayView<uint8> RawValue = Text.Slice(ValueStart, Cursor - ValueStart);
			++Cursor;

			EXmlPathKind Kind;
			if (!ClassifyPathAttribute(Attribute, Kind)) continue;
			FXmlPathReference& Reference = Out.AddDefaulted_GetRef();
			Reference.Element = Element;
			Reference.Attribute = Attribute;
			Reference.Kind = Kind;
			if (!DecodeAttributeValue(RawValue, Reference.Value))
			{
				return Fail(OutError, FString::Printf(TEXT("%s: <%s %s=...> is not valid UTF-8"),
					*DocumentName, *Element, *Attribute));
			}
		}
	}
	return true;
}

/**
 * Prove before parsing that MuJoCo can only ever resolve this model's files from the VFS.
 * MuJoCo's VFS falls back to the server filesystem for any name it cannot match, and matches
 * unknown paths by case-insensitive final component. So every file reference must be a
 * contained relative path whose final component names an uploaded asset; then no lookup
 * (parse-time include/model, or compile-time mesh/texture/hfield/skin) can reach the disk.
 * Included and sub-model XML documents are scanned the same way, recursively.
 */
bool ValidateVfsOnlyReferences(const TArray<uint8>& RootXml,
	const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	TMap<FString, TArray<const FString*>> AssetsByBaseName;
	for (const TPair<FString, TArray<uint8>>& Asset : Assets)
	{
		AssetsByBaseName.FindOrAdd(AsciiLower(BaseName(Asset.Key))).Add(&Asset.Key);
	}

	struct FPendingDocument
	{
		FString Name;
		TConstArrayView<uint8> Text;
	};
	TArray<FPendingDocument> Pending;
	Pending.Add({TEXT("model XML"), RootXml});
	TSet<const FString*> Scanned;
	while (!Pending.IsEmpty())
	{
		const FPendingDocument Document = Pending.Pop(EAllowShrinking::No);
		TArray<FXmlPathReference> References;
		if (!ScanXmlPathReferences(Document.Text, Document.Name, References, OutError)) return false;
		for (const FXmlPathReference& Reference : References)
		{
			if (Reference.Kind == EXmlPathKind::Directory)
			{
				if (!Reference.Value.IsEmpty() && !IsContainedRelativePath(Reference.Value, true))
				{
					return Fail(OutError, FString::Printf(
						TEXT("%s: %s='%s' must be a relative directory with no '..', root or drive"),
						*Document.Name, *Reference.Attribute, *Reference.Value));
				}
				continue;
			}
			if (!IsContainedRelativePath(Reference.Value, false))
			{
				return Fail(OutError, FString::Printf(
					TEXT("%s: <%s %s='%s'> must be a relative path with no '..', root or drive"),
					*Document.Name, *Reference.Element, *Reference.Attribute, *Reference.Value));
			}
			const TArray<const FString*>* Matches =
				AssetsByBaseName.Find(AsciiLower(BaseName(Reference.Value)));
			if (Matches == nullptr)
			{
				return Fail(OutError, FString::Printf(
					TEXT("%s: <%s %s='%s'> names no uploaded asset; the server never reads its own filesystem"),
					*Document.Name, *Reference.Element, *Reference.Attribute, *Reference.Value));
			}
			const bool bXmlDocument = Reference.Element.Equals(TEXT("include"), ESearchCase::IgnoreCase)
				|| Reference.Element.Equals(TEXT("model"), ESearchCase::IgnoreCase);
			if (!bXmlDocument) continue;
			for (const FString* Key : *Matches)
			{
				if (Scanned.Contains(Key)) continue;
				Scanned.Add(Key);
				Pending.Add({FString::Printf(TEXT("asset '%s'"), **Key), Assets.FindChecked(*Key)});
			}
		}
	}
	return true;
}

/** Compile a spec against its VFS and take ownership of the spec either way (compile does not). */
mjModel* CompileAndDeleteSpec(mjSpec* Spec, const mjVFS* Vfs, FString& OutError)
{
	mjModel* const Model = mj_compile(Spec, Vfs);
	if (Model == nullptr)
	{
		OutError = ErrorText(mjs_getError(Spec), TEXT("mj_compile failed"));
	}
	mj_deleteSpec(Spec);
	return Model;
}

/** Prebuilt buffer: load straight in. Version-locked to this libmujoco. */
mjModel* FromMjb(const TArray<uint8>& Bytes, FString& OutError)
{
	mjModel* const Model = mj_loadModelBuffer(Bytes.GetData(), Bytes.Num());
	if (Model == nullptr)
	{
		OutError = TEXT("mj_loadModelBuffer failed (version-mismatched MJB?)");
	}
	return Model;
}

/** XML text + referenced asset bytes: parse to a spec against a VFS of the assets, then compile. */
mjModel* FromXml(const TArray<uint8>& Bytes, const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	if (ContainsNull(Bytes))
	{
		OutError = TEXT("XML source contains an embedded NUL byte");
		return nullptr;
	}
	if (!ValidateAssetNames(Assets, OutError) || !ValidateVfsOnlyReferences(Bytes, Assets, OutError))
	{
		return nullptr;
	}

	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	if (!MountAssets(Vfs, Assets, OutError))
	{
		mj_deleteVFS(&Vfs);
		return nullptr;
	}

	// mj_parseXMLString wants a NUL-terminated C string; the wire bytes carry no terminator.
	TArray<uint8> Terminated(Bytes);
	Terminated.Add(0);

	char Error[1024] = {0};
	mjSpec* const Spec = mj_parseXMLString(
		reinterpret_cast<const char*>(Terminated.GetData()), &Vfs, Error, sizeof(Error));
	if (Spec == nullptr)
	{
		OutError = ErrorText(Error, TEXT("mj_parseXMLString failed"));
		mj_deleteVFS(&Vfs);
		return nullptr;
	}

	mjModel* const Model = CompileAndDeleteSpec(Spec, &Vfs, OutError);
	mj_deleteVFS(&Vfs);
	return Model;
}

uint16 ReadU16(const uint8* At)
{
	return static_cast<uint16>(At[0] | (At[1] << 8));
}

uint32 ReadU32(const uint8* At)
{
	return static_cast<uint32>(At[0]) | (static_cast<uint32>(At[1]) << 8)
		| (static_cast<uint32>(At[2]) << 16) | (static_cast<uint32>(At[3]) << 24);
}

/** Expand a stored/deflated single-volume zip in memory. Entry names obey the asset-name rule. */
bool ExtractMjz(const TArray<uint8>& Archive, TMap<FString, TArray<uint8>>& OutEntries, FString& OutError)
{
	constexpr uint32 EndOfDirectorySignature = 0x06054b50;
	constexpr uint32 DirectoryEntrySignature = 0x02014b50;
	constexpr uint32 LocalHeaderSignature = 0x04034b50;
	constexpr int64 EndOfDirectoryBytes = 22;
	constexpr int64 DirectoryEntryBytes = 46;
	constexpr int64 LocalHeaderBytes = 30;

	const uint8* const Data = Archive.GetData();
	const int64 Size = Archive.Num();
	int64 EndOfDirectory = INDEX_NONE;
	for (int64 At = Size - EndOfDirectoryBytes; At >= 0 && At >= Size - EndOfDirectoryBytes - MAX_uint16; --At)
	{
		if (ReadU32(Data + At) == EndOfDirectorySignature)
		{
			EndOfDirectory = At;
			break;
		}
	}
	if (EndOfDirectory == INDEX_NONE) return Fail(OutError, TEXT("mjz is not a zip archive"));

	const uint8* const End = Data + EndOfDirectory;
	const uint16 EntryCount = ReadU16(End + 10);
	const uint32 DirectorySize = ReadU32(End + 12);
	const uint32 DirectoryOffset = ReadU32(End + 16);
	if (ReadU16(End + 4) != 0 || ReadU16(End + 6) != 0 || ReadU16(End + 8) != EntryCount)
		return Fail(OutError, TEXT("multi-volume mjz archives are not supported"));
	if (EntryCount == MAX_uint16 || DirectorySize == MAX_uint32 || DirectoryOffset == MAX_uint32)
		return Fail(OutError, TEXT("zip64 mjz archives are not supported"));
	if (EntryCount > MaxMjzEntries)
		return Fail(OutError, FString::Printf(TEXT("mjz has more than %d entries"), MaxMjzEntries));
	const int64 DirectoryEnd = static_cast<int64>(DirectoryOffset) + DirectorySize;
	if (DirectoryEnd > EndOfDirectory) return Fail(OutError, TEXT("mjz central directory is out of bounds"));

	int64 ExpandedTotal = 0;
	int64 Cursor = DirectoryOffset;
	for (int32 Index = 0; Index < EntryCount; ++Index)
	{
		if (Cursor + DirectoryEntryBytes > DirectoryEnd || ReadU32(Data + Cursor) != DirectoryEntrySignature)
			return Fail(OutError, TEXT("mjz central directory is corrupt"));
		const uint8* const Entry = Data + Cursor;
		const uint16 Flags = ReadU16(Entry + 8);
		const uint16 Method = ReadU16(Entry + 10);
		const uint32 Crc = ReadU32(Entry + 16);
		const uint32 CompressedSize = ReadU32(Entry + 20);
		const uint32 ExpandedSize = ReadU32(Entry + 24);
		const uint16 NameLength = ReadU16(Entry + 28);
		const int64 Next = Cursor + DirectoryEntryBytes + NameLength + ReadU16(Entry + 30) + ReadU16(Entry + 32);
		const uint32 LocalOffset = ReadU32(Entry + 42);
		if (Next > DirectoryEnd) return Fail(OutError, TEXT("mjz central directory is corrupt"));
		FString Name;
		if (!DecodeUtf8(Entry + DirectoryEntryBytes, NameLength, Name))
			return Fail(OutError, TEXT("mjz entry name is not valid UTF-8"));
		Cursor = Next;
		if (Name.EndsWith(TEXT("/")) || ExpandedSize == 0) continue;

		if (!IsContainedRelativePath(Name, false))
		{
			return Fail(OutError, FString::Printf(
				TEXT("mjz entry '%s' must be a relative path with no '..', root, drive or NUL"), *Name));
		}
		if ((Flags & 1) != 0)
			return Fail(OutError, FString::Printf(TEXT("mjz entry '%s' is encrypted"), *Name));
		if (Method != 0 && Method != 8)
			return Fail(OutError, FString::Printf(TEXT("mjz entry '%s' uses unsupported compression %d"), *Name, Method));
		if (ExpandedSize > static_cast<uint32>(MAX_int32) || ExpandedTotal + ExpandedSize > MaxMjzExpandedBytes)
			return Fail(OutError, FString::Printf(TEXT("mjz expands beyond %lld bytes"), MaxMjzExpandedBytes));
		if (OutEntries.Contains(Name))
			return Fail(OutError, FString::Printf(TEXT("mjz entry '%s' is duplicated"), *Name));

		if (static_cast<int64>(LocalOffset) + LocalHeaderBytes > DirectoryOffset
			|| ReadU32(Data + LocalOffset) != LocalHeaderSignature)
		{
			return Fail(OutError, FString::Printf(TEXT("mjz entry '%s' has a corrupt local header"), *Name));
		}
		const int64 DataOffset = static_cast<int64>(LocalOffset) + LocalHeaderBytes
			+ ReadU16(Data + LocalOffset + 26) + ReadU16(Data + LocalOffset + 28);
		if (DataOffset + CompressedSize > DirectoryOffset)
			return Fail(OutError, FString::Printf(TEXT("mjz entry '%s' is out of bounds"), *Name));

		TArray<uint8> Bytes;
		Bytes.SetNumUninitialized(static_cast<int32>(ExpandedSize));
		if (Method == 0)
		{
			if (CompressedSize != ExpandedSize)
				return Fail(OutError, FString::Printf(TEXT("mjz entry '%s' has inconsistent sizes"), *Name));
			FMemory::Memcpy(Bytes.GetData(), Data + DataOffset, ExpandedSize);
		}
		else if (!FCompression::UncompressMemory(NAME_Zlib, Bytes.GetData(), ExpandedSize,
			Data + DataOffset, CompressedSize, COMPRESS_NoFlags,
			static_cast<uintptr_t>(static_cast<intptr_t>(RawDeflateWindowBits))))
		{
			return Fail(OutError, FString::Printf(TEXT("mjz entry '%s' could not be inflated"), *Name));
		}
		if (FCrc::MemCrc32(Bytes.GetData(), Bytes.Num()) != Crc)
			return Fail(OutError, FString::Printf(TEXT("mjz entry '%s' failed its CRC check"), *Name));
		ExpandedTotal += ExpandedSize;
		OutEntries.Add(MoveTemp(Name), MoveTemp(Bytes));
	}
	return true;
}

/**
 * MuJoCo archive (.mjz) from memory: expand it without touching disk, then compile its
 * model.xml as an XML source whose VFS is the archive's entries plus any extra assets.
 */
mjModel* FromMjz(const TArray<uint8>& Bytes, const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	TMap<FString, TArray<uint8>> Entries;
	if (!ExtractMjz(Bytes, Entries, OutError)) return nullptr;
	TArray<uint8> RootXml;
	if (!Entries.RemoveAndCopyValue(MjzRootModel, RootXml))
	{
		OutError = TEXT("mjz archive has no root model.xml");
		return nullptr;
	}
	for (const TPair<FString, TArray<uint8>>& Asset : Assets)
	{
		if (Entries.Contains(Asset.Key) || Asset.Key == MjzRootModel)
		{
			OutError = FString::Printf(TEXT("asset '%s' collides with an mjz archive entry"), *Asset.Key);
			return nullptr;
		}
		Entries.Add(Asset.Key, Asset.Value);
	}
	return FromXml(RootXml, Entries, OutError);
}
} // namespace

mjModel* FromBytes(const TArray<uint8>& Bytes, const FString& Format,
	const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	OutError.Reset();
	if (Bytes.Num() == 0)
	{
		OutError = TEXT("empty model source buffer");
		return nullptr;
	}
	if (Format.Equals(TEXT("mjb"), ESearchCase::IgnoreCase))
	{
		return FromMjb(Bytes, OutError);
	}
	if (Format.Equals(TEXT("xml"), ESearchCase::IgnoreCase))
	{
		return FromXml(Bytes, Assets, OutError);
	}
	if (Format.Equals(TEXT("mjz"), ESearchCase::IgnoreCase))
	{
		return FromMjz(Bytes, Assets, OutError);
	}
	OutError = FString::Printf(TEXT("unknown model source format '%s' (expected mjb|xml|mjz)"), *Format);
	return nullptr;
}

bool CompileFileToMjb(const FString& Path, const FString& Format,
	TArray<uint8>& OutMjb, FString& OutError)
{
	OutError.Reset();
	OutMjb.Reset();

	mjModel* Model = nullptr;
	if (Format.Equals(TEXT("xml"), ESearchCase::IgnoreCase))
	{
		// Disk parse+compile: mj_loadXML resolves meshdir/texturedir and every
		// <include> relative to the file's own directory, so a normal on-disk scene
		// (e.g. a menagerie scene.xml with an assets/ subdir) needs no VFS mounting.
		char Error[1024] = {0};
		Model = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
		if (Model == nullptr)
		{
			OutError = ErrorText(Error, TEXT("mj_loadXML failed"));
			return false;
		}
	}
	else if (Format.Equals(TEXT("mjz"), ESearchCase::IgnoreCase)
		|| Format.Equals(TEXT("mjb"), ESearchCase::IgnoreCase))
	{
		TArray<uint8> Bytes;
		if (!FFileHelper::LoadFileToArray(Bytes, *Path))
		{
			OutError = FString::Printf(TEXT("could not read %s '%s'"), *Format.ToLower(), *Path);
			return false;
		}
		Model = FromBytes(Bytes, Format, TMap<FString, TArray<uint8>>(), OutError);
		if (Model == nullptr)
		{
			return false;
		}
	}
	else
	{
		OutError = FString::Printf(TEXT("unknown model file format '%s' (expected mjb|xml|mjz)"), *Format);
		return false;
	}

	const mjtSize WideSize = mj_sizeModel(Model);
	if (WideSize <= 0 || WideSize > MAX_int32)
	{
		mj_deleteModel(Model);
		OutError = TEXT("compiled model size exceeds TArray limits");
		return false;
	}
	const int32 Size = static_cast<int32>(WideSize);
	OutMjb.SetNumUninitialized(Size);
	mj_saveModel(Model, nullptr, OutMjb.GetData(), Size);
	mj_deleteModel(Model);
	if (OutMjb.Num() == 0)
	{
		OutError = TEXT("compiled model serialized to zero bytes");
		return false;
	}
	return true;
}
} // namespace MjModelSource
