//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCCASPrinter.h"
#include "CASDWARFObject.h"
#include "llvm/DebugInfo/DWARF/DWARFContext.h"
#include "llvm/MC/CAS/MCCASDebugV1.h"
#include "llvm/Support/DataExtractor.h"
#include "llvm/Support/FormatVariadic.h"

using namespace llvm;
using namespace llvm::cas;
using namespace llvm::mccasformats::v1;

namespace {
struct IndentGuard {
  constexpr static int IndentWidth = 2;
  IndentGuard(int &Indent) : Indent{Indent} { Indent += IndentWidth; }
  ~IndentGuard() { Indent -= IndentWidth; }
  int &Indent;
};

bool isDwarfSection(MCObjectProxy MCObj) {
  // Currently, the only way to detect debug sections is through the kind of its
  // children objects. TODO: find a better way to check this.
  // Dwarf Sections have >= 1 references.
  if (MCObj.getNumReferences() == 0)
    return false;

  ObjectRef FirstRef = MCObj.getReference(0);
  const MCSchema &Schema = MCObj.getSchema();
  Expected<MCObjectProxy> FirstMCRef = Schema.get(FirstRef);
  if (!FirstMCRef)
    return false;

  return FirstMCRef->getKindString().contains("debug");
}
} // namespace

MCCASPrinter::MCCASPrinter(PrinterOptions Options, ObjectStore &CAS,
                           raw_ostream &OS)
    : Options(Options), MCSchema(CAS), Indent{0}, OS(OS) {}

MCCASPrinter::~MCCASPrinter() { OS << "\n"; }

Expected<CASDWARFObject>
MCCASPrinter::discoverDwarfSections(cas::ObjectRef CASObj) {
  Expected<MCObjectProxy> MCObj = MCSchema.get(CASObj);
  if (!MCObj)
    return MCObj.takeError();
  CASDWARFObject DWARFObj(MCObj->getSchema());
  if (Options.DwarfDump) {
    if (Error E = DWARFObj.discoverDwarfSections(*MCObj))
      return std::move(E);
    if (Error E = DWARFObj.discoverDebugInfoSection(*MCObj, OS))
      return std::move(E);
  }
  return DWARFObj;
}

Error MCCASPrinter::printMCObject(ObjectRef CASObj, CASDWARFObject &Obj,
                                  DWARFContext *DWARFCtx) {
  // The object identifying the schema is not considered an MCObject, as such we
  // don't attempt to cast or print it.
  if (CASObj == MCSchema.getRootNodeTypeID())
    return Error::success();

  Expected<MCObjectProxy> MCObj = MCSchema.get(CASObj);
  if (!MCObj)
    return MCObj.takeError();
  return printMCObject(*MCObj, Obj, DWARFCtx);
}

Error MCCASPrinter::printMCObject(MCObjectProxy MCObj, CASDWARFObject &Obj,
                                  DWARFContext *DWARFCtx) {
  // Initialize DWARFObj.
  std::unique_ptr<DWARFContext> DWARFContextHolder;
  if (Options.DwarfDump && !DWARFCtx) {
    auto DWARFObj = std::make_unique<CASDWARFObject>(Obj);
    DWARFContextHolder = std::make_unique<DWARFContext>(std::move(DWARFObj));
    DWARFCtx = DWARFContextHolder.get();
  }

  // If only debug sections were requested, skip non-debug sections.
  if (Options.DwarfSectionsOnly && SectionRef::Cast(MCObj) &&
      !isDwarfSection(MCObj))
    return Error::success();

  // Print CAS Id.
  OS.indent(Indent);
  OS << formatv("{0, -15} {1} \n", MCObj.getKindString(), MCObj.getID());
  if (Options.HexDump) {
    auto data = MCObj.getData();
    if (Options.HexDumpOneLine) {
      OS.indent(Indent);
      llvm::interleave(
          data.take_front(data.size()), OS,
          [this](unsigned char c) { OS << llvm::format_hex(c, 4); }, " ");
      OS << "\n";
    } else {
      while (!data.empty()) {
        OS.indent(Indent);
        llvm::interleave(
            data.take_front(8), OS,
            [this](unsigned char c) { OS << llvm::format_hex(c, 4); }, " ");
        OS << "\n";
        data = data.drop_front(data.size() < 8 ? data.size() : 8);
      }
    }
  }

  // Dwarfdump.
  if (DWARFCtx) {
    IndentGuard Guard(Indent);
    if (Error Err = Obj.dump(OS, Indent, *DWARFCtx, MCObj, Options.ShowForm,
                             Options.Verbose))
      return Err;
  }
  return printSimpleNested(MCObj, Obj, DWARFCtx);
}

static Error printAbbrevOffsets(raw_ostream &OS,
                                DebugAbbrevOffsetsRef OffsetsRef) {
  DebugAbbrevOffsetsRefAdaptor Adaptor(OffsetsRef);
  Expected<SmallVector<size_t>> Offsets = Adaptor.decodeOffsets();
  if (!Offsets)
    return Offsets.takeError();
  llvm::interleaveComma(*Offsets, OS);
  OS << "\n";
  return Error::success();
}

Error MCCASPrinter::printDIEAttrs(BinaryStreamReader &Reader,
                                  StringRef DIEData) {
  constexpr auto IsLittleEndian = true;
  constexpr auto AddrSize = 8;
  constexpr auto FormParams =
      dwarf::FormParams{4 /*Version*/, AddrSize, dwarf::DwarfFormat::DWARF32};

  uint64_t AttrAndFormSizes = 0;
  while (true) {
    uint64_t StartOffset = Reader.getOffset();
    uint64_t AttrAsInt;
    if (auto E = Reader.readULEB128(AttrAsInt))
      return E;
    if (AttrAsInt == 0)
      break;
    auto Attr = static_cast<dwarf::Attribute>(AttrAsInt);

    uint64_t FormAsInt;
    if (auto E = Reader.readULEB128(FormAsInt))
      return E;
    auto Form = static_cast<dwarf::Form>(FormAsInt);

    OS.indent(Indent);
    OS << formatv("{0, -30} {1, -25} ", dwarf::AttributeString(Attr),
                  dwarf::FormEncodingString(Form));

    AttrAndFormSizes += Reader.getOffset() - StartOffset;
    if (doesntDedup(Form, Attr)) {
      OS << "<data in separate block>\n";
      continue;
    }

    Expected<uint64_t> FormSize =
        getFormSize(Form, FormParams, DIEData, Reader.getOffset(),
                    IsLittleEndian, AddrSize);
    if (!FormSize)
      return FormSize.takeError();

    ArrayRef<uint8_t> RawBytes;
    if (auto E = Reader.readArray(RawBytes, *FormSize))
      return E;
    OS << '[';
    llvm::interleave(
        RawBytes, OS, [&](uint8_t Char) { OS << utohexstr(Char); }, " ");
    OS << "]\n";
  }
  OS.indent(Indent);
  OS << "Attr + Form size = " << AttrAndFormSizes << "\n";
  return Error::success();
}

static Expected<dwarf::Tag> readTag(BinaryStreamReader &Reader) {
  uint64_t TagAsInt;
  if (auto E = Reader.readULEB128(TagAsInt))
    return E;
  return static_cast<dwarf::Tag>(TagAsInt);
}

static Expected<bool> readHasChildren(BinaryStreamReader &Reader) {
  char HasChildren;
  if (auto E = Reader.readInteger(HasChildren))
    return E;
  return HasChildren;
}

Error MCCASPrinter::printDIERef(BinaryStreamReader &Reader,
                                dwarf::Tag Tag,
                                StringRef DIEData,
                                ArrayRef<DIERef> &CASChildrenStack) {
  Expected<bool> MaybeHasChildren = readHasChildren(Reader);
  if (!MaybeHasChildren)
    return MaybeHasChildren.takeError();
  bool HasChildren = *MaybeHasChildren;

  IndentGuard Guard(Indent);
  if (auto E = printDIEAttrs(Reader, DIEData))
    return E;

  if (!HasChildren)
    return Error::success();

  while (true) {
    Expected<dwarf::Tag> MaybeChildTag = readTag(Reader);
    if (!MaybeChildTag)
      return MaybeChildTag.takeError();

    dwarf::Tag ChildTag = *MaybeChildTag;
    if (ChildTag == dwarf::Tag::DW_TAG_null)
      break;

    if (ChildTag == getTagForDIEInAnotherBlock()) {
      if (auto E = printDIERef(CASChildrenStack.front()))
        return E;
      CASChildrenStack = CASChildrenStack.drop_front();
      continue;
    }

    OS.indent(Indent);
    OS << dwarf::TagString(ChildTag) << "\n";

    if (auto E = printDIERef(Reader, ChildTag, DIEData, CASChildrenStack))
      return E;
  }

  return Error::success();
}

Expected<SmallVector<DIERef>> loadAllChildren(DIERef DIE) {
  SmallVector<DIERef> Children;
  const MCSchema &Schema = DIE.getSchema();
  auto LoadRef = [&](ObjectRef Obj) -> Error {
    auto MaybeLoaded = Schema.get(Obj);
    if (!MaybeLoaded)
      return MaybeLoaded.takeError();
    if (auto DIEChild = DIERef::Cast(*MaybeLoaded)) {
      Children.push_back(*DIEChild);
      return Error::success();
    }
    if (DebugDIEAbbrev::Cast(*MaybeLoaded))
      return Error::success();
    return createStringError(inconvertibleErrorCode(),
                             "Expected DIERef as child");
  };
  if (auto E = DIE.forEachReference(LoadRef))
    return E;
  return Children;
}

Error MCCASPrinter::printDIERef(DIERef Ref) {
  StringRef DIEData = Ref.getData();
  BinaryStreamReader Reader(DIEData, support::endianness::little);
  Expected<SmallVector<DIERef>> MaybeChildren = loadAllChildren(Ref);
  if (!MaybeChildren)
    return MaybeChildren.takeError();
  ArrayRef<DIERef> ChildrenRefs = *MaybeChildren;

  Expected<dwarf::Tag> MaybeTag = readTag(Reader);
  if (!MaybeTag)
    return MaybeTag.takeError();

  // The tag of a fresh block must be meaningful, otherwise we wouldn't have
  // made a new block.
  assert(*MaybeTag != dwarf::Tag::DW_TAG_null &&
         *MaybeTag != getTagForDIEInAnotherBlock());

  OS.indent(Indent);
  OS << dwarf::TagString(*MaybeTag) << " " << Ref.getID().toString() << "\n";

  return printDIERef(Reader, *MaybeTag, DIEData, ChildrenRefs);
}

Error MCCASPrinter::printSimpleNested(MCObjectProxy Ref, CASDWARFObject &Obj,
                                      DWARFContext *DWARFCtx) {
  IndentGuard Guard(Indent);

  if (auto AbbrevOffsetsRef = DebugAbbrevOffsetsRef::Cast(Ref);
      Options.DebugAbbrevOffsets && AbbrevOffsetsRef)
    if (auto E = printAbbrevOffsets(OS, *AbbrevOffsetsRef))
      return E;

  if (auto DIE = DIERef::Cast(Ref); Options.DIERefs && DIE)
    return printDIERef(*DIE);

  return Ref.forEachReference(
      [&](ObjectRef CASObj) { return printMCObject(CASObj, Obj, DWARFCtx); });
}
