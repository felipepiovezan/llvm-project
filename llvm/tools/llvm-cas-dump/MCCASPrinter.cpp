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

struct DIEPrinter {
  static Error printDIETopLevelRef(DIETopLevelRef Ref, raw_ostream &OS,
                                   int Indent) {
    Expected<LoadedDIETopLevel> LoadedRef = loadDIETopLevel(Ref);
    if (!LoadedRef)
      return LoadedRef.takeError();

    StringRef DistinctData = LoadedRef->DistinctData.getData();
    if (DistinctData.size() < Dwarf4HeaderSize32Bit)
      return createStringError(inconvertibleErrorCode(),
                               "DistinctData must contain a CU Header");

    StringRef DistinctNoHeader = DistinctData.drop_front(Dwarf4HeaderSize32Bit);
    return DIEPrinter(OS, Indent, LoadedRef->AbbrevEntries, DistinctNoHeader)
        .printDIERef(LoadedRef->RootDIE);
  };

private:
  DIEPrinter(raw_ostream &OS, int Indent, ArrayRef<DIEAbbrevRef> AbbrevEntries,
             StringRef DistinctData)
      : OS(OS), Indent(Indent), AbbrevEntries(AbbrevEntries),
        DistinctReader(DistinctData, support::endianness::little) {}

  Error printDIERef(DIEDataRef Ref);
  Error printDIERef(BinaryStreamReader &DataReader, unsigned AbbrevIdx,
                    StringRef DIEData, ArrayRef<DIEDataRef> &DIEChildrenStack);
  Error printDIEAttrs(AbbrevEntryReader &AbbrevReader,
                      BinaryStreamReader &Reader, StringRef DIEData);

  AbbrevEntryReader getAbbrevEntryReader(unsigned AbbrevIdx) {
    StringRef AbbrevData =
        AbbrevEntries[decodeAbbrevIndex(AbbrevIdx)].getData();
    return AbbrevEntryReader(AbbrevData);
  }

  raw_ostream &OS;
  int Indent;
  ArrayRef<DIEAbbrevRef> AbbrevEntries;
  BinaryStreamReader DistinctReader;
};

Error DIEPrinter::printDIEAttrs(AbbrevEntryReader &AbbrevReader,
                                BinaryStreamReader &DataReader,
                                StringRef DIEData) {
  constexpr auto IsLittleEndian = true;
  constexpr auto AddrSize = 8;
  constexpr auto FormParams =
      dwarf::FormParams{4 /*Version*/, AddrSize, dwarf::DwarfFormat::DWARF32};

  while (true) {
    Expected<dwarf::Attribute> Attr = AbbrevReader.readAttr();
    if (!Attr)
      return Attr.takeError();
    if (*Attr == getEndOfAttributesMarker())
      break;

    Expected<dwarf::Form> Form = AbbrevReader.readForm();
    if (!Form)
      return Form.takeError();

    OS.indent(Indent);
    OS << formatv("{0, -30} {1, -25} ", dwarf::AttributeString(*Attr),
                  dwarf::FormEncodingString(*Form));

    Expected<uint64_t> FormSize =
        getFormSize(*Form, FormParams, DIEData, DataReader.getOffset(),
                    IsLittleEndian, AddrSize);
    if (!FormSize)
      return FormSize.takeError();

    if (doesntDedup(*Form, *Attr)) {
      if (auto E = DistinctReader.skip(*FormSize))
        return E;
      OS << "<data in separate block>\n";
      continue;
    }

    ArrayRef<uint8_t> RawBytes;
    if (auto E = DataReader.readArray(RawBytes, *FormSize))
      return E;
    OS << '[';
    llvm::interleave(
        RawBytes, OS, [&](uint8_t Char) { OS << utohexstr(Char); }, " ");
    OS << "]\n";
  }
  return Error::success();
}

Expected<uint64_t> readAbbrevIdx(BinaryStreamReader &Reader) {
  uint64_t Idx;
  if (auto E = Reader.readULEB128(Idx))
    return E;
  return Idx;
}

Error DIEPrinter::printDIERef(BinaryStreamReader &DataReader,
                              unsigned AbbrevIdx, StringRef DIEData,
                              ArrayRef<DIEDataRef> &DIEChildrenStack) {
  AbbrevEntryReader AbbrevReader = getAbbrevEntryReader(AbbrevIdx);

  if (Expected<dwarf::Tag> MaybeTag = AbbrevReader.readTag())
    OS.indent(Indent) << dwarf::TagString(*MaybeTag) << "\n";
  else
    return MaybeTag.takeError();

  Expected<bool> MaybeHasChildren = AbbrevReader.readHasChildren();
  if (!MaybeHasChildren)
    return MaybeHasChildren.takeError();

  IndentGuard Guard(Indent);
  if (auto E = printDIEAttrs(AbbrevReader, DataReader, DIEData))
    return E;

  if (!*MaybeHasChildren)
    return Error::success();

  while (true) {
    Expected<uint64_t> ChildAbbrevIdx = readAbbrevIdx(DistinctReader);
    if (!ChildAbbrevIdx)
      return ChildAbbrevIdx.takeError();

    if (*ChildAbbrevIdx == getEndOfDIESiblingsMarker())
      break;

    if (*ChildAbbrevIdx == getDIEInAnotherBlockMarker()) {
      if (auto E = printDIERef(DIEChildrenStack.front()))
        return E;
      DIEChildrenStack = DIEChildrenStack.drop_front();
      continue;
    }

    if (auto E =
            printDIERef(DataReader, *ChildAbbrevIdx, DIEData, DIEChildrenStack))
      return E;
  }

  return Error::success();
}

Error DIEPrinter::printDIERef(DIEDataRef Ref) {
  StringRef DIEData = Ref.getData();
  BinaryStreamReader DataReader(DIEData, support::endianness::little);

  Expected<uint64_t> MaybeAbbrevIdx = readAbbrevIdx(DistinctReader);
  if (!MaybeAbbrevIdx)
    return MaybeAbbrevIdx.takeError();
  auto AbbrevIdx = *MaybeAbbrevIdx;

  // The tag of a fresh block must be meaningful, otherwise we wouldn't have
  // made a new block.
  assert(AbbrevIdx != getEndOfDIESiblingsMarker() &&
         AbbrevIdx != getDIEInAnotherBlockMarker());

  OS.indent(Indent);
  OS << "CAS block: " << Ref.getID().toString() << "\n";

  Expected<SmallVector<DIEDataRef>> MaybeChildren =
      loadAllRefs<DIEDataRef>(Ref);
  if (!MaybeChildren)
    return MaybeChildren.takeError();
  ArrayRef<DIEDataRef> Children = *MaybeChildren;

  return printDIERef(DataReader, AbbrevIdx, DIEData, Children);
}

Error MCCASPrinter::printSimpleNested(MCObjectProxy Ref, CASDWARFObject &Obj,
                                      DWARFContext *DWARFCtx) {
  IndentGuard Guard(Indent);

  if (auto AbbrevOffsetsRef = DebugAbbrevOffsetsRef::Cast(Ref);
      Options.DebugAbbrevOffsets && AbbrevOffsetsRef)
    if (auto E = printAbbrevOffsets(OS, *AbbrevOffsetsRef))
      return E;

  if (auto TopRef = DIETopLevelRef::Cast(Ref); TopRef && Options.DIERefs)
    return DIEPrinter::printDIETopLevelRef(*TopRef, OS, Indent);

  return Ref.forEachReference(
      [&](ObjectRef CASObj) { return printMCObject(CASObj, Obj, DWARFCtx); });
}
