//===- MC/MCCASDebugV1.cpp ------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/CAS/MCCASDebugV1.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/DebugInfo/DWARF/DWARFDataExtractor.h"

using namespace llvm;
using namespace llvm::mccasformats;
using namespace llvm::mccasformats::v1;

Expected<uint64_t>
mccasformats::v1::getFormSize(dwarf::Form Form, dwarf::FormParams FP,
                              StringRef CUData, uint64_t CUOffset,
                              bool IsLittleEndian, uint8_t AddressSize) {
  uint64_t FormSize = 0;
  bool Indirect = false;
  Error Err = Error::success();
  do {
    Indirect = false;
    switch (Form) {
    case dwarf::DW_FORM_addr:
    case dwarf::DW_FORM_ref_addr: {
      FormSize +=
          (Form == dwarf::DW_FORM_addr) ? FP.AddrSize : FP.getRefAddrByteSize();
      break;
    }
    case dwarf::DW_FORM_exprloc:
    case dwarf::DW_FORM_block: {
      DWARFDataExtractor DWARFExtractor(CUData, IsLittleEndian, AddressSize);
      uint64_t PrevOffset = CUOffset;
      CUOffset += DWARFExtractor.getULEB128(&CUOffset, &Err);
      FormSize += CUOffset - PrevOffset;
      break;
    }
    case dwarf::DW_FORM_block1: {
      DWARFDataExtractor DWARFExtractor(CUData, IsLittleEndian, AddressSize);
      uint64_t PrevOffset = CUOffset;
      CUOffset += DWARFExtractor.getU8(&CUOffset, &Err);
      FormSize += CUOffset - PrevOffset;
      break;
    }
    case dwarf::DW_FORM_block2: {
      DWARFDataExtractor DWARFExtractor(CUData, IsLittleEndian, AddressSize);
      uint64_t PrevOffset = CUOffset;
      CUOffset += DWARFExtractor.getU16(&CUOffset, &Err);
      FormSize += CUOffset - PrevOffset;
      break;
    }
    case dwarf::DW_FORM_block4: {
      DWARFDataExtractor DWARFExtractor(CUData, IsLittleEndian, AddressSize);
      uint64_t PrevOffset = CUOffset;
      CUOffset += DWARFExtractor.getU32(&CUOffset, &Err);
      FormSize += CUOffset - PrevOffset;
      break;
    }
    case dwarf::DW_FORM_implicit_const:
    case dwarf::DW_FORM_flag_present: {
      FormSize += 0;
      break;
    }
    case dwarf::Form::DW_FORM_compressed_ref4_1:
    case dwarf::DW_FORM_data1:
    case dwarf::DW_FORM_ref1:
    case dwarf::DW_FORM_flag:
    case dwarf::DW_FORM_strx1:
    case dwarf::DW_FORM_addrx1: {
      FormSize += 1;
      break;
    }
    case dwarf::Form::DW_FORM_compressed_ref4_2:
    case dwarf::DW_FORM_data2:
    case dwarf::DW_FORM_ref2:
    case dwarf::DW_FORM_strx2:
    case dwarf::DW_FORM_addrx2: {
      FormSize += 2;
      break;
    }
    case dwarf::Form::DW_FORM_compressed_ref4_3:
    case dwarf::DW_FORM_strx3: {
      FormSize += 3;
      break;
    }
    case dwarf::DW_FORM_data4:
    case dwarf::DW_FORM_ref4:
    case dwarf::DW_FORM_ref_sup4:
    case dwarf::DW_FORM_strx4:
    case dwarf::DW_FORM_addrx4: {
      FormSize += 4;
      break;
    }
    case dwarf::DW_FORM_ref_sig8:
    case dwarf::DW_FORM_data8:
    case dwarf::DW_FORM_ref8:
    case dwarf::DW_FORM_ref_sup8: {
      FormSize += 8;
      break;
    }
    case dwarf::DW_FORM_data16: {
      FormSize += 16;
      break;
    }
    case dwarf::DW_FORM_sdata: {
      DWARFDataExtractor DWARFExtractor(CUData, IsLittleEndian, AddressSize);
      uint64_t PrevOffset = CUOffset;
      DWARFExtractor.getSLEB128(&CUOffset, &Err);
      FormSize += CUOffset - PrevOffset;
      break;
    }
    case dwarf::DW_FORM_udata:
    case dwarf::DW_FORM_ref_udata:
    case dwarf::DW_FORM_rnglistx:
    case dwarf::DW_FORM_loclistx:
    case dwarf::DW_FORM_GNU_addr_index:
    case dwarf::DW_FORM_GNU_str_index:
    case dwarf::DW_FORM_addrx:
    case dwarf::DW_FORM_strx: {
      DWARFDataExtractor DWARFExtractor(CUData, IsLittleEndian, AddressSize);
      uint64_t PrevOffset = CUOffset;
      DWARFExtractor.getULEB128(&CUOffset, &Err);
      FormSize += CUOffset - PrevOffset;
      break;
    }
    case dwarf::DW_FORM_LLVM_addrx_offset: {
      DWARFDataExtractor DWARFExtractor(CUData, IsLittleEndian, AddressSize);
      uint64_t PrevOffset = CUOffset;
      DWARFExtractor.getULEB128(&CUOffset, &Err);
      FormSize += CUOffset - PrevOffset + 4;
      break;
    }
    case dwarf::DW_FORM_string: {
      DWARFDataExtractor DWARFExtractor(CUData, IsLittleEndian, AddressSize);
      auto CurrOffset = CUOffset;
      DWARFExtractor.getCStr(&CUOffset, &Err);
      FormSize += CUOffset - CurrOffset;
      break;
    }
    case dwarf::DW_FORM_indirect: {
      DWARFDataExtractor DWARFExtractor(CUData, IsLittleEndian, AddressSize);
      uint64_t PrevOffset = CUOffset;
      Form =
          static_cast<dwarf::Form>(DWARFExtractor.getULEB128(&CUOffset, &Err));
      Indirect = true;
      FormSize += CUOffset - PrevOffset;
      break;
    }
    case dwarf::DW_FORM_strp:
    case dwarf::DW_FORM_sec_offset:
    case dwarf::DW_FORM_GNU_ref_alt:
    case dwarf::DW_FORM_GNU_strp_alt:
    case dwarf::DW_FORM_line_strp:
    case dwarf::DW_FORM_strp_sup: {
      FormSize += FP.getDwarfOffsetByteSize();
      break;
    }
    case dwarf::DW_FORM_addrx3:
    case dwarf::DW_FORM_lo_user: {
      llvm_unreachable("usupported form");
      break;
    }
    }
  } while (Indirect && !Err);

  if (Err)
    return std::move(Err);

  return FormSize;
}

template <> struct llvm::DenseMapInfo<llvm::dwarf::Form> {
  static llvm::dwarf::Form getEmptyKey() {
    return static_cast<llvm::dwarf::Form>(
        DenseMapInfo<uint16_t>::getEmptyKey());
  }

  static llvm::dwarf::Form getTombstoneKey() {
    return static_cast<llvm::dwarf::Form>(
        DenseMapInfo<uint16_t>::getTombstoneKey());
  }

  static unsigned getHashValue(const llvm::dwarf::Form &OVal) {
    return DenseMapInfo<uint16_t>::getHashValue(OVal);
  }

  static bool isEqual(const llvm::dwarf::Form &LHS,
                      const llvm::dwarf::Form &RHS) {
    return LHS == RHS;
  }
};

bool mccasformats::v1::doesntDedup(dwarf::Form Form, dwarf::Attribute Attr) {
  // This is a list of attributes known to have a high impact in the
  // deduplication of CAS objects.
  // Some of these are dependent on the Attribute in which they are used.
  static const DenseMap<dwarf::Form, SmallVector<dwarf::Attribute>>
      FormsToPartition{
          {dwarf::Form::DW_FORM_ref_addr, {}},
          {dwarf::Form::DW_FORM_strp, {}},
          {dwarf::Form::DW_FORM_ref4, {}},
          {dwarf::Form::DW_FORM_compressed_ref4_1, {}},
          {dwarf::Form::DW_FORM_compressed_ref4_2, {}},
          {dwarf::Form::DW_FORM_compressed_ref4_3, {}},
          {dwarf::Form::DW_FORM_data1,
           {dwarf::Attribute::DW_AT_call_file,
            dwarf::Attribute::DW_AT_decl_file}},
          {dwarf::Form::DW_FORM_data2,
           {dwarf::Attribute::DW_AT_call_file,
            dwarf::Attribute::DW_AT_decl_file}},
          {dwarf::Form::DW_FORM_data4,
           {dwarf::Attribute::DW_AT_call_file,
            dwarf::Attribute::DW_AT_decl_file}},
          {dwarf::Form::DW_FORM_data8,
           {dwarf::Attribute::DW_AT_decl_file,
            dwarf::Attribute::DW_AT_call_file}},
          {dwarf::Form::DW_FORM_addrx, {}},
      };

  auto it = FormsToPartition.find(Form);
  if (it == FormsToPartition.end())
    return false;
  if (it->second.empty())
    return true;
  return llvm::is_contained(it->second, Attr);
}

ArrayRef<char> mccasformats::v1::convertRef4(ArrayRef<char> Ref4Data) {
  assert(Ref4Data.size() == 4);
  auto UnsignedData = makeArrayRef(
      reinterpret_cast<const uint8_t *>(Ref4Data.data()), Ref4Data.size());
  auto Reader = BinaryStreamReader(UnsignedData, support::endianness::little);

  uint32_t Ref4;
  if (auto Err = Reader.readInteger(Ref4))
    handleAllErrors(std::move(Err)); // this should never fail
  auto Ref4APInt = APInt(32, Ref4);
  int BytesNeeded = (Ref4APInt.getActiveBits() + 7) / 8;
  assert(BytesNeeded <= 4 && BytesNeeded > 0);
  return Ref4Data.take_front(BytesNeeded);
}

static dwarf::Form convertForm(dwarf::Form Form, ArrayRef<char> FormData) {
  if (Form != dwarf::Form::DW_FORM_ref4)
    return Form;
  auto ConvertedRef = convertRef4(FormData);
  switch (ConvertedRef.size()) {
    case 1:
      return dwarf::Form::DW_FORM_compressed_ref4_1;
    case 2:
      return dwarf::Form::DW_FORM_compressed_ref4_2;
    case 3:
      return dwarf::Form::DW_FORM_compressed_ref4_3;
    case 4:
      return dwarf::Form::DW_FORM_ref4;
  }
  llvm_unreachable("invalid form size");
}

void AbbrevEntryWriter::writeAbbrevEntry(DWARFDie DIE,
                                         ArrayRef<char> DebugInfoData) {
  // [uleb(Tag), has_children]
  // [uleb(Attr), uleb(Form)]*
  writeULEB128(DIE.getTag());
  writeByte(DIE.hasChildren());
  for (const DWARFAttribute &AttrValue : DIE.attributes()) {
    writeULEB128(AttrValue.Attr);

    ArrayRef<char> FormData =
        DebugInfoData.slice(AttrValue.Offset, AttrValue.ByteSize);
    writeULEB128(convertForm(AttrValue.Value.getForm(), FormData));
  }
}

Expected<dwarf::Tag> AbbrevEntryReader::readTag() {
  uint64_t TagAsInt;
  if (auto E = DataStream.readULEB128(TagAsInt))
    return std::move(E);
  return static_cast<dwarf::Tag>(TagAsInt);
}

Expected<bool> AbbrevEntryReader::readHasChildren() {
  char HasChildren;
  if (auto E = DataStream.readInteger(HasChildren))
    return std::move(E);
  return HasChildren;
}

Expected<dwarf::Attribute> AbbrevEntryReader::readAttr() {
  if (DataStream.bytesRemaining() == 0)
    return static_cast<dwarf::Attribute>(getEndOfAttributesMarker());
  uint64_t AttrAsInt;
  if (auto E = DataStream.readULEB128(AttrAsInt))
    return std::move(E);
  return static_cast<dwarf::Attribute>(AttrAsInt);
}

Expected<dwarf::Form> AbbrevEntryReader::readForm() {
  uint64_t FormAsInt;
  if (auto E = DataStream.readULEB128(FormAsInt))
    return std::move(E);
  return static_cast<dwarf::Form>(FormAsInt);
}

uint64_t
mccasformats::v1::reconstructAbbrevSection(raw_ostream &OS,
                                           ArrayRef<StringRef> AbbrevEntries) {
  uint64_t WrittenSize = 0;
  for (auto [EntryIdx, EntryData] : enumerate(AbbrevEntries)) {
    // Dwarf 5: Section 7.5.3:
    // Each declaration begins with an unsigned LEB128 number representing the
    // abbreviation code itself. [...] The abbreviation code 0 is reserved for
    // null debugging information entries.
    const uint64_t AbbrevCode = EntryIdx + 1;
    WrittenSize += encodeULEB128(AbbrevCode, OS);

    uint64_t PrevSize = WrittenSize;
    BinaryStreamReader Reader(EntryData, support::endianness::little);
    // [uleb(Tag), has_children]
    uint64_t TagAsInt;
    uint8_t HasChildren;
    if (auto Err = Reader.readULEB128(TagAsInt))
      handleAllErrors(std::move(Err)); // this should never fail
    if (auto Err = Reader.readInteger(HasChildren))
      handleAllErrors(std::move(Err)); // this should never fail
    WrittenSize += encodeULEB128(TagAsInt, OS);
    OS << HasChildren;
    WrittenSize += 1;
    assert(HasChildren == 0 || HasChildren == 1);

    // [uleb(Attr), uleb(Form)]*
    while (!Reader.empty()) {
      uint64_t AttrAsInt;
      uint64_t FormAsInt;
      if (auto Err = Reader.readULEB128(AttrAsInt))
        handleAllErrors(std::move(Err)); // this should never fail
      if (auto Err = Reader.readULEB128(FormAsInt))
        handleAllErrors(std::move(Err)); // this should never fail

      WrittenSize += encodeULEB128(AttrAsInt, OS);

      auto Form = static_cast<dwarf::Form>(FormAsInt);
      if (Form == dwarf::Form::DW_FORM_compressed_ref4_1 ||
          Form == dwarf::Form::DW_FORM_compressed_ref4_2 ||
          Form == dwarf::Form::DW_FORM_compressed_ref4_3)
        Form = dwarf::Form::DW_FORM_ref4;

      WrittenSize += encodeULEB128(Form, OS);
    }
    assert(WrittenSize - PrevSize == EntryData.size());

    // Dwarf 5: Section 7.5.3:
    // The series of attribute specifications ends with an entry containing 0
    // for the name and 0 for the form.
    OS.write_zeros(2);
    WrittenSize += 2;
  }

  // Dwarf 5: Section 7.5.3:
  // The abbreviations for a given compilation unit end with an entry
  // consisting of a 0 byte for the abbreviation code.
  OS.write_zeros(1);
  WrittenSize += 1;
  return WrittenSize;
}
