set(LLVM_LINK_COMPONENTS
  Support
  CAS
  CASObjectFormats
  )

add_llvm_tool(llvm-cas-mc-dump
  llvm-cas-mc-dump.cpp
  MCCASPrinter.cpp
)
