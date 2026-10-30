add_lldb_unittest(ObjectFileWasmTests
  TestObjectFileWasm.cpp

  LINK_LIBS
    lldbPluginObjectFileWasm
    lldbCore
    lldbUtilityHelpers
    LLVMTestingSupport
  )
