add_lldb_library(lldbPluginOperatingSystemSwift PLUGIN
  OperatingSystemSwift.cpp

  LINK_LIBS
    lldbCore
    lldbInterpreter
    lldbSymbol
    lldbTarget
    lldbValueObject
    lldbPluginProcessUtility
  )
