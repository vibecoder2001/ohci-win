add_library(ohci_core STATIC
  src/ohci_core.c
)
target_include_directories(ohci_core PUBLIC include)
