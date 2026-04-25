add_library(ohci_core STATIC
  src/ohci_core.c
  src/ohci_dma.c
  src/ohci_hc.c
  src/ohci_pool.c
  src/ohci_control.c
  src/ohci_drain.c
)
target_include_directories(ohci_core PUBLIC include)
