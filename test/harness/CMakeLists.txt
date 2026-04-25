add_library(fake_hc STATIC fake_hc.c fake_hc_exec.c)
target_link_libraries(fake_hc PUBLIC ohci_core)
target_include_directories(fake_hc PUBLIC .)

add_executable(test_smoke test_smoke.c)
target_link_libraries(test_smoke PRIVATE ohci_core)
add_test(NAME smoke COMMAND test_smoke)

add_executable(test_layout test_layout.c)
target_link_libraries(test_layout PRIVATE ohci_core)
add_test(NAME layout COMMAND test_layout)

add_executable(test_fake_hc test_fake_hc.c)
target_link_libraries(test_fake_hc PRIVATE fake_hc)
add_test(NAME fake_hc COMMAND test_fake_hc)

add_executable(test_edtd_layout test_edtd_layout.c)
target_link_libraries(test_edtd_layout PRIVATE ohci_core)
add_test(NAME edtd_layout COMMAND test_edtd_layout)

add_executable(test_dma test_dma.c)
target_link_libraries(test_dma PRIVATE ohci_core)
add_test(NAME dma COMMAND test_dma)

add_executable(test_hc_init test_hc_init.c)
target_link_libraries(test_hc_init PRIVATE fake_hc)
add_test(NAME hc_init COMMAND test_hc_init)

add_executable(test_pool test_pool.c)
target_link_libraries(test_pool PRIVATE ohci_core)
add_test(NAME pool COMMAND test_pool)

add_executable(test_control_build test_control_build.c)
target_link_libraries(test_control_build PRIVATE ohci_core)
add_test(NAME control_build COMMAND test_control_build)

add_executable(test_submit test_submit.c)
target_link_libraries(test_submit PRIVATE fake_hc)
add_test(NAME submit COMMAND test_submit)

add_executable(test_fake_exec test_fake_exec.c)
target_link_libraries(test_fake_exec PRIVATE fake_hc)
add_test(NAME fake_exec COMMAND test_fake_exec)
