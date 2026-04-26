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

add_executable(test_done test_done.c)
target_link_libraries(test_done PRIVATE fake_hc)
add_test(NAME done COMMAND test_done)

add_executable(test_control_e2e test_control_e2e.c)
target_link_libraries(test_control_e2e PRIVATE fake_hc)
add_test(NAME control_e2e COMMAND test_control_e2e)

add_executable(test_bulk_list test_bulk_list.c)
target_link_libraries(test_bulk_list PRIVATE fake_hc)
add_test(NAME bulk_list COMMAND test_bulk_list)

add_executable(test_bulk test_bulk.c)
target_link_libraries(test_bulk PRIVATE fake_hc)
add_test(NAME bulk COMMAND test_bulk)

add_executable(test_bulk_sg test_bulk_sg.c)
target_link_libraries(test_bulk_sg PRIVATE fake_hc)
add_test(NAME bulk_sg COMMAND test_bulk_sg)

add_executable(test_interrupt_skeleton test_interrupt_skeleton.c)
target_link_libraries(test_interrupt_skeleton PRIVATE fake_hc)
add_test(NAME interrupt_skeleton COMMAND test_interrupt_skeleton)

add_executable(test_interrupt test_interrupt.c)
target_link_libraries(test_interrupt PRIVATE fake_hc)
add_test(NAME interrupt COMMAND test_interrupt)

add_executable(test_teardown test_teardown.c)
target_link_libraries(test_teardown PRIVATE fake_hc)
add_test(NAME teardown COMMAND test_teardown)

add_executable(test_multi_endpoint_e2e test_multi_endpoint_e2e.c)
target_link_libraries(test_multi_endpoint_e2e PRIVATE fake_hc)
add_test(NAME multi_endpoint_e2e COMMAND test_multi_endpoint_e2e)

add_executable(test_itd_pool test_itd_pool.c)
target_link_libraries(test_itd_pool PRIVATE ohci_core)
add_test(NAME itd_pool COMMAND test_itd_pool)
