add_executable(test_smoke test_smoke.c)
target_link_libraries(test_smoke PRIVATE ohci_core)
add_test(NAME smoke COMMAND test_smoke)

add_executable(test_layout test_layout.c)
target_link_libraries(test_layout PRIVATE ohci_core)
add_test(NAME layout COMMAND test_layout)
