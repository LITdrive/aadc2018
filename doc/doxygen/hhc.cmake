# hhc.exe always returns nonzero exit codes
# indicect hhc.exe execution is to avoid build error in visual studio
message("HTML HELP Compiler generates doc....in ${CMAKE_CURRENT_BINARY_DIR}")
EXECUTE_PROCESS(
  COMMAND ${HTML_HELP_COMPILER} html/index.hhp
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)