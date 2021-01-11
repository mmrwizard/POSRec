file(GLOB SOURCES "*.h" "*.cpp")
if(MSVC)
    list(APPEND SOURCES "getopt.c")
endif(MSVC)

add_library(pmc STATIC ${SOURCES})
