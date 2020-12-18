file(GLOB AES_SRC "*.h" "*.cpp" "*.hpp" "*.c")
add_library(aes STATIC ${AES_SRC})
