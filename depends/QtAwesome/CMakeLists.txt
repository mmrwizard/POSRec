file(GLOB QTAWESOME_SRC "*.h" "*.cpp" "*.ui" "*.qrc")
add_library(qtawesome STATIC ${QTAWESOME_SRC})
qt5_use_modules(qtawesome Core Gui Widgets)
