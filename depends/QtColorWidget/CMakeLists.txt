file(GLOB COLORWIDGET_SRC "*.cpp" "*.hpp" "*.h" "*.qrc" "*.ui" "^([^.]+)$")
list(APPEND COLORWIDGET_SRC 
    AbstractWidgetList
    BoundColorSelector
    ColorDelegate
    ColorDialog
    ColorListWidget
    ColorPreview
    ColorSelector
    ColorWheel
    GradientSlider
    HueSlider
)
add_library(qtcolorwidget STATIC ${COLORWIDGET_SRC})
target_include_directories(qtcolorwidget PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(qtcolorwidget PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets)
