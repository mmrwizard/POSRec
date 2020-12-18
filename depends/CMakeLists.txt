add_subdirectory(QtAwesome)
add_subdirectory(QtColorWidget)
add_subdirectory(ranger)
add_subdirectory(aes)
add_subdirectory(gco)
add_subdirectory(splinter)
set_target_properties(splinter
                      qtawesome
                      qtcolorwidget
                      ranger
                      aes
                      gco
                      PROPERTIES FOLDER "depends")

if(${H2O_BUILD_DOG})
    add_subdirectory(dog)
endif(${H2O_BUILD_DOG})
