/*
 * Created Date: Sunday, July 5th 2020, 12:11:31 am
 * Author: Han
 * Version Information ♥ H2O
 * Copyright (c) 2020 Vrlab
 */

#pragma once

#define H2O_VERSION_MAJOR 1
#define H2O_VERSION_MINOR 4
#define H2O_VERSION_REVISION 2

// Preprocessor to string conversion
#define H2O_TO_STRING_HELPER(x) #x
#define H2O_TO_STRING(x) H2O_TO_STRING_HELPER(x)

// H2O version as a string; for example "0.9.0".
#define H2O_VERSION_STRING H2O_TO_STRING(H2O_VERSION_MAJOR) "." \
                             H2O_TO_STRING(H2O_VERSION_MINOR) "." \
                             H2O_TO_STRING(H2O_VERSION_REVISION)
