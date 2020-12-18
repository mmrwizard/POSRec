include(CMakeHelper)
h2o_add_test(test_utf8 test_utf8.cpp h2o_base)
h2o_add_test(test_proj test_proj.cpp h2o_base PROJ4::PROJ4)
h2o_add_test(test_rpc test_rpc.cpp h2o_base h2o_sat)
