DEF_HELPER_1(debug, void, env)
DEF_HELPER_2(raise_exception, void, env, int)
DEF_HELPER_3(bcd_add,         int,  env, int, int)
DEF_HELPER_4(check_flags,     void, env, int, int, int)