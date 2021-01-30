idf_component_register(SRCS "timer.c" "ulp_adc_example_main.c" 
                            "spiff.c"
                            "bleinit.c"
                        INCLUDE_DIRS "."
                       INCLUDE_DIRS "include"
                      REQUIRES soc nvs_flash ulp driver bt)
#
# ULP support additions to component CMakeLists.txt.
#
# 1. The ULP app name must be unique (if multiple components use ULP).
set(ulp_app_name ulp_${COMPONENT_NAME})
#
# 2. Specify all assembly source files.
#    Files should be placed into a separate directory (in this case, ulp/),
#    which should not be added to COMPONENT_SRCS.
set(ulp_s_sources "ulp/main.S"
"ulp/i2c.S"
"ulp/stack.S"
"ulp/i2c-util.S"
"ulp/i2c_mpu.S"
)
#"ulp/i2c-util.S"
#"ulp/i2c_mpu.S"
# 3. List all the component source files which include automatically
#    generated ULP export file, ${ulp_app_name}.h:
set(ulp_exp_dep_srcs "ulp_adc_example_main.c")
#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
