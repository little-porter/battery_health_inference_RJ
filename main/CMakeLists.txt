# idf_component_register(SRC_DIRS  "." "./Src"
#                        INCLUDE_DIRS "." "./Inc")

# set(PROJECT_SRC_DIRS "\".\" \"./Src\"")
# set(PROJECT_INC_DIRS "\".\" \"./Inc\"")

# message("PROJECT_SRC_DIRS: ${PROJECT_SRC_DIRS}")
# message("PROJECT_INC_DIRS: ${PROJECT_INC_DIRS}")

idf_component_register(SRC_DIRS  ${PROJECT_SRC_DIRS}
                       INCLUDE_DIRS ${PROJECT_INC_DIRS})


littlefs_create_partition_image(lfs ./littlefs_image FLASH_IN_PROJECT)
