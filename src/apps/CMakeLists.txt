add_executable(sifted_fusiond fusiond.cpp)
target_link_libraries(sifted_fusiond PRIVATE sifted::core sifted_options)
