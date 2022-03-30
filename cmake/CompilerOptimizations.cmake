function(set_project_optimizations project_name)
    set(MSVC_OPTIMIZATIONS
        /LTCG # link time optimization
    )
    set(CLANG_OPTIMIZATIONS
        -flto # link time optimization
    )
    set(GCC_OPTIMIZATIONS
        -flto # link time optimization
    )

    if(OPTIMIZE_FOR_NATIVE)
        set(CLANG_OPTIMIZATIONS -march=native ${CLANG_OPTIMIZATIONS})   
        set(GCC_OPTIMIZATIONS -march=native ${GCC_OPTIMIZATIONS})    
    endif()

    if(MSVC)
        set(PROJECT_OPTIMIZATIONS ${MSVC_OPTIMIZATIONS})
    elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
        set(PROJECT_OPTIMIZATIONS ${CLANG_OPTIMIZATIONS})
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(PROJECT_OPTIMIZATIONS ${GCC_OPTIMIZATIONS})
    else()
        message(
            AUTHOR_WARNING
                "No extra optimizations set for '${CMAKE_CXX_COMPILER_ID}' compiler."
        )
    endif()

    target_compile_options(${project_name} PUBLIC ${PROJECT_OPTIMIZATIONS})

endfunction()
