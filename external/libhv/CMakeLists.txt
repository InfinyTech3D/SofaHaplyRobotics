include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
    hv
    GIT_REPOSITORY https://github.com/ithewei/libhv
    GIT_TAG v1.3.2
    GIT_SHALLOW TRUE
    GIT_PROGRESS FALSE
    EXCLUDE_FROM_ALL
)

add_compile_options(${HAPLY_EXTERNAL_COMPILE_OPTIONS})

set(BUILD_SHARED OFF CACHE INTERNAL "")
set(BUILD_EXAMPLES OFF CACHE INTERNAL "")
set(BUILD_UNITTEST OFF CACHE INTERNAL "")
set(WITH_OPENSSL OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(hv)
