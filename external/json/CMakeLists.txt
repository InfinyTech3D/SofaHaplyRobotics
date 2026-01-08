include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

message(STATUS "## ${PROJECT_NAME}: nlohmann_json not found. Will fetch and include v3.11.3")
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)
