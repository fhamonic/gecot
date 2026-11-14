rd /s /q build
conan build . -of=build -b=missing -pr=mingw_gcc15_c++26 -vv
cd build
cpack