rd /s /q build
conan build . -of=build -b=missing -pr=mingw_gcc13 -vv
cd build
cpack