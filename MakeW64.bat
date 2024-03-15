rd /s /q build
conan build . -of=build -b=missing -pr=mingw_gcc12 -vv
cd build
cpack