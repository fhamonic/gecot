from conan import ConanFile
from conan.tools.cmake import CMake


class CompressorRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"
    build_policy = "missing"

    def requirements(self):
        self.requires("nlohmann_json/3.11.3", override=True)
        self.requires("json-schema-validator/2.3.0")
        self.requires("fast-cpp-csv-parser/cci.20240102")
        self.requires("onetbb/2021.9.0")
        self.requires("boost/1.85.0")
        self.requires("parallel-hashmap/1.37")
        self.requires("eigen/3.4.0")
        self.requires("spdlog/1.14.0")
        # self.requires("gdal/[>=3.8.3]")

        self.requires("melon/1.0.0-alpha.1")
        self.requires("mippp/0.1")

    def build_requirements(self):
        self.tool_requires("cmake/3.27.1")
        self.requires("gtest/1.14.0")

        if self.settings.os == "Windows":
            self.requires("mingw-builds/12.2.0")

    def generate(self):
        print("conanfile.py: IDE include dirs:")
        for lib, dep in self.dependencies.items():
            if not lib.headers:
                continue
            for inc in dep.cpp_info.includedirs:
                print("\t" + inc)

    def cmake_variables(self):
        vars = {}
        if self.settings.os != "Windows":
            vars["ENABLE_TESTING"] = "ON"
        if self.settings.os == "Windows":
            for lib, dep in self.dependencies.items():
                if lib.ref.name == "onetbb":
                    vars["CONAN_TBB_INCLUDE_DIR"] = ";".join(
                        dep.cpp_info.components["libtbb"].includedirs
                    )
                    vars["CONAN_TBB_LIB_DIR"] = ";".join(
                        dep.cpp_info.components["libtbb"].libdirs
                    )
                if lib.ref.name == "mingw-builds":
                    vars["CONAN_MINGW_LIB_DIR"] = ";".join(dep.cpp_info.libdirs)
        print("variables ", vars)
        return vars

    def build(self):
        cmake = CMake(self)
        cmake.configure(variables=self.cmake_variables())
        cmake.build()
