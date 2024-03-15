from conan import ConanFile
from conan.tools.cmake import CMake

class CompressorRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"
    build_policy = "missing"

    def requirements(self):
        self.requires("json-schema-validator/[>=2.0.0]")
        self.requires("fast-cpp-csv-parser/cci.20211104")
        if self.settings.os != "Windows":
            self.requires("onetbb/2021.9.0")
        self.requires("boost/[1.82.0]")
        self.requires("parallel-hashmap/1.37")
        self.requires("eigen/[>=3.4.0]")
        self.requires("fmt/10.1.1")

        self.requires("melon/0.5")
        self.requires("mippp/0.1")

    def build_requirements(self):
        self.tool_requires("cmake/3.27.1")
        self.requires("gtest/1.14.0")
        # self.build_requires("gcc/12.2.0")

    def generate(self):
        print("conanfile.py: IDE include dirs:")
        for lib, dep in self.dependencies.items():
            if not lib.headers:
                continue
            for inc in dep.cpp_info.includedirs:
                print("\t" + inc)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
