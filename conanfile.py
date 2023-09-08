from conan import ConanFile
from conan.tools.cmake import CMake
from conan.tools.files import copy

class CompressorRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"
    build_policy = "missing"

    def requirements(self):
        self.requires("nlohmann_json/3.11.2")
        self.requires("json-schema-validator/2.2.0")
        self.requires("fast-cpp-csv-parser/cci.20211104")
        self.requires("onetbb/2021.10.0")
        self.requires("boost/1.83.0")
        self.requires("parallel-hashmap/1.37")
        self.requires("eigen/3.4.0")
        self.requires("fmt/10.1.1")
        self.requires("range-v3/0.12.0")
        self.requires("gtest/1.14.0")

    def build_requirements(self):
        self.tool_requires("cmake/3.27.1")
        # self.build_requires("gcc/12.2.0")
        
    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
