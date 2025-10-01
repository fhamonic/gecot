from conan import ConanFile
from conan.tools.cmake import CMake


class CompressorRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"
    build_policy = "missing"

    def requirements(self):
        self.requires("nlohmann_json/3.11.3", override=True)
        self.requires("json-schema-validator/2.3.0")
        self.requires("vincentlaucsb-csv-parser/2.3.0")
        self.requires("onetbb/2021.12.0")
        self.requires(
            "boost/1.88.0",
            options={
                "filesystem_use_std_fs": True,
                "without_atomic": True,
                "without_charconv": True,
                "without_chrono": True,
                "without_cobalt": True,
                "without_container": True,
                "without_context": True,
                "without_contract": True,
                "without_coroutine": True,
                "without_date_time": True,
                "without_exception": True,
                "without_fiber": True,
                "without_filesystem": True,
                "without_graph": True,
                "without_graph_parallel": True,
                "without_iostreams": True,
                "without_json": True,
                "without_locale": True,
                "without_log": True,
                "without_math": True,
                "without_mpi": True,
                "without_nowide": True,
                "without_process": True,
                "without_program_options": False,
                "without_python": True,
                "without_random": True,
                "without_regex": True,
                "without_serialization": True,
                "without_stacktrace": True,
                "without_system": True,
                "without_test": True,
                "without_thread": True,
                "without_timer": True,
                "without_type_erasure": True,
                "without_url": True,
                "without_wave": True,
            },
        )

        self.requires("parallel-hashmap/1.37")
        self.requires("eigen/3.4.0")
        self.requires("spdlog/1.14.0")
        # self.requires("gdal/[>=3.8.3]")

        self.requires("melon/1.0.0-alpha.1")
        self.requires("mippp/0.2")

    def build_requirements(self):
        self.requires("gtest/1.14.0")

    def generate(self):
        print(
            'conanfile.py: Include directories:\n\t"{}"'.format(
                '",\n\t"'.join(
                    [
                        dir
                        for lib, dep in self.dependencies.items()
                        if lib.headers
                        for dir in dep.cpp_info.includedirs
                    ]
                )
            )
        )

    def cmake_variables(self):
        vars = {}
        if self.settings.os != "Windows":
            vars["ENABLE_TESTING"] = "ON"
        # if self.settings.os == "Windows":
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
