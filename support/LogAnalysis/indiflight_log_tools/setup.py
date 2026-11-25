# Metadata lives in pyproject.toml. This file only describes the C decoder.
#
# The decoder (blackbox-tools) is NOT a Python extension module: it has no
# PyInit_* function and is loaded with ctypes. We still use setuptools'
# Extension machinery to compile it (it handles compilers, flags and
# cross-platform details), but:
#   - the output file gets a fixed name (blackbox_decode.so) instead of
#     blackbox_decode.cpython-3XX-<platform>.so, so the Python code can find it
#     regardless of the interpreter version, and
#   - the wheel is tagged py3-none-<platform> instead of cp3XX-cp3XX-<platform>,
#     so a single wheel per OS/arch installs on every Python 3 version.

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

try:
    from setuptools.command.bdist_wheel import bdist_wheel  # setuptools >= 70.1
except ImportError:
    from wheel.bdist_wheel import bdist_wheel

SRC = "indiflight_log_tools/blackbox-tools/src/"


class build_plain_shared_lib(build_ext):
    def get_ext_filename(self, fullname):
        # indiflight_log_tools.blackbox_decode -> indiflight_log_tools/blackbox_decode.so
        return fullname.replace(".", "/") + ".so"

    def get_export_symbols(self, ext):
        # don't require a PyInit_blackbox_decode symbol
        return ext.export_symbols


class bdist_wheel_py3_none(bdist_wheel):
    def finalize_options(self):
        super().finalize_options()
        self.root_is_pure = False  # still platform specific

    def get_tag(self):
        _, _, plat = super().get_tag()
        return "py3", "none", plat


blackbox_decode = Extension(
    "indiflight_log_tools.blackbox_decode",
    sources=[SRC + f for f in (
        "blackbox_decode.c",
        "blackbox_fielddefs.c",
        "decoders.c",
        "parser.c",
        "platform.c",
        "stats.c",
        "stream.c",
        "tools.c",
        "units.c",
    )],
    include_dirs=[SRC],
    extra_compile_args=["-O3", "-std=gnu99"],
    libraries=["m"],
    export_symbols=["main"],
)

setup(
    ext_modules=[blackbox_decode],
    cmdclass={"build_ext": build_plain_shared_lib, "bdist_wheel": bdist_wheel_py3_none},
)
