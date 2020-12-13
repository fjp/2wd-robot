"""
    pangocffi.ffi_instance_builder
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    generates an FFI for pangocffi
"""

from pathlib import Path
from cffi import FFI
from typing import Optional


class FFIInstanceBuilder:

    def __init__(self, source: Optional[str] = None):
        self.source = source

    def generate(self) -> FFI:
        # Read the C definitions
        c_definitions_glib_file = open(
            str(Path(__file__).parent / 'c_definitions_glib.txt'),
            'r'
        )
        c_definitions_pango_file = open(
            str(Path(__file__).parent / 'c_definitions_pango.txt'),
            'r'
        )
        c_definitions_glib = c_definitions_glib_file.read()
        c_definitions_pango = c_definitions_pango_file.read()

        ffi = FFI()
        ffi.cdef(c_definitions_glib)
        ffi.cdef(c_definitions_pango)
        if self.source is not None:
            ffi.set_source(self.source, None)

        return ffi
