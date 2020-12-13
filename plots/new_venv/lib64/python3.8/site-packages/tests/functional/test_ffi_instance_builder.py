from pangocffi.ffi_instance_builder import FFIInstanceBuilder
from cffi import FFI


def test_ffi_instance_builder():
    ffi_builder = FFIInstanceBuilder('test')
    ffi = ffi_builder.generate()
    assert isinstance(ffi, FFI)
