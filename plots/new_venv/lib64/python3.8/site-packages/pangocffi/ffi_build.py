"""
    pangocffi.ffi_build
    ~~~~~~~~~~~~~~~~~~~

    Build the cffi bindings for pangocffi

"""

import sys
from pathlib import Path
import importlib.util

sys.path.append(str(Path(__file__).parent))

# Create an empty _generated folder if needed
(Path(__file__).parent / '_generated').mkdir(exist_ok=True)

# Because we can't directly load the instance builder (it would run
# ``__init__.py`` for any module import) we have to do this dubious import.
spec = importlib.util.spec_from_file_location(
    'ffi_instance_builder',
    str(Path(__file__).parent / 'ffi_instance_builder.py')
)
ffi_instance_builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ffi_instance_builder)

# Generate the bindings
ffiBuilder = ffi_instance_builder.FFIInstanceBuilder(
    source='pangocffi._generated.ffi'
)
ffi = ffiBuilder.generate()

if __name__ == '__main__':
    ffi.compile()
