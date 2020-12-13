try:
    __SLYCOT_SETUP__
except NameError:
    __SLYCOT_SETUP__ = False


if __SLYCOT_SETUP__:
    import sys as _sys
    _sys.stderr.write('Running from numpy source directory.\n')
    del _sys
else:

    # import slycot.examples

    # Analysis routines (15/40 wrapped)
    from .analysis import ab01nd, ab05md, ab05nd, ab07nd, ab08nd, ab08nz
    from .analysis import ab09ad, ab09ax, ab09bd, ab09md, ab09nd
    from .analysis import ab13bd, ab13dd, ab13ed, ab13fd


    # Data analysis routines (0/7 wrapped)

    # Filtering routines (0/6 wrapped)

    # Identification routines (0/5 wrapped)

    # Mathematical routines (7/81 wrapped)
    from .math import mc01td, mb03rd, mb03vd, mb03vy, mb03wd, mb05md, mb05nd

    # Synthesis routines (15/50 wrapped)

    from .synthesis import sb01bd,sb02md,sb02mt,sb02od,sb03md,sb03od
    from .synthesis import sb04md,sb04qd,sb10ad,sb10dd,sb10hd,sg03ad
    from .synthesis import sg02ad, sg03bd, sb10fd

    # Transformation routines (9/40 wrapped)
    from .transform import tb01id, tb03ad, tb04ad
    from .transform import tb05ad
    from .transform import tc04ad, tc01od
    from .transform import tf01md, tf01rd
    from .transform import td04ad, tb01pd

    # Version information
    from .version import version as __version__

    from numpy.testing import Tester
    test = Tester().test
    bench = Tester().bench
