"""Parking control with fused localization and bounded local planning."""

# The matrices here are small; BLAS thread pools compete with the EKF and
# sensor callbacks on the vehicle CPU. Set this before NumPy is imported.
import os as _os

_os.environ['OPENBLAS_NUM_THREADS'] = '1'
_os.environ['OMP_NUM_THREADS'] = '1'
del _os
