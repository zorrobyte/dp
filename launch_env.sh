#!/usr/bin/bash

#Force F150 fingerprint
export SKIP_FW_QUERY=1
export FINGERPRINT="F150"

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$NEOS_VERSION" ]; then
  export NEOS_VERSION="20"
fi

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="6.2"
fi

if [ -z "$PASSIVE" ]; then
  export PASSIVE="1"
fi

export STAGING_ROOT="/data/safe_staging"
