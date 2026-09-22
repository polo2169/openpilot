#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="18.4"
fi

export STAGING_ROOT="/data/safe_staging"

# Combined lateral/RVV trial; retain physical rearm and diagnostic-query exclusion.
export SKIP_FW_QUERY=1
export PSA_T9_LATERAL_TEST=1
export PSA_T9_RVV_TEST=1
export PSA_T9_SPLIT_AXES_TEST=1
export PSA_DASHCAM_ONLY=0
