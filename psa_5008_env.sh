# Dedicated P87 configuration. Lateral mode is an explicit vehicle experiment.
export SKIP_FW_QUERY=1
export PSA_5008_ROLLING_START=1
export PSA_5008_LATERAL_TEST=1
if [ "$PSA_5008_LATERAL_TEST" = "1" ]; then
  unset FAKESEND
else
  export FAKESEND=1
fi
