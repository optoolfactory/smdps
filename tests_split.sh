#!/bin/bash

function cleanup {
    kill $CAR_TESTS
    kill $OTHER_TESTS
}

$UNIT_TEST selfdrive/car &
CAR_TESTS=$!

($UNIT_TEST common && \
$UNIT_TEST opendbc/can && \
$UNIT_TEST selfdrive/boardd && \
$UNIT_TEST selfdrive/controls && \
$UNIT_TEST selfdrive/monitoring && \
$UNIT_TEST system/loggerd && \
$UNIT_TEST selfdrive/locationd && \
$UNIT_TEST selfdrive/test/longitudinal_maneuvers && \
$UNIT_TEST system/tests && \
$UNIT_TEST system/ubloxd && \
selfdrive/locationd/test/_test_locationd_lib.py && \
./system/ubloxd/tests/test_glonass_runner && \
$UNIT_TEST selfdrive/athena && \
$UNIT_TEST selfdrive/thermald && \
$UNIT_TEST system/hardware/tici && \
$UNIT_TEST tools/lib/tests && \
./selfdrive/ui/tests/create_test_translations.sh && \
QT_QPA_PLATFORM=offscreen ./selfdrive/ui/tests/test_translations && \
./selfdrive/ui/tests/test_translations.py && \
./common/tests/test_util && \
./common/tests/test_swaglog && \
./selfdrive/boardd/tests/test_boardd_usbprotocol && \
./system/loggerd/tests/test_logger &&\
./system/proclogd/tests/test_proclog && \
./tools/replay/tests/test_replay && \
./tools/cabana/tests/test_cabana && \
./system/camerad/test/ae_gray_test && \
./selfdrive/test/process_replay/test_fuzzy.py) &

OTHER_TESTS=$!

trap cleanup EXIT

wait $CAR_TESTS $OTHER_TESTS