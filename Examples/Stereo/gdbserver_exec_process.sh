#!/usr/bin/env bash
docker exec orb /bin/bash -c "gdbserver :22 /tmp/example/cmake-build-debug-remote-host/example"
