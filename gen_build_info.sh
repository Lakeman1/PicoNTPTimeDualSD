#!/bin/bash

COMMIT=$(git rev-parse --short HEAD)
BRANCH=$(git rev-parse --abbrev-ref HEAD)
DATE=$(date "+%Y-%m-%d %H:%M")

cat > BuildInfo.h <<EOF
#ifndef BUILDINFO_H
#define BUILDINFO_H

#define FW_GIT_COMMIT "$COMMIT"
#define FW_GIT_BRANCH "$BRANCH"
#define FW_BUILD_DATE "$DATE"

#endif
EOF