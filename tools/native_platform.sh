#!/usr/bin/env bash
# Shared native-build platform selection. This script does not cross-compile.
case "$(uname -m)" in
  x86_64) LEGBOT_ARCH=x86_64; LEGBOT_DEFAULT_JOBS=2 ;;
  aarch64|arm64) LEGBOT_ARCH=aarch64; LEGBOT_DEFAULT_JOBS=1 ;;
  *) echo '仅支持原生 x86_64 或 ARM64 构建，不支持 ARM32。' >&2; return 1 2>/dev/null || exit 1 ;;
esac
