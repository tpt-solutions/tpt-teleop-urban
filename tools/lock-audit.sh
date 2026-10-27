#!/usr/bin/env bash
# Zero-lock hot-path audit.
#
# Fails if a hot-path crate source uses a lock, a blocking channel, or an async
# runtime. Hot-path crates are the ones that run on the thread-per-core control
# path (spec §4.5 and §13 of todo.md).
#
# Run from the repository root: tools/lock-audit.sh
set -euo pipefail

HOT_PATH_CRATES=(core safety compliance edge)
FORBIDDEN='std::sync::(Mutex|RwLock|Condvar|mpsc)|parking_lot|\bMutex\b|\bRwLock\b|tokio|crossbeam|async fn|\.await\b'

status=0
checked=0
for crate in "${HOT_PATH_CRATES[@]}"; do
    dir="crates/tpt-t-urban-${crate}/src"
    [ -d "$dir" ] || continue
    checked=$((checked + 1))
    if grep -rnE "$FORBIDDEN" "$dir"; then
        echo "lock-audit: forbidden primitive found in $dir" >&2
        status=1
    fi
done

if [ "$status" -eq 0 ]; then
    echo "lock-audit: ${checked} hot-path crate(s) clean"
fi
exit "$status"
