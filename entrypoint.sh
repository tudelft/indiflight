#!/bin/bash

cd /indiflight

# BOARD/PROFILE: from -e BOARD=... -e PROFILE=..., otherwise from the selection
# made with the VS Code task "IntelliSense: select board and profile"
SELECTION_FILE=.vscode/build_selection.json
BOARD_SRC="-e BOARD"
PROFILE_SRC="-e PROFILE"
if [[ -z "$BOARD" || -z "$PROFILE" ]] && [[ -f "$SELECTION_FILE" ]]; then
    if [[ -z "$BOARD" ]]; then
        BOARD=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["board"])' "$SELECTION_FILE")
        BOARD_SRC="$SELECTION_FILE"
    fi
    if [[ -z "$PROFILE" ]]; then
        PROFILE=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["profile"])' "$SELECTION_FILE")
        PROFILE_SRC="$SELECTION_FILE"
    fi
fi
[[ -z "$BOARD" ]] && BOARD_SRC="NOT SET"
[[ -z "$PROFILE" ]] && PROFILE_SRC="NOT SET"
export BOARD PROFILE

print_config() {
    local bar="=================================================================="
    echo
    echo "$bar"
    echo "  $1"
    printf "  BOARD   = %-24s (%s)\n" "${BOARD:-<none>}" "$BOARD_SRC"
    printf "  PROFILE = %-24s (%s)\n" "${PROFILE:-<none>}" "$PROFILE_SRC"
    echo "$bar"
    echo
}

print_config "Building with:"

# On a terminal, replace the per-file "%% ..." lines with one updating
# "[n/N] file" line. Everything else (warnings, errors, linking) is printed as is.
progress_filter() {
    awk '
        /^%%TOTAL / { total = $2; next }
        /^%% / {
            n++; f = $NF; sub(/^\.\//, "", f)
            printf "\r\033[K[%d/%s] %s", n, (total ? total : "?"), f
            shown = 1; fflush(); next
        }
        { if (shown) { printf "\n"; shown = 0 } print; fflush() }
        END { if (shown) printf "\n" }
    '
}

if [[ -t 1 ]]; then
    make -j PROGRESS=y "$@" 2>&1 | progress_filter
    MAKE_STATUS=${PIPESTATUS[0]}
else
    make -j "$@"
    MAKE_STATUS=$?
fi

if [[ $MAKE_STATUS -eq 0 ]]; then
    print_config "SUCCESS, built with:"
else
    print_config "FAILED (exit $MAKE_STATUS), attempted with:"
    exit $MAKE_STATUS
fi

if [[ $UPLOAD_PARAMETERS == "y" ]]; then
    if [[ -z "$BOARD" || -z "$PROFILE" ]]; then
        echo "UPLOAD_PARAMETERS=y needs both BOARD and PROFILE." >&2
        exit 1
    fi

    echo "Wait for serial device to come up"
    sleep 5

    VENDOR_ID="0483"
    PRODUCT_ID="5740"
    DEVICE=
    for dev in /dev/ttyACM*; do
        [[ -e "$dev" ]] || continue
        info=$(udevadm info --query=property --name "$dev" 2>/dev/null)
        if grep -q "^ID_VENDOR_ID=$VENDOR_ID$" <<< "$info" && \
           grep -q "^ID_MODEL_ID=$PRODUCT_ID$" <<< "$info"; then
            DEVICE=$dev
            break
        fi
    done

    if [[ -n "$DEVICE" ]]; then
        echo "Uploading parameters to $DEVICE"
    else
        echo "No STM32 serial device found." >&2
        exit 1
    fi

    /usr/bin/python3 support/scripts/uploadProfile.py "$DEVICE" \
        "/indiflight/configs/boards/${BOARD}.txt" \
        "/indiflight/configs/profiles/${PROFILE}.txt"
fi
