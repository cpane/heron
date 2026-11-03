#!/usr/bin/env bash

# Check, one question at a time, that every wheel turns the right way.
#
# For anyone who has just built or rewired a Heron: no code knowledge
# needed. Run it from this repository on your computer, with the robot
# switched on and lifted so no wheel touches anything:
#
#   builder/wheel_check.sh <user>@<pi>
#
# Part 1, the motors: it turns one motor at a time for 2 seconds and asks
# which wheel turned, and which way. If any answer is wrong, it works out
# the fix and prints it two ways: which wires to move on the motor board,
# or the two lines to change in the robot's profile instead.
#
# Part 2, the remote (optional, once part 1 passes): it runs heron_teleop
# and asks you to push each stick in turn. It reads what the robot saw from
# heron_teleop's status line, checks the wheel commands against what that
# stick should do, and asks whether the wheels did it.
#
# The guide for builders, with what each answer means, is
# docs/builder-guide.md.
#
# How it works: everything runs on the Pi over SSH. Part 1 runs motor_link
# drive with one wheel at 600 of 1000; part 2 runs heron_teleop in the
# background and reads its once-a-second status lines. Both need the
# Arduino's port, so if the heron-teleop service (teleop_autostart.sh) is
# running, the script offers to stop it, and to start it again at the end.
# Stopping and starting it uses sudo on the Pi, which asks for the Pi
# user's password.
#
# Motor order is the protocol's: 1 front-left, 2 front-right, 3 back-right,
# 4 back-left. Positive drives the robot forward. The profile's terminal
# and sign lines say which motor-board terminal (M1..M4) and which polarity
# each one uses (docs/rig-profiles.md).

set -Eeuo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <user@host>"
    echo "Example: $0 <user>@<pi>"
    exit 1
fi

TARGET="$1"

if [[ "${TARGET}" != *@* ]]; then
    echo "ERROR: Target must be specified as user@host."
    echo "Example: $0 <user>@<pi>"
    exit 1
fi

TARGET_USER="${TARGET%@*}"
UNIT="heron-teleop@${TARGET_USER}.service"

DRIVE_VALUE=600
DRIVE_SECONDS=2
WHEELS=("front-left" "front-right" "back-right" "back-left")

SERVICE_STOPPED=0
TELEOP_PID=""
RUN_LOG="$(mktemp "${TMPDIR:-/tmp}/heron-wheel-check.XXXXXX")"
TELEOP_LOG="$(mktemp "${TMPDIR:-/tmp}/heron-wheel-check.XXXXXX")"

banner() {
    echo
    echo "========================================"
    echo " $1"
    echo "========================================"
    echo
}

# Returns 0 for yes, 1 for no; asks again on anything else.
ask_yes() {
    local answer
    while true; do
        read -r -p "$1 [y/n] " answer
        case "${answer}" in
            y|Y|yes|Yes) return 0 ;;
            n|N|no|No) return 1 ;;
        esac
        echo "Please answer y or n."
    done
}

stop_teleop() {
    if [[ -z "${TELEOP_PID}" ]]; then
        return
    fi
    # SIGINT, as Ctrl-C would: heron_teleop then sends Stop to the Arduino
    # and closes the port cleanly.
    ssh -n "${TARGET}" "pkill -INT -x heron_teleop" 2>/dev/null || true
    local waited=0
    while kill -0 "${TELEOP_PID}" 2>/dev/null && [[ ${waited} -lt 10 ]]; do
        sleep 0.5
        waited=$((waited + 1))
    done
    kill "${TELEOP_PID}" 2>/dev/null || true
    wait "${TELEOP_PID}" 2>/dev/null || true
    TELEOP_PID=""
}

start_service_again() {
    if [[ ${SERVICE_STOPPED} -ne 1 ]]; then
        return
    fi
    echo
    if ask_yes "Start the teleop service again, so the remote drives the robot?"; then
        # -t so sudo can ask for the password.
        ssh -t "${TARGET}" "sudo systemctl start '${UNIT}'"
        SERVICE_STOPPED=0
        echo "Started."
    fi
}

cleanup() {
    stop_teleop
    rm -f "${RUN_LOG}" "${TELEOP_LOG}"
    if [[ ${SERVICE_STOPPED} -eq 1 ]]; then
        echo
        echo "NOTE: the teleop service is still stopped. Start it again with"
        echo "  ssh -t ${TARGET} sudo systemctl start ${UNIT}"
        echo "or restart the Pi."
    fi
}
trap cleanup EXIT

# ---------------------------------------------------------------------------
# Before anything turns

banner "Heron wheel check"
echo "Target:      ${TARGET}"
echo

echo "Checking SSH connectivity..."
if ! ssh -n -o BatchMode=yes -o ConnectTimeout=5 "${TARGET}" true 2>/dev/null; then
    echo "ERROR: cannot log in to ${TARGET} without a password."
    echo "  Is the Pi on, and is the address right? If it asks for a password,"
    echo "  set up a key once with:   ssh-copy-id ${TARGET}"
    exit 1
fi
echo "OK."

echo "Checking the Pi has Heron's programs and a rig..."
if ! ssh -n "${TARGET}" "test -x heron/bin/motor_link && test -x heron/bin/heron_teleop"; then
    echo "ERROR: Heron's programs are not on the Pi. Build and copy them first:"
    echo "  scripts/build_cpp.sh && scripts/deploy_cpp.sh ${TARGET}"
    exit 1
fi
RIG="$(ssh -n "${TARGET}" "cat heron/rig 2>/dev/null || true")"
if [[ -z "${RIG}" ]]; then
    echo "ERROR: the Pi has not been told which robot it is in. Set it first:"
    echo "  scripts/set_rig.sh ${TARGET} <rig>"
    exit 1
fi
if ! PROFILE_TEXT="$(ssh -n "${TARGET}" "cat 'heron/profiles/${RIG}.conf'" 2>/dev/null)"; then
    echo "ERROR: the Pi has no profile for rig '${RIG}'. Copy it there with:"
    echo "  scripts/deploy_cpp.sh ${TARGET}"
    exit 1
fi
# The [motor] lines; '^terminal *=' does not match terminal_and_sign.
read -r -a TERMINALS <<< "$(printf '%s\n' "${PROFILE_TEXT}" | sed -n 's/^terminal *= *//p' | head -n 1)"
read -r -a SIGNS <<< "$(printf '%s\n' "${PROFILE_TEXT}" | sed -n 's/^sign *= *//p' | head -n 1)"
if [[ ${#TERMINALS[@]} -ne 4 || ${#SIGNS[@]} -ne 4 ]]; then
    echo "ERROR: profiles/${RIG}.conf on the Pi has no terminal and sign lines of four each."
    exit 1
fi
echo "OK: rig '${RIG}', terminals ${TERMINALS[*]}, signs ${SIGNS[*]}."

if ssh -n "${TARGET}" "systemctl is-active --quiet '${UNIT}'"; then
    echo
    echo "The teleop service is running: it lets the remote drive the robot, and"
    echo "while it runs nothing else can talk to the motors."
    if ! ask_yes "Stop it for this check? (sudo on the Pi will ask for its password)"; then
        echo "Nothing checked."
        exit 1
    fi
    ssh -t "${TARGET}" "sudo systemctl stop '${UNIT}'"
    SERVICE_STOPPED=1
fi

echo
echo "Before the wheels turn:"
echo "  - Lift the robot so no wheel touches anything: a box under its frame."
echo "  - Motor battery connected and switched on."
echo "  - On the L293D motor shield, the PWR jumper must be off."
echo "  - Keep fingers and cables clear of the wheels."
echo
echo "\"Forward\" below means the way that would roll the robot forward: the"
echo "top of the wheel moves toward the robot's front."
echo
if ! ask_yes "Ready?"; then
    echo "Nothing checked."
    start_service_again
    exit 1
fi

# ---------------------------------------------------------------------------
# Part 1: one motor at a time

TURNED=()     # per motor: 0..3 the wheel that turned, -1 none, -2 several
DIRECTION=()  # per motor: f or b

for i in 0 1 2 3; do
    values=(0 0 0 0)
    values[i]=${DRIVE_VALUE}
    while true; do
        banner "Part 1: motor $((i + 1)) of 4"
        echo "This should turn the ${WHEELS[i]} wheel forward, for ${DRIVE_SECONDS} seconds."
        read -r -p "Watch the wheels, and press Enter to start. " _
        echo "Turning..."
        if ! ssh -n "${TARGET}" "heron/bin/motor_link drive ${values[*]} --seconds ${DRIVE_SECONDS}" \
                > "${RUN_LOG}" 2>&1; then
            echo "ERROR: the motors could not be driven. What the Pi said:"
            tail -n 5 "${RUN_LOG}"
            exit 1
        fi
        echo
        echo "Which wheel turned?"
        echo "  1  front-left     3  back-right     5  none"
        echo "  2  front-right    4  back-left      6  more than one"
        echo "  r  run it again"
        choice=""
        while true; do
            read -r -p "> " choice
            case "${choice}" in
                [1-6]|r|R) break ;;
            esac
            echo "Please answer 1 to 6, or r."
        done
        case "${choice}" in
            r|R) continue ;;
            5) TURNED[i]=-1; DIRECTION[i]=f; break ;;
            6) TURNED[i]=-2; DIRECTION[i]=f; break ;;
        esac
        TURNED[i]=$((choice - 1))
        while true; do
            read -r -p "Which way did it turn? f forward, b backward: " choice
            case "${choice}" in
                f|F) DIRECTION[i]=f; break ;;
                b|B) DIRECTION[i]=b; break ;;
            esac
        done
        break
    done
done

banner "Part 1: results"

describe() {
    case "$1" in
        -1) echo "nothing turned" ;;
        -2) echo "more than one wheel turned" ;;
        *)
            if [[ "$2" == "f" ]]; then
                echo "${WHEELS[$1]} turned forward"
            else
                echo "${WHEELS[$1]} turned backward"
            fi
            ;;
    esac
}

ALL_RIGHT=1
for i in 0 1 2 3; do
    if [[ ${TURNED[i]} -eq ${i} && "${DIRECTION[i]}" == "f" ]]; then
        mark="OK   "
    else
        mark="WRONG"
        ALL_RIGHT=0
    fi
    printf '  %s  motor %d (%s): %s\n' "${mark}" $((i + 1)) "${WHEELS[i]}" "$(describe "${TURNED[i]}" "${DIRECTION[i]}")"
done
echo

if [[ ${ALL_RIGHT} -ne 1 ]]; then
    for i in 0 1 2 3; do
        if [[ ${TURNED[i]} -lt 0 ]]; then
            echo "A motor turned no wheel, or more than one. That is not a mix-up the"
            echo "script can work out: check that the motor battery is on, that each"
            echo "motor's two wires are tight in their terminal, and that no wheel is"
            echo "jammed. Then run this again."
            start_service_again
            exit 1
        fi
    done
    for i in 0 1 2 3; do
        for j in 0 1 2 3; do
            if [[ ${i} -lt ${j} && ${TURNED[i]} -eq ${TURNED[j]} ]]; then
                echo "Motors $((i + 1)) and $((j + 1)) both turned the ${WHEELS[${TURNED[i]}]} wheel."
                echo "One of those answers must be wrong: run this again."
                start_service_again
                exit 1
            fi
        done
    done

    # Motor i's command went out on terminal T[i] with sign S[i], and turned
    # wheel w. So wheel w's motor is on terminal T[i]; it belongs on T[w],
    # the terminal the profile gives wheel w. Its two wires need swapping if
    # it turned backward, or if T[w] has the opposite sign from T[i].
    echo "Fix it ONE of two ways."
    echo
    echo "Either A, move wires on the motor board (motor battery off first):"
    NEW_TERMINALS=(0 0 0 0)
    NEW_SIGNS=(+ + + +)
    for i in 0 1 2 3; do
        w=${TURNED[i]}
        NEW_TERMINALS[w]=${TERMINALS[i]}
        if [[ "${DIRECTION[i]}" == "f" ]]; then
            NEW_SIGNS[w]=${SIGNS[i]}
        elif [[ "${SIGNS[i]}" == "+" ]]; then
            NEW_SIGNS[w]=-
        else
            NEW_SIGNS[w]=+
        fi
        swap=0
        if [[ "${DIRECTION[i]}" == "b" ]]; then
            swap=1
        fi
        if [[ "${SIGNS[i]}" != "${SIGNS[w]}" ]]; then
            swap=$((1 - swap))
        fi
        if [[ ${w} -eq ${i} && ${swap} -eq 0 ]]; then
            continue
        fi
        if [[ ${TERMINALS[i]} == "${TERMINALS[w]}" ]]; then
            echo "  - The ${WHEELS[w]} motor, on terminal M${TERMINALS[i]}: swap its two wires."
        elif [[ ${swap} -eq 1 ]]; then
            echo "  - The ${WHEELS[w]} motor is on terminal M${TERMINALS[i]}: move it to M${TERMINALS[w]},"
            echo "    and swap its two wires."
        else
            echo "  - The ${WHEELS[w]} motor is on terminal M${TERMINALS[i]}: move it to M${TERMINALS[w]},"
            echo "    keeping its two wires in the same order."
        fi
    done
    echo
    echo "Or B, leave the wires and change two lines in profiles/${RIG}.conf on"
    echo "this computer, under [motor]:"
    echo
    echo "  terminal = ${NEW_TERMINALS[*]}"
    echo "  sign = ${NEW_SIGNS[*]}"
    echo
    echo "then copy it to the Pi with   scripts/deploy_cpp.sh ${TARGET}"
    echo
    echo "Then run this check again."
    start_service_again
    exit 1
fi

echo "All four motors turn the right wheel, the right way."
echo

# ---------------------------------------------------------------------------
# Part 2: the remote

banner "Part 2: the remote"
echo "This runs the program that drives with the remote, and asks you to push"
echo "each stick in turn. The robot must still be lifted."
echo
if ! ask_yes "Check the remote too?"; then
    start_service_again
    exit 0
fi

# The newest status line printed after line $1 of the log, or nothing.
# -tt gives the remote program a terminal, so its lines end in \r\n.
latest_status() {
    tail -n "+$(( $1 + 1 ))" "${TELEOP_LOG}" | tr -d '\r' \
        | grep -E '^(DRIVING|ARMED|DISARMED) ' | tail -n 1 || true
}

log_lines() {
    wc -l < "${TELEOP_LOG}" | tr -d ' '
}

teleop_alive() {
    if ! kill -0 "${TELEOP_PID}" 2>/dev/null; then
        echo
        echo "ERROR: heron_teleop stopped. What the Pi said:"
        tr -d '\r' < "${TELEOP_LOG}" | tail -n 8
        TELEOP_PID=""
        exit 1
    fi
}

# From a status line: "vx vy wz w1 w2 w3 w4", the numbers after those words.
numbers() {
    printf '%s\n' "$1" | awk '{
        for (i = 1; i < NF; i++) {
            if ($i == "vx") vx = $(i + 1)
            if ($i == "vy") vy = $(i + 1)
            if ($i == "wz") wz = $(i + 1)
            if ($i == "wheels") { w1 = $(i + 1); w2 = $(i + 2); w3 = $(i + 3); w4 = $(i + 4) }
        }
        print vx, vy, wz, w1, w2, w3, w4
    }'
}

echo
echo "Starting heron_teleop on the Pi (it resets the Arduino)..."
# -tt: a terminal on the Pi, so the program ends if this connection drops.
ssh -tt "${TARGET}" "heron/bin/heron_teleop" < /dev/null > "${TELEOP_LOG}" 2>&1 &
TELEOP_PID=$!

echo
echo "Arm the remote:"
echo "  1. Left stick (the throttle) all the way down."
echo "  2. Switch SwA (top left) down."
echo "  3. Throttle up to about half way."
echo
echo "Waiting for the robot to say DRIVING..."
start=$(log_lines)
shown=""
waited=0
while true; do
    teleop_alive
    line="$(latest_status "${start}")"
    mode="${line%% *}"
    if [[ "${mode}" == "DRIVING" ]]; then
        echo "DRIVING."
        break
    fi
    if [[ -n "${mode}" && "${mode}" != "${shown}" ]]; then
        case "${mode}" in
            DISARMED) echo "  Disarmed: is the transmitter on, and SwA down?" ;;
            ARMED)    echo "  Armed, waiting for the throttle: bring it all the way down, then up." ;;
        esac
        shown="${mode}"
    fi
    if [[ ${waited} -ge 240 ]]; then
        echo "ERROR: no DRIVING after 2 minutes. Last status:"
        echo "  ${line:-none}"
        exit 1
    fi
    sleep 0.5
    waited=$((waited + 1))
done

# name | what to do | which number (1 vx, 2 vy, 3 wz) | sign | wheel signs | what the wheels should do
MOVES=(
    "Right stick up|Push the right stick all the way up and hold it.|1|+|++++|all four turn forward"
    "Right stick down|Push the right stick all the way down and hold it.|1|-|----|all four turn backward"
    "Right stick left|Push the right stick all the way left and hold it.|2|+|-+-+|front-left and back-right turn backward, front-right and back-left forward (the robot would slide left)"
    "Left stick left|Keep the throttle where it is, push the left stick all the way left and hold it.|3|+|-++-|front-left and back-left turn backward, front-right and back-right forward (the robot would turn left on the spot)"
)
COMPONENT_NAMES=("" "forward/backward" "sideways" "turning")

FAILED=0
for move in "${MOVES[@]}"; do
    IFS='|' read -r name instruction component sign pattern expect <<< "${move}"
    banner "Part 2: ${name}"
    echo "${instruction}"
    echo "Waiting for the robot to see it..."
    start=$(log_lines)
    shown=""
    waited=0
    while true; do
        teleop_alive
        line="$(latest_status "${start}")"
        if [[ "${line%% *}" == "DRIVING" ]]; then
            # The biggest of vx, vy, wz, if it is at least 0.3: which way
            # the stick says to go, and its sign.
            read -r seen_component seen_sign <<< "$(numbers "${line}" | awk '{
                best = 0; c = 0
                for (k = 1; k <= 3; k++) { a = $k < 0 ? -$k : $k; if (a > best) { best = a; c = k } }
                if (best >= 0.3) print c, ($c < 0 ? "-" : "+"); else print 0, "+"
            }')"
            if [[ ${seen_component} -ne 0 ]]; then
                break
            fi
        elif [[ -n "${line}" && "${line%% *}" != "${shown}" ]]; then
            echo "  The robot is not DRIVING (${line%% *}): arm it again as before."
            shown="${line%% *}"
        fi
        if [[ ${waited} -ge 120 ]]; then
            echo "ERROR: no stick movement seen in 1 minute."
            exit 1
        fi
        sleep 0.5
        waited=$((waited + 1))
    done

    wheel_signs="$(numbers "${line}" | awk '{
        s = ""
        for (k = 4; k <= 7; k++) s = s ($k < 0 ? "-" : "+")
        print s
    }')"
    echo "Seen: ${line}"
    echo "Let go of the stick."
    echo
    if [[ ${seen_component} -ne ${component} || "${seen_sign}" != "${sign}" ]]; then
        echo "WRONG: the robot read this stick as ${COMPONENT_NAMES[seen_component]} (${seen_sign}),"
        echo "not ${COMPONENT_NAMES[component]} (${sign}). The remote's channel or stick direction"
        echo "is set up differently from Heron's: see \"The remote reads wrong\" in"
        echo "docs/builder-guide.md."
        FAILED=1
        continue
    fi
    if [[ "${wheel_signs}" != "${pattern}" ]]; then
        echo "WRONG: the wheel commands were ${wheel_signs}, not ${pattern}."
        FAILED=1
        continue
    fi
    if ask_yes "Did ${expect}?"; then
        echo "OK."
    else
        echo "WRONG: the commands were right, so a wheel is wired differently from"
        echo "part 1. Run this again from the start."
        FAILED=1
    fi
done

banner "Part 2: the arm switch"
echo "Push the throttle up a little and the right stick up, then flip SwA up."
echo "Waiting for the robot to say DISARMED..."
start=$(log_lines)
waited=0
while true; do
    teleop_alive
    line="$(latest_status "${start}")"
    if [[ "${line%% *}" == "DISARMED" ]]; then
        break
    fi
    if [[ ${waited} -ge 120 ]]; then
        echo "ERROR: no DISARMED in 1 minute."
        exit 1
    fi
    sleep 0.5
    waited=$((waited + 1))
done
if ask_yes "Did every wheel stop at once?"; then
    echo "OK."
else
    echo "WRONG: the wheels should stop the moment SwA goes up."
    FAILED=1
fi

stop_teleop

banner "Result"
if [[ ${FAILED} -eq 0 ]]; then
    echo "Every wheel and every stick checks out. The robot is ready to drive."
else
    echo "Some checks were wrong: see the WRONG lines above."
fi
start_service_again
exit ${FAILED}
