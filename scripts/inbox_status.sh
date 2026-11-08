#!/usr/bin/env bash
#
# List what is waiting in every workstream's Inbox.
#
# Each workstream's resume file (docs/RESUME-<area>.md) ends with an Inbox:
# facts another workstream found for it, one "### " entry each, waiting for
# the owner to check and fold in when it resumes (CLAUDE.md, "Workstreams").
# An entry is easy to miss: nobody sees it until that workstream is picked
# up. This script lists them all, wherever they are:
#
#   - on develop;
#   - on any branch not yet merged into develop, local or on origin, so an
#     entry is seen before its pull request is merged;
#   - in the working tree, written but not yet committed.
#
# A branch counts only for what it changed since it left develop (its merge
# base): an entry it added is waiting on it; an entry it deleted has been
# folded in on it, and goes from develop when it is merged, so it is listed
# as such, not as waiting. A branch that merely carries an entry from an
# older develop says nothing about it.
#
# Usage:
#   scripts/inbox_status.sh          # the list, for a person
#   scripts/inbox_status.sh --hook   # JSON for Claude Code's SessionStart
#                                    # hook (.claude/settings.json); prints
#                                    # nothing when every Inbox is empty
#
# Reads git only; changes nothing. Unlike the other host scripts it takes no
# user@host: it never touches the Pi.

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

MODE="list"
if [[ $# -gt 1 ]] || { [[ $# -eq 1 ]] && [[ "$1" != "--hook" ]]; }; then
    echo "Usage: $0 [--hook]"
    exit 1
fi
if [[ $# -eq 1 ]]; then
    MODE="hook"
fi

# A session-start hook must never get in the way of a session: in hook mode
# any failure prints nothing and exits 0.
if [[ "${MODE}" == "hook" ]]; then
    trap 'exit 0' ERR
    exec 2>/dev/null
fi

cd "${REPO_ROOT}"

if ! git rev-parse --verify --quiet develop >/dev/null; then
    if [[ "${MODE}" == "hook" ]]; then
        exit 0
    fi
    echo "ERROR: No develop branch in this clone."
    exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT
ENTRIES="${WORK}/entries"
: > "${ENTRIES}"

# The "### " headings between "## Inbox" and the next "## " heading, as
# "<area><TAB><heading><TAB><where><TAB><kind>" lines; kind is "has", or
# "gone" for an entry a branch deletes.
inbox_entries() {
    local area="$1"
    local where="$2"
    local kind="${3:-has}"
    awk -v area="${area}" -v where="${where}" -v kind="${kind}" '
        /^## /      { inbox = ($0 == "## Inbox"); next }
        inbox && /^### / {
            sub(/^### +/, "")
            print area "\t" $0 "\t" where "\t" kind
        }'
}

# develop first, then every branch it does not contain yet. A branch and its
# origin copy are the same place to a reader, so origin/ is dropped.
REFS="develop"
REFS="${REFS} $(git for-each-ref --format='%(refname:short)' \
    --no-merged=develop refs/heads refs/remotes/origin \
    | grep -v '^origin/HEAD$' || true)"

# A resume file's contents at a ref, or in the working tree ("WORKTREE");
# nothing if it does not exist there.
show_file() {
    if [[ "$1" == "WORKTREE" ]]; then
        cat "$2" 2>/dev/null || true
    else
        git show "$1:$2" 2>/dev/null || true
    fi
}

resume_files() {
    if [[ "$1" == "WORKTREE" ]]; then
        ls docs/RESUME-*.md 2>/dev/null || true
    else
        git ls-tree --name-only "$1" docs/ | grep '^docs/RESUME-.*\.md$' || true
    fi
}

area_of() {
    local area
    area="$(basename "$1" .md)"
    printf '%s' "${area#RESUME-}"
}

# develop: every entry on it is there to be folded in.
for file in $(resume_files develop); do
    show_file develop "${file}" | inbox_entries "$(area_of "${file}")" develop >> "${ENTRIES}"
done

# A branch, or the working tree: only what it changed since it left develop
# (its merge base). Entries it added are waiting there; entries it deleted
# are folded in there. Entries it merely still carries from an older
# develop say nothing, so a branch started before an entry was folded does
# not bring it back.
scan_changes() {
    local tip="$1"
    local base="$2"
    local where="$3"
    local file area
    for file in $( { resume_files "${base}"; resume_files "${tip}"; } | sort -u); do
        area="$(area_of "${file}")"
        show_file "${base}" "${file}" | inbox_entries "${area}" "${where}" \
            | sort > "${WORK}/base"
        show_file "${tip}" "${file}" | inbox_entries "${area}" "${where}" \
            | sort > "${WORK}/tip"
        comm -13 "${WORK}/base" "${WORK}/tip" >> "${ENTRIES}"
        comm -23 "${WORK}/base" "${WORK}/tip" \
            | awk -F '\t' -v OFS='\t' '{ $4 = "gone"; print }' >> "${ENTRIES}"
    done
}

for ref in ${REFS}; do
    if [[ "${ref}" == "develop" ]]; then
        continue
    fi
    scan_changes "${ref}" "$(git merge-base develop "${ref}")" "${ref#origin/}"
done

if [[ -n "$(git status --porcelain -- 'docs/RESUME-*.md')" ]]; then
    scan_changes WORKTREE HEAD "working tree"
fi

# One line per entry: "<area><TAB><heading><TAB><where it is>". An entry on
# develop is simply waiting; otherwise say which branches carry it, since it
# reaches develop only when one of them is merged.
SUMMARY="${WORK}/summary"
sort -u "${ENTRIES}" | awk -F '\t' '
    function add(list, item) {
        if (index(", " list ", ", ", " item ", ") > 0) return list
        return list == "" ? item : list ", " item
    }
    {
        key = $1 "\t" $2
        if (!(key in seen)) { seen[key] = 1; order[++n] = key }
        if ($4 == "gone")          gone[key] = add(gone[key], $3)
        else if ($3 == "develop")  ondev[key] = 1
        else                       places[key] = add(places[key], $3)
    }
    END {
        for (i = 1; i <= n; i++) {
            key = order[i]
            if (key in gone && key in ondev)
                print key "\tfolded\ton develop; folded in on " gone[key] \
                      ", gone when merged"
            else if (key in ondev)
                print key "\twaiting\ton develop"
            else if (places[key] == "working tree")
                print key "\twaiting\tin the working tree, not committed"
            else if (key in places)
                print key "\twaiting\tonly on " places[key]
        }
    }' | sort > "${SUMMARY}"

WAITING="${WORK}/waiting"
FOLDED="${WORK}/folded"
awk -F '\t' '$3 == "waiting" { print $1 "\t" $2 "\t" $4 }' "${SUMMARY}" > "${WAITING}"
awk -F '\t' '$3 == "folded"  { print $1 "\t" $2 "\t" $4 }' "${SUMMARY}" > "${FOLDED}"
COUNT="$(wc -l < "${WAITING}" | tr -d ' ')"

# "<area>:" then each entry and where it is.
print_entries() {
    awk -F '\t' '
        $1 != last { if (NR > 1) print ""; print "  " $1 ":"; last = $1 }
        { print "    - " $2; print "        (" $3 ")" }' "$1"
}

if [[ "${MODE}" == "list" ]]; then
    echo "========================================"
    echo " Heron Workstream Inboxes"
    echo "========================================"
    echo
    if [[ "${COUNT}" -eq 0 ]]; then
        echo "Nothing waiting, on develop or on any unmerged branch."
    else
        echo "Waiting:"
        print_entries "${WAITING}"
        echo
        echo "Each owner folds its entries in when it resumes: /pickup <area>."
    fi
    if [[ -s "${FOLDED}" ]]; then
        echo
        echo "Already folded in, waiting only for a merge:"
        print_entries "${FOLDED}"
    fi
    exit 0
fi

# Hook mode: quiet unless something is waiting.
if [[ "${COUNT}" -eq 0 ]]; then
    exit 0
fi

# JSON string escaping in plain bash, so the hook needs no jq or python.
json_escape() {
    local s="$1"
    s="${s//\\/\\\\}"
    s="${s//\"/\\\"}"
    s="${s//$'\t'/ }"
    s="${s//$'\n'/\\n}"
    printf '%s' "${s}"
}

# For the person: one line, the areas and their counts.
COUNTS="$(cut -f1 "${WAITING}" | uniq -c | awk '{ printf "%s%s %s", sep, $2, $1; sep = ", " }')"
MESSAGE="Workstream Inboxes waiting: ${COUNTS}. Details: scripts/inbox_status.sh"

# For Claude: every waiting entry and where it is, and what to do with them.
CONTEXT="Heron workstream Inbox entries are waiting (scripts/inbox_status.sh, run at session start):"
while IFS="$(printf '\t')" read -r area heading where; do
    CONTEXT="${CONTEXT}"$'\n'"- ${area}: ${heading} (${where})"
done < "${WAITING}"
CONTEXT="${CONTEXT}"$'\n'"Each entry is read and folded in by its own workstream when it resumes (/pickup <area>), never by another. Mention one when the work at hand touches it; an entry only on a branch reaches develop when that branch is merged."

printf '{"systemMessage":"%s","hookSpecificOutput":{"hookEventName":"SessionStart","additionalContext":"%s"}}\n' \
    "$(json_escape "${MESSAGE}")" "$(json_escape "${CONTEXT}")"
