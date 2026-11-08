---
name: pickup
description: Start a session on one Heron workstream (lidar, remote, odometry, motor, telemetry, builder, navigation) - read its resume file and Inbox, check its branches, and say where it stands. Use when the user says "resume", "continue" or "pick up" a workstream.
argument-hint: <area>
---

# Pick up a workstream

The workstream is `$ARGUMENTS`. Named `pickup`, not `resume`: Claude Code
already has a `/resume` command, for reopening a conversation.

The rules are `CLAUDE.md`, "Workstreams". This is the routine they imply
at the start of a session.

1. **Which workstream.** If no area was given, read `docs/RESUME.md`. If
   exactly one workstream is active, take it; if more than one, ask which.
   Never guess.
2. **Read in, in this order:**
   - its section in `CLAUDE.md`, and the documents it says to read before
     working (read the ones today's work touches; skim the others);
   - `docs/RESUME-<area>.md`, all of it;
   - its **Inbox**, the last section: facts other workstreams found for
     it.
   Read no other workstream's resume file or docs, except shared ones
   (`CLAUDE.md`, `docs/cpp-build.md`, `docs/rig-*.md`).
3. **Check git.** Run `git status`, `git branch --no-merged develop` and
   `git log --oneline develop..` on any branch of this workstream's. Note
   which are pushed and which wait on others (stacked branches merge in
   order).
4. **Report, briefly, then wait for the user's go-ahead:**
   - where it stands, in two or three lines;
   - Inbox entries, each with what folding it in would change. Propose
     folding them in first, on a `chore/` branch, where they bear on the
     next step. Check each against the owner's docs: an entry can be
     wrong, and an [unverified] one stays unverified until measured;
   - unmerged or unpushed branches, and their merge order;
   - the next step, from "Picking up", and anything it needs from the
     user (the Pi on, hands at the bench, a decision).

Do not start the work until the user agrees to the next step, and do not
re-open decisions the resume file records as made.
