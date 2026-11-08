---
name: pause
description: End a session on one Heron workstream - bring its resume file and the index up to date, hand facts for other workstreams to their Inboxes, commit, and report what is left. Use when the user says to stop, pause, wrap up or hand over a workstream.
argument-hint: <area> [workstream]
---

# Pause a workstream

The workstream is `$ARGUMENTS` (or, if none, the one this session worked
on; ask if that is unclear). "Pause" here ends a session. Only if the
user says the **workstream** is pausing (`workstream` in the arguments,
or in words) is it marked paused in the index and `CLAUDE.md`.

The rules are `CLAUDE.md`, "Workstreams". This is the routine they imply
at the end of a session: a fresh session, with none of this
conversation, must be able to pick up from the files alone.

1. **The resume file, `docs/RESUME-<area>.md`:**
   - "Last updated" set to today, with the state in one line;
   - new decisions under "Decided", dated, with who made them;
   - "Picking up" rewritten as the next steps, concrete enough to start
     without this conversation: commands, file paths, what to measure,
     and what needs the user (the Pi on, hands at the bench);
   - finished steps removed from "Picking up", and their results moved
     to where the file keeps results;
   - open questions under "Still open", each tagged [unverified] where
     nothing measured it.
   Keep the file's own structure and style; do not rewrite the rest.
2. **Facts for other workstreams.** Anything learned that belongs to
   another workstream goes in its Inbox, never into its docs: dated,
   "From <area>", where it was measured, how sure ([ours], [unverified]),
   and where the full write-up is. Shared facts (hardware, build, rigs)
   go in `CLAUDE.md`, `docs/cpp-build.md` or `docs/rig-*.md`, each change
   on its own `chore/` branch.
3. **The index, `docs/RESUME.md`:** this workstream's row, its state and
   where to pick up. If the workstream is pausing: "paused <date>" in the
   row, and in its `CLAUDE.md` heading.
4. **Commit** on the workstream's branch (never `develop` or `main`), in
   logical groups, as `CLAUDE.local.md` says. Do not push unless asked.
5. **Report:**
   - what the resume file now says to do next;
   - Inbox entries written, and to which workstreams;
   - branches not merged, which are pushed, and the order they merge in;
   - anything waiting on the user.
