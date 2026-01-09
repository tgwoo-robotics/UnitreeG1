# Project Skills & Guidelines

This file contains auto-applied skills for Claude Code.

---

# Project Rules

## Git Commit Rules
- **NEVER include Co-Authored-By in commit messages**
- Keep commit messages concise and in Korean or English as appropriate

---

# Skill 1: Dispatching Parallel Agents

Use when facing 2+ independent tasks that can be worked on without shared state or sequential dependencies.

## Overview

When you have multiple unrelated failures (different test files, different subsystems, different bugs), investigating them sequentially wastes time. Each investigation is independent and can happen in parallel.

**Core principle:** Dispatch one agent per independent problem domain. Let them work concurrently.

## When to Use

**Use when:**
- 3+ test files failing with different root causes
- Multiple subsystems broken independently
- Each problem can be understood without context from others
- No shared state between investigations

**Don't use when:**
- Failures are related (fix one might fix others)
- Need to understand full system state
- Agents would interfere with each other

## The Pattern

### 1. Identify Independent Domains
Group failures by what's broken - each domain is independent.

### 2. Create Focused Agent Tasks
Each agent gets:
- **Specific scope:** One test file or subsystem
- **Clear goal:** Make these tests pass
- **Constraints:** Don't change other code
- **Expected output:** Summary of what you found and fixed

### 3. Dispatch in Parallel
```typescript
Task("Fix agent-tool-abort.test.ts failures")
Task("Fix batch-completion-behavior.test.ts failures")
Task("Fix tool-approval-race-conditions.test.ts failures")
// All three run concurrently
```

### 4. Review and Integrate
When agents return: Read each summary, verify fixes don't conflict, run full test suite.

---

# Skill 2: Codex CLI Integration

Use when the user asks to run Codex CLI (codex exec, codex resume) or references OpenAI Codex for code analysis, refactoring, or automated editing.

## Running a Task
1. Ask the user which model to run: `gpt-5` or `gpt-5-codex`
2. Ask the user which reasoning effort to use: `low`, `medium`, or `high`
3. Select sandbox mode: `--sandbox read-only` (default), `workspace-write`, or `danger-full-access`
4. Assemble and run the command

### Quick Reference
| Use case | Sandbox mode | Key flags |
| --- | --- | --- |
| Read-only review | `read-only` | `--sandbox read-only` |
| Apply local edits | `workspace-write` | `--sandbox workspace-write --full-auto` |
| Network/broad access | `danger-full-access` | `--sandbox danger-full-access --full-auto` |
| Resume session | Inherited | `echo "prompt" \| codex exec resume --last` |

## Following Up
- After every `codex` command, confirm next steps with user
- When resuming: `echo "new prompt" | codex exec resume --last`

---

# Skill 3: Codex-Claude Engineering Loop

Orchestrates a dual-AI engineering loop where Claude Code plans and implements, while Codex validates and reviews.

## Core Workflow
- **Claude Code**: Architecture, planning, and execution
- **Codex**: Validation and code review
- **Continuous Review**: Each AI reviews the other's work

## The Perfect Loop
```
Plan (Claude) -> Validate Plan (Codex) -> Feedback ->
Implement (Claude) -> Review Code (Codex) ->
Fix Issues (Claude) -> Re-validate (Codex) -> Repeat until perfect
```

## Phases

1. **Planning**: Claude creates detailed plan
2. **Plan Validation**: Codex reviews for logic errors, edge cases, security
3. **Feedback Loop**: Refine plan based on Codex feedback
4. **Execution**: Claude implements using Edit/Write/Read tools
5. **Cross-Review**: Codex reviews implementation
6. **Iterative Improvement**: Continue until quality standards met

---

# Skill 4: Prompt Coach (Session Log Analysis)

Analyze Claude Code session logs to improve prompt quality and efficiency.

## Capabilities
- Token usage & cost analysis
- Prompt quality scoring
- Tool usage patterns
- Session efficiency metrics
- Productivity time patterns
- File modification heatmap
- Error & recovery analysis
- Project switching analysis

## Quick Commands
- "Give me a general analysis of my Claude Code usage"
- "Analyze my prompt quality"
- "How much have I spent on Claude Code this month?"
- "What tools do I use most?"

## Prompt Quality Guidelines

### Good Prompts Include:
- File paths when referencing code
- Error messages when debugging
- Expected behavior or outcome
- Clear success criteria

### Context-Rich Brief Prompts (Score 8-10):
- "git commit" - Claude has git diff context
- "run tests" - project structure provides context
- "yes/no/1/2" - answering Claude's questions

### Vague Prompts to Avoid (Score 0-4):
- "fix the bug" - which bug? where?
- "optimize it" - optimize what?
- "make it better" - better how?

## Log Location
`~/.claude/projects/` - Each project has `.jsonl` session files

## Analysis Commands
```bash
# Find recent sessions
find ~/.claude/projects -name "*.jsonl" -mtime -30

# Check log size
du -sh ~/.claude/projects

# Count sessions
find ~/.claude/projects -name "*.jsonl" | wc -l
```

## Token Pricing (Nov 2025)
| Model | Input | Output | Cache Writes | Cache Reads |
|-------|-------|--------|--------------|-------------|
| Opus 4.1 | $15/1M | $75/1M | $18.75/1M | $1.50/1M |
| Sonnet 4.5 | $3/1M | $15/1M | $3.75/1M | $0.30/1M |
| Haiku 4.5 | $1/1M | $5/1M | $1.25/1M | $0.10/1M |
