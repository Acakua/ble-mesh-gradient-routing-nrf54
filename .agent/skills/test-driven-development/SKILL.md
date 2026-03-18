---
name: test-driven-development
description: "Implement code using the Red-Green-Refactor cycle. No production code allowed without a failing test first."
risk: unknown
source: community
date_added: "2026-02-27"
---

# Test-Driven Development (TDD)

## The Iron Law
**NO PRODUCTION CODE WITHOUT A FAILING TEST FIRST.**
If you wrote code before the test, delete it and start over.

## Red-Green-Refactor Cycle
1. **RED**: Write a minimal failing test. Verify it fails for the expected reason.
2. **GREEN**: Write the simplest code to pass the test.
3. **REFACTOR**: Clean up the code while keeping tests green.

## Rules
- One behavior per test.
- Test real code, avoids mocks if possible.
- If it's hard to test, the design is likely wrong.
- Delete code if you skip the cycle. Sunk cost fallacy: unverified code is technical debt.

## Exit Criteria
- Every new function has a test.
- Watched each test fail first.
- Minimal code used.
- All tests pass.
