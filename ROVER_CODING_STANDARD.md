# Rover Coding Standard for Python

This document defines the coding standard for Python in the rover software stack. It is inspired in part by the idea of using a small number of strict rules to improve clarity, boundedness, testability, and static analysability in critical software. It is written for this repository and for this rover. The rules below are intended to make the code easier to understand under review, easier to analyse with tools, and more predictable in operation.

These rules are written for Python in a ROS 2 rover stack. Python is well suited to orchestration, state management, configuration, diagnostics, integration, and tooling. It is not well suited to hard real-time inner loops where timing must be guaranteed within tight bounds. The rules below are intended to preserve Python where it is useful, while reducing the risks that come from dynamic behaviour, implicit defaults, and loose structure.

## 1. Rule: Restrict all code to simple, explicit control flow.

Do not use recursion in production code. Do not build control flow out of dynamic features such as `eval`, `exec`, runtime code generation, or string-built logic where a normal branch, dispatch table, or helper function would do. Avoid hidden state changes inside lambdas, comprehensions, decorators, and callbacks when a named function would make the behaviour clearer.

### Rationale:

Simple control flow is easier to inspect, easier to test, and easier to analyse statically. Python already permits a large amount of runtime indirection. If this freedom is used casually, the actual behaviour of the program becomes difficult to reconstruct from the source. That is undesirable in any large system, and especially undesirable in rover software where understanding failure paths matters.

The ban on recursion preserves an acyclic call structure and removes one source of unbounded stack growth and difficult-to-prove termination behaviour. In Python, recursion also carries a practical risk of shallow recursion limits and poor fit for operational code. The more general restriction on dynamic control flow serves the same purpose that bans on preprocessor tricks, function pointers, and similar constructs served in other languages: it prevents control flow from becoming obscure.

The purpose of this rule is not to reject abstraction. It is to reject abstraction that hides behaviour rather than clarifying it.

## 2. Rule: Every loop, wait, retry, poll, and queue must have an explicit bound.

It must be possible to identify, from the source, what limits the runtime behaviour of every loop and wait. This includes iteration counts, deadlines, retry counts, queue sizes, history sizes, cache sizes, and message accumulation. If a loop is intended to run continuously as part of a node executor, timer, or scheduler, that role must be explicit, and the work done on each pass must itself remain bounded.

### Rationale:

The aim is not to eliminate loops, but to eliminate hidden unboundedness. In rover software, the dangerous form of unboundedness is often not an obvious `while True` written openly in the code. It is a wait that quietly retries forever, a tracker that grows with incoming messages, a callback that appends into a list without limit, or a polling loop whose termination depends on conditions no one has stated clearly.

A bounded loop is easier to reason about and easier to test. It also gives clearer failure behaviour. If the bound is reached, the code should declare failure explicitly rather than drifting into delay, memory growth, or operational ambiguity. In Python this matters particularly because the runtime already introduces some non-determinism through scheduling, garbage collection, and interpreter overhead. Bounded application logic helps contain that uncertainty.

This rule also applies to histories and buffers. A list that grows indefinitely in a timer callback is simply another form of unbounded loop state.

## 3. Rule: Use Python only for bounded orchestration, coordination, and system logic. Do not place hard real-time control obligations on Python.

Python may be used for mission sequencing, state machines, bring-up, checks, supervisory logic, diagnostics, configuration, testing, simulation glue, and operator-facing tooling. Code that must meet strict timing guarantees, low-latency control deadlines, or deterministic actuator update periods should be implemented in a more suitable lower-level component.

### Rationale:

This rule exists because language choice is itself a safety decision. Python is highly productive and expressive, which makes it valuable for the parts of the system that coordinate behaviour. It is less suitable for code that must guarantee narrow timing bounds. CPython does not give precise control over garbage collection pauses, interpreter latency, thread scheduling, or callback dispatch. For that reason, Python should not be asked to perform work whose correctness depends on hard real-time guarantees.

This is not an argument against Python in the rover stack. It is an argument for assigning Python the right responsibilities. High-level mission logic, node orchestration, checks, and diagnostics benefit from Python's clarity and speed of development. Tight motor control loops, watchdog enforcement, and similarly timing-sensitive mechanisms should remain in lower-level code or firmware designed for that purpose.

The benefit of this rule is architectural clarity. Once the role of Python is defined, the coding rules that follow become more meaningful and easier to enforce.

## 4. Rule: No function should be longer than what can be understood as a single logical unit when viewed on one screen or one printed page.

As a rule of thumb, this means about 50 to 60 lines of code per function or method, with one line per statement and one line per declaration. A longer function is not forbidden absolutely, but it requires justification in the structure of the code rather than in the patience of the reader.

### Rationale:

Each function should represent one logical step in the software. When a function grows too large, it usually begins to mix responsibilities, hide state transitions, and obscure failure handling. A review then stops being a review of one unit of logic and becomes a search through a corridor of conditions, assignments, and side effects. That weakens both manual analysis and tool-based analysis.

A size guideline of about 50 to 60 lines is large enough to permit straightforward code and small enough to discourage monolithic orchestration functions. The point is not to fragment code into trivial wrappers. The point is to separate distinct responsibilities so that preconditions, state changes, and fault paths are visible.

This rule should apply to all production functions, not only to obviously safety-relevant ones. In practice, code that begins life as convenience code often becomes operational code over time. A consistent standard avoids endless arguments about which functions matter and prevents large helper functions from becoming hidden mission-critical machinery.

A file may contain many functions. The rule applies to each function, not to the file as a whole.

## 5. Rule: Validate every parameter, configuration value, external input, and interface result before relying on it.

All launch arguments, ROS parameters, YAML values, topic names, frame names, service responses, action results, message fields, and external data used in decisions must be checked for validity before use. Unknown or invalid values must fail closed. Silent fallback to permissive defaults is not allowed in safety-relevant or mission-relevant paths.

### Rationale:

Bad configuration is one of the most common and least interesting causes of serious faults. It is also one of the easiest to prevent. A rover stack has many configuration surfaces: launch files, parameter files, node parameters, action goals, services, topic contracts, frame names, and operator inputs. If invalid values are accepted quietly, the system may continue to run in a misconfigured state that is difficult to diagnose and dangerous to trust.

Validation should therefore happen at the boundary where the value enters the code. That keeps failure close to cause, reduces the amount of code that must defend against nonsense later, and makes behaviour easier to test. Failing closed is important because permissive defaults can mask mistakes. If an invalid direction silently becomes another valid direction, or an unknown policy quietly maps to a default, the system has not become robust; it has become harder to understand.

This rule also covers return values and interface contracts. The result of a service call or action request should be treated as untrusted until checked.

## 6. Rule: Assertions, guards, and invariants must be used to make assumptions explicit.

Code should state and check the assumptions on which it depends: valid state transitions, parameter ranges, non-null references where required, monotonic timing assumptions, bounded counters, and interface contracts. Assertions must not replace real runtime error handling where failure is possible in normal operation. Preconditions that may be violated by configuration, hardware state, or external input must be enforced with explicit checks and explicit failure handling.

### Rationale:

Software becomes more analysable when its assumptions are made visible. That remains true in Python. A system is easier to trust when it says, in the code, what it expects to be true before proceeding. This applies to state machines, launch preparation, topic contracts, and hardware-facing logic.

Python's built-in `assert` is not a complete answer because assertions may be disabled, and because many operational failures are not programmer errors but environmental ones. For that reason, this rule is broader than simply using `assert`. It requires explicit guards and invariant checks, with appropriate recovery or safe failure paths. The purpose is to make hidden assumptions visible and testable.

This rule should not be satisfied mechanically. Meaningless checks are of no value. The useful checks are those that establish preconditions, postconditions, or state invariants that matter to system behaviour.

## 7. Rule: Callbacks, timers, and actuator paths must be short, non-blocking, and explicit about failure.

Subscription callbacks, timer callbacks, and code that issues commands to actuators or motion-related services must do limited work, avoid blocking waits where possible, and expose failure clearly. If a command cannot be issued, acknowledged, completed, stopped, or aborted as intended, the code must enter a defined safe failure path and report the reason.

### Rationale:

In Python ROS systems, callbacks are one of the easiest places for complexity to accumulate unnoticed. A timer callback begins as a simple status update and grows into a state machine. A subscription callback begins as a message handler and grows into a history manager, transform calculator, and controller. This leads to long callback bodies, hidden latency, implicit concurrency assumptions, and poor failure visibility.

Keeping callbacks short and explicit reduces this risk. A callback should usually collect input, perform bounded work, update state, and return. If it must trigger larger behaviour, that behaviour should be delegated to smaller helpers with clear responsibilities.

The specific emphasis on actuator paths exists because failures there matter more. If a stop request fails, or an action goal cannot be cancelled cleanly, or a motion-related service becomes unavailable, the code must not merely log the event and continue as though nothing happened. Control-path failures must be treated as first-class failures, because they define whether the rover remains under control.

## 8. Rule: Launch files must orchestrate structure, not contain hidden operational logic.

Launch files may compose subsystems, select configuration, validate arguments, and wire nodes together. They should not contain complex business logic, large blocks of conditional behaviour, string-built expressions where ordinary structure would suffice, or operational policy that belongs inside nodes or configuration.

### Rationale:

Launch code is often treated as harmless glue, but in a ROS rover stack it can become one of the most operationally important parts of the system. A large launch file may decide which nodes exist, which parameters they receive, which checks are performed, and which failure paths are possible at start-up. Once this logic becomes large and string-driven, it is difficult to test, difficult to review, and difficult to analyse.

The purpose of this rule is to keep launch code structural. It should describe how the system is assembled, not become a hidden programming language for mission behaviour. If operational logic is important enough to matter, it is important enough to live in ordinary code where it can be tested and analysed properly.

This rule also helps with modularity. Shorter launch files encourage separation into reusable sub-launches and clearer subsystem boundaries.

## 9. Rule: All production code must pass automated static analysis, type checking, and tests in continuous integration with zero unexplained warnings.

All production Python packages must be checked in CI with an enforced linter, an enforced type checker, and the existing ROS lint and test machinery. Complexity and maintainability checks should also be applied where practical. Warnings should be treated as defects to be resolved or justified locally and explicitly. Broad suppression of warnings is not acceptable.

### Rationale:

A small set of coding rules only matters if compliance can be checked routinely and early. Static analysis tools, type checkers, and test suites provide that routine check. They also catch classes of defects that manual review will miss, especially in a growing codebase with many modules and many contributors.

Typing matters here not because Python should imitate a statically typed language for its own sake, but because type information makes interfaces clearer and makes analysis more powerful. Linters and complexity checks matter not because style is sacred, but because they expose structural problems before those problems harden into the architecture.

Zero unexplained warnings is important because warnings that are tolerated become invisible. If a tool is confused by the code, the safer default is usually to simplify the code, not to assume the tool is mistaken. Where suppression is necessary, it should be narrow, local, and justified so that it can be revisited later.

Testing is included in this rule because static checks alone do not establish correct behaviour. In a rover stack, tests must cover not only nominal paths but also timeout paths, stale data, unavailable services, bad parameters, shutdown, and recovery behaviour.
