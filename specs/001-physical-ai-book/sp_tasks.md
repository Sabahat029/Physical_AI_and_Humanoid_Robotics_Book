# sp.tasks - Physical_AI_and_Humanoid_Robotics_Book

This file defines the **strict, sequential task graph** required to move the project from research to implementation readiness. Each phase has **hard dependencies** and ends with a **checkpoint**. No task may begin until its predecessor is complete.

## Phase 1 — Research Foundation

**Path type:** Sequential (strict dependency)

┌────────────────────────────────────────────────────────────┐
│ Task 1.1: Define Research Scope & Keywords                 │
│   - Physical AI, Humanoid Robotics, Embodied AI            │
│   - Mechanical, Electrical, Control, AI layers             │
│      ↓                                                     │
│ Task 1.2: Collect Authoritative Sources                    │
│   - Textbooks, IEEE/ACM papers, arXiv, standards            │
│   - Industry references (Boston Dynamics, Tesla, etc.)     │
│      ↓                                                     │
│ Task 1.3: Evaluate & Filter Sources                        │
│   - Relevance, academic credibility, recency               │
│      ↓                                                     │
│ Task 1.4: Extract Core Concepts & Terminology              │
│   - Definitions, models, equations, architectures           │
│      ↓                                                     │
│ Task 1.5: Draft High-Level Chapter Map                     │
│   - 10-chapter structure aligned with /sp.specification    │
│      ↓ [CHECKPOINT 1 — Research Foundation Locked]         │
└────────────────────────────────────────────────────────────┘

---

## Phase 2 — Chapter-wise Content Research

**Path type:** Sequential (depends on Phase 1)

┌────────────────────────────────────────────────────────────┐
│ Task 2.1: Research Chapter 1–3 Sources (depends on 1.5)    │
│   - Physical AI foundations, mechanics, hardware            │
│      ↓                                                     │
│ Task 2.2: Synthesize Chapter 1–3 Key Points                │
│   - Concepts → explanations → diagrams                     │
│      ↓                                                     │
│ Task 2.3: Research Chapter 4–6 Sources                     │
│   - Perception, control systems, AI methods                │
│      ↓                                                     │
│ Task 2.4: Synthesize Chapter 4–6 Key Points                │
│      ↓                                                     │
│ Task 2.5: Research Chapter 7–10 Sources                    │
│   - Architectures, build pipeline, software, applications  │
│      ↓                                                     │
│ Task 2.6: Synthesize Chapter 7–10 Key Points               │
│      ↓                                                     │
│ Task 2.7: Organize All Research by Chapter & Section       │
│   - Map content to book folders                             │
│      ↓ [CHECKPOINT 2 — Research Complete]                  │
└────────────────────────────────────────────────────────────┘

---

## Phase 3 — Technical Structuring & Intelligence Components

**Path type:** Sequential (depends on Phase 2)

┌────────────────────────────────────────────────────────────┐
│ Task 3.1: Identify Reusable Skills                         │
│   - IK solvers, SLAM, gait generation, RL skills            │
│      ↓                                                     │
│ Task 3.2: Define Subagents                                 │
│   - perception_agent, control_agent, learning_agent        │
│      ↓                                                     │
│ Task 3.3: Specify Tools                                    │
│   - Simulators, diagram builders, MCQ generator             │
│      ↓                                                     │
│ Task 3.4: Map Skills/Subagents/Tools to Chapters           │
│      ↓                                                     │
│ Task 3.5: Draft Agent Architecture Diagrams                │
│      ↓ [CHECKPOINT 3 — Vertical Intelligence Frozen]       │
└────────────────────────────────────────────────────────────┘

---

## Phase 4 — Writing Preparation & Planning

**Path type:** Linear (depends on Phase 3)

┌────────────────────────────────────────────────────────────┐
│ Task 4.1: Create Chapter-wise Writing Outlines             │
│   - Section order, learning flow                            │
│      ↓                                                     │
│ Task 4.2: Define Code vs Theory Boundaries                 │
│   - What goes into book vs GitHub                           │
│      ↓                                                     │
│ Task 4.3: Establish Citation & Diagram Standards           │
│      ↓                                                     │
│ Task 4.4: Prepare GitHub Repository Structure              │
│      ↓                                                     │
│ Task 4.5: Final Pre-Implementation Review                  │
│      ↓ [CHECKPOINT 4 — READY FOR /sp.implementation]      │
└────────────────────────────────────────────────────────────┘

---

## Dependency Legend

* Each task **must complete before the next begins**
* Checkpoints represent **hard freeze points**
* No implementation work starts before Checkpoint 4

---

**Status:** This /sp.tasks file fully bridges `/sp.specification` → `/sp.implementation`.
Once Checkpoint 4 is reached, the project is implementation-authorized.