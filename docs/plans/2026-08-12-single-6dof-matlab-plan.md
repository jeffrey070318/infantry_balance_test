# Single 6-DOF MATLAB Tutorial Implementation Plan

**Goal:** Create a tested MATLAB tutorial that reproduces one published six-state wheel-legged linear model and demonstrates continuous-time LQR stabilization.

**Architecture:** Keep the RoboMaster reference model separate from the teaching runner. A MATLAB unit test locks down matrix transcription and verifies the control properties before the plotting script is trusted.

**Tech Stack:** MATLAB R2025b, Control System Toolbox, MATLAB Unit Test Framework

---

### Task 1: Lock Down the Model Contract with a Failing Test

**Files:**
- Create: `matlab/single_6dof_tutorial/tests/test_single_6dof.m`

- [ ] Define tests for matrix dimensions, published coefficients, state metadata, and controllability.
- [ ] Run `runtests` and confirm failure because `paper_model_6dof` does not exist.

### Task 2: Implement the Published Model

**Files:**
- Create: `matlab/single_6dof_tutorial/paper_model_6dof.m`

- [ ] Transcribe the published `A` and `B` matrices exactly.
- [ ] Add state names, state units, input names, input units, leg length, and source warning.
- [ ] Run the tests and confirm the model-contract tests pass.

### Task 3: Add Closed-Loop Behavioral Tests

**Files:**
- Modify: `matlab/single_6dof_tutorial/tests/test_single_6dof.m`

- [ ] Add tests for an unstable open-loop pole, stable LQR poles, and convergence from a two-degree pitch disturbance.
- [ ] Run the test before the teaching runner exists; model-level control tests must pass independently.

### Task 4: Build the Teaching Entry Point

**Files:**
- Create: `matlab/single_6dof_tutorial/run_single_6dof.m`

- [ ] Print model dimensions, controllability rank, open-loop poles, LQR gain, and closed-loop poles.
- [ ] Simulate zero-reference initial-condition response for ten seconds.
- [ ] Plot six states in unit-compatible groups and both control inputs.
- [ ] Include assertions that stop on wrong dimensions, uncontrollability, or unstable closed-loop poles.

### Task 5: Write Desktop Instructions and Verify End to End

**Files:**
- Create: `matlab/single_6dof_tutorial/README.md`

- [ ] Document how to open the folder and run the script in the MATLAB desktop.
- [ ] Explain the state order, units, expected command-window output, and expected plots.
- [ ] Run all tests with MATLAB R2025b.
- [ ] Run `run_single_6dof` in batch mode and confirm it completes without warnings or errors.
