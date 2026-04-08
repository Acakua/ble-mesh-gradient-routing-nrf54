# ADR: Documentation Structure for Stress Test Guide

## Context and Problem Statement
The GRADIENT_SRV project lacks a comprehensive guide for running stress tests. Users need clear instructions on how to set up the environment, initiate tests, and collect/analyze data.

## Design Decisions
1. **Target Audience**: Researchers and developers working with the BLE Mesh Gradient Routing system.
2. **Document Format**: Multi-section `README.md` in the project root.
3. **Sections**:
   - **Prerequisites**: Software (Python, Zephyr SDK) and Hardware (nRF54/52 development kits).
   - **Environment Setup**: Python dependencies, UART configuration.
   - **Test Modes**: 
     - **Broadcast Mode** (Automatic test for all nodes).
     - **Downlink Stress Mode** (Targeted testing).
   - **Real-time Monitoring**: Instruction on using the Web Dashboard (`index.html`).
   - **Data Collection**: Where to find CSV logs and what the fields mean.
4. **Language**: The README will be in English for broad compatibility, but I will provide a Vietnamese version or summary if requested (user's primary language is Vietnamese). I'll start with English as a standard.

## Consequences
- **Positive**: Standardized testing procedure, easier onboarding for new users.
- **Negative**: documentation needs maintenance when codebase changes.
