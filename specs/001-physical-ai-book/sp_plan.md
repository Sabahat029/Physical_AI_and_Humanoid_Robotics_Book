# Physical AI & Humanoid Robotics — AI-Native, SpeckitPlus-Driven Book

**Title**: Physical AI & Humanoid Robotics — AI-Native, SpeckitPlus-Driven Book
**Theme**: Embodied AI — linking digital intelligence and physical embodiment
**Branch**: 001-physical-ai-book
**Date**: December 15, 2025
**Status**: Active Planning

## Summary

Expanding on the feature specification, this plan outlines the comprehensive development approach for creating a graduate-level textbook on Physical AI and Humanoid Robotics. The book will bridge theoretical concepts of embodied intelligence with practical implementation using ROS 2, simulation environments, and AI integration.

## Objectives

- Teach Physical AI principles and embodied intelligence
- Master ROS 2, Gazebo/Unity simulation, NVIDIA Isaac AI
- Integrate GPT conversational AI for humanoids
- Provide conceptual + practical understanding of embodied AI

## Audience

- Graduate students/researchers in AI & Robotics
- ML practitioners exploring embodiment
- Roboticists and cognitive scientists

## Modules

1. Robotic Nervous System (ROS 2)
   - ROS 2 Nodes, Topics, Services, Actions
   - Python Agents → ROS via rclpy
   - URDF, ROS packages, launch files

2. Digital Twin (Gazebo & Unity)
   - Physics & environment simulation
   - Sensors: LiDAR, Depth, IMUs
   - Integration with ROS controllers

3. AI-Robot Brain (NVIDIA Isaac)
   - Isaac Sim, Isaac ROS, VSLAM, Nav2
   - Reinforcement learning, Sim-to-real

4. Vision-Language-Action (VLA)
   - Whisper Voice-to-Action
   - LLM cognitive planning → ROS actions
   - Multi-modal interaction
   - Capstone: autonomous humanoid executes tasks

## Weekly Breakdown

1-2: Introduction — Physical AI, embodied intelligence, sensors
3-5: ROS 2 fundamentals (Module 1)
6-7: Simulation with Gazebo & Unity (Module 2)
8-10: NVIDIA Isaac (Module 3)
11-12: Humanoid development (Module 4)
13: Conversational Robotics (Module 4)

## Success Criteria

- Verified, traceable claims
- ADRs & PHRs to document reasoning improvements across chapters
- Reusable components (Skills/Subagents/Tools) to be created for recurring patterns
- Final book in Markdown + PDF, Docusaurus-ready

## Constraints

- 20–30 chapters, 20k–30k words
- APA citations, 50% primary/peer-reviewed sources
- AI diagrams via SpeckitPlus
- Zero plagiarism
- Spec-driven workflow with GitHub commits

## Module 1: Robotic Nervous System (ROS 2)

### Weeks 1-2: Introduction to Physical AI & Sensor Systems
**Chapters 1-2**
- Foundations of Physical AI & Embodied Intelligence
- Sensor systems: LIDAR, cameras, IMUs
- Introduction to ROS 2 architecture
- ADR: Define ROS package standards
- PHR: Sensor diagram generation

### Weeks 3-5: ROS 2 Fundamentals
**Chapters 3-5**
- Nodes, topics, services, actions
- Python Agents → ROS 2 controllers (rclpy)
- URDF for humanoid description
- Capstone milestone: ROS 2 node setup & Python bridging
- ADR: Node naming & URDF conventions
- PHR: ROS-Python scaffold prompts

## Module 2: Digital Twin (Gazebo & Unity)

### Weeks 6-7
**Chapters 6-9**
- Gazebo simulation setup
- Physics, gravity, collisions
- Unity visualization
- Simulating sensors (LiDAR, Depth, IMU)
- Capstone milestone: Environment + sensor integration
- ADR: Simulation parameters & sensor calibration
- PHR: Scene & sensor prompt templates
- Docusaurus Documentation Link: ./docs-site/modules/module-2-digital-twin

## Module 3: AI-Robot Brain (NVIDIA Isaac™)

### Weeks 8-10
**Chapters 10-12**
- NVIDIA Isaac Sim & SDK overview
- AI-powered perception & VSLAM
- Path planning with Nav2
- Capstone milestone: Isaac perception + navigation setup
- ADR: Isaac node configuration & VSLAM tuning
- PHR: Isaac integration prompts
- Docusaurus Documentation Link: ./docs-site/modules/module-3-ai-robot-brain

## Module 4: Vision-Language-Action (VLA)

### Weeks 11-12
**Chapters 13-15**
- Voice-to-Action integration (Whisper)
- Cognitive Planning via LLMs
- Capstone milestone: Voice command → action pipeline
- ADR: VLA mapping & failure handling
- PHR: LLM task translation prompts
- Docusaurus Documentation Link: ./docs-site/modules/module-4-vla

### Week 13
**Chapter 16**
- Autonomous humanoid fetch-and-deliver
- Full system integration (ROS 2 + Gazebo + Isaac + VLA)
- Testing & evaluation
- ADR: Final system decisions
- PHR: Complete system prompts & diagram generation
- Docusaurus Documentation Link: ./docs-site/modules/capstone-project

## Supplementary Chapters & Appendix

### Weeks 1-13
**Chapters 17-20**
- Supplementary chapters / exercises
- Bonus diagrams & tables
- Reuse Skills / Subagents / Tools across chapters
- Document all ADRs & PHRs
- Appendix: Hardware specs, lab setup, edge kits
- Export final PDF
- Docusaurus Documentation Link: ./docs-site/supplementary

## Reusable Skills / Subagents / Tools

### Purpose
- Create modular, reusable intelligence components
- Accelerate project development across chapters
- Document and link all ADRs, PHRs, and code examples

### Skills (Atomic Capabilities)
- ROS 2 Node Creator → Automates package scaffolding
- URDF Validator → Checks humanoid robot description
- Sensor Simulator → Generates synthetic sensor data for LiDAR, IMU, RGB-D
- Voice-to-Action Translator → Converts Whisper audio input to ROS 2 commands
- VLA Planner → Converts natural language tasks into step-by-step robot actions
- Navigation Planner → Nav2 path planning templates
- Perception Module → Reusable perception pipeline for Isaac Sim

### Subagents (Multi-step workflows)
- Capstone Task Executor → Orchestrates fetch-and-deliver workflow
- Simulation Builder → Creates Gazebo + Unity environments automatically
- Data Collector → Captures simulated data for reinforcement learning
- Debug Assistant → Logs and suggests fixes for ROS 2 & Isaac nodes
- System Integrator → Coordinates ROS 2, Isaac, and VLA modules for testing

### Tools
- ADR Tracker → Maintain Architectural Decision Records in Markdown
- PHR Generator → Produces Prompt-Helper Records for diagrams and AI prompts
- Diagram Exporter → Auto-generates figures for book chapters
- Version Control Wrapper → Git integration for commits & branch management
- Cloud Sync → Optional AWS/NVIDIA Omniverse deployment automation

### Integration Notes
- Skills and Subagents are referenced in all relevant modules
- Ensure each chapter links to corresponding PHRs and ADRs
- Maintain modularity to allow updates or additions without breaking other workflows
- Document usage examples in Appendix for readers to replicate
- Docusaurus Documentation Link: ./docs-site/appendix