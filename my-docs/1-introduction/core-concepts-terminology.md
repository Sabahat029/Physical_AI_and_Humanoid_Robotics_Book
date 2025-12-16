# Core Concepts & Terminology Extraction

**Date**: December 15, 2025  
**Module**: Introduction to Physical AI & Humanoid Robotics  
**Task**: Task 1.4 - Extract Core Concepts & Terminology  
**Status**: In Progress

## Core Concept Categories

### Physical AI & Embodied Intelligence Concepts

#### 1. Embodied Cognition
**Definition**: The theory that cognitive processes are deeply rooted in the body's interactions with the environment.
**Key Aspects**:
- Thinking is constrained by the nature of the body
- Physical interactions shape cognitive development
- Intelligence emerges from sensorimotor loop
**Applications**: Humanoid robot design, learning algorithms
**Related Concepts**: Morphological computation, sensorimotor learning

#### 2. Morphological Computation
**Definition**: The idea that part of the computation required by the organism is offloaded to the morphology of the system.
**Key Aspects**:
- Body structure contributes to computational efficiency
- Passive dynamics can simplify control
- Hardware design affects cognitive capabilities
**Applications**: Joint compliance, body-part design, material selection
**Related Concepts**: Embodied intelligence, physical computation

#### 3. Sensorimotor Loop
**Definition**: The continuous cycle of sensing, processing, and acting that characterizes embodied systems.
**Key Aspects**:
- Tight coupling between perception and action
- Real-time processing requirements
- Environmental feedback integration
**Applications**: Control system design, learning algorithms
**Related Concepts**: Active inference, closed-loop control

#### 4. Active Inference
**Definition**: The process by which agents act to fulfill prior beliefs about desired sensory states.
**Key Aspects**:
- Action as inference, not just reaction
- Prediction-based behavior
- Minimization of free energy
**Applications**: Autonomous behavior, planning systems
**Related Concepts**: Predictive processing, Bayesian inference

### Mechanical Layer Concepts

#### 5. Kinematics
**Definition**: The study of motion without considering the forces that cause it.
**Key Aspects**:
- Forward kinematics: joint angles → end-effector position
- Inverse kinematics: desired position → required joint angles
- Jacobian matrices for velocity relationships
**Applications**: Robot motion planning, control systems
**Related Concepts**: Forward/Inverse kinematics, Jacobian

#### 6. Dynamics
**Definition**: The study of forces and torques that cause motion.
**Key Aspects**:
- Newton-Euler vs. Lagrangian formulations
- Rigid body dynamics
- Multi-body systems
**Applications**: Robot simulation, control design
**Related Concepts**: Inverse dynamics, forward dynamics

#### 7. Whole-Body Control
**Definition**: Control approach that considers the entire robot as a unified dynamic system.
**Key Aspects**:
- Integration of multiple control objectives
- Constraint handling
- Optimization-based control
**Applications**: Humanoid balance, manipulation
**Related Concepts**: Task prioritization, optimization control

#### 8. Center of Mass (CoM)
**Definition**: The point where the total mass of the body is assumed to be concentrated.
**Key Aspects**:
- Balance and stability analysis
- Zero Moment Point (ZMP) relationship
- Trajectory planning considerations
**Applications**: Walking pattern generation, balance control
**Related Concepts**: ZMP, balance control, stability

### Control Layer Concepts

#### 9. Robot Operating System (ROS)
**Definition**: Flexible framework for writing robot software with message-passing between processes.
**Key Aspects**:
- Distributed computing architecture
- Publish-subscribe communication model
- Package and workspace management
**Applications**: Robot software development, standardization
**Related Concepts**: ROS 2, nodes, topics, services, actions

#### 10. Inverse Kinematics (IK)
**Definition**: Mathematical process of determining joint angles required to achieve a desired end-effector pose.
**Key Aspects**:
- Analytical vs. numerical solutions
- Multiple solutions and redundancy
- Jacobian-based methods
**Applications**: Robot motion planning, manipulation
**Related Concepts**: Forward kinematics, Jacobian, redundancy

#### 11. Zero Moment Point (ZMP)
**Definition**: A point where the net moment of the ground reaction forces is zero.
**Key Aspects**:
- Balance and stability criterion
- Walking pattern generation
- Dynamic balance control
**Applications**: Humanoid locomotion, stability analysis
**Related Concepts**: Center of Mass, balance control

#### 12. Operational Space Control
**Definition**: Control methodology that operates in the task space rather than joint space.
**Key Aspects**:
- Task-specific control objectives
- Compliance control
- Prioritization between tasks
**Applications**: Whole-body control, manipulation
**Related Concepts**: Task space, joint space, inverse kinematics

### AI Layer Concepts

#### 13. Reinforcement Learning (RL)
**Definition**: Machine learning paradigm where an agent learns to make decisions by interacting with an environment.
**Key Aspects**:
- Reward signal for learning
- Exploration vs. exploitation trade-off
- Markov Decision Processes
**Applications**: Robot learning, adaptive control
**Related Concepts**: Q-learning, Policy Gradient, Deep RL

#### 14. Deep Learning
**Definition**: Machine learning using neural networks with multiple layers to extract high-level features.
**Key Aspects**:
- Hierarchical feature learning
- End-to-end learning
- Representation learning
**Applications**: Perception, control, planning
**Related Concepts**: Neural Networks, CNN, RNN, Transformers

#### 15. Sim-to-Real Transfer
**Definition**: Process of transferring knowledge or policies learned in simulation to real robots.
**Key Aspects**:
- Domain randomization
- Reality gap mitigation
- Domain adaptation
**Applications**: Robot training, policy transfer
**Related Concepts**: Domain randomization, system identification

#### 16. Visual SLAM (VSLAM)
**Definition**: Simultaneous Localization and Mapping using visual sensors.
**Key Aspects**:
- Camera pose estimation
- Scene reconstruction
- Feature tracking and matching
**Applications**: Robot navigation, mapping
**Related Concepts**: SLAM, ORB-SLAM, Visual odometry

## Critical Terminology

### Mechanical Components
- **Actuator**: Device that converts control signals into physical motion
- **Compliance**: The ability of a system to yield to external forces
- **Degrees of Freedom (DOF)**: Independent parameters defining robot configuration
- **Transmission**: Mechanism transferring power from motor to joint
- **End-effector**: Final link of robot manipulator for interaction

### Control Systems
- **Controller**: System that generates control signals to achieve desired behavior
- **Feedback**: Information about system state used for control
- **Stability**: Property of system returning to equilibrium after disturbance
- **Trajectory**: Time-varying path in state space
- **Impedance Control**: Control strategy specifying force-velocity relationship

### AI & Learning
- **Policy**: Mapping from states to actions in RL
- **State**: Complete information required to determine future behavior
- **Reward**: Signal indicating success of action in environment
- **Epoch**: Complete pass through training dataset
- **Hyperparameter**: Parameters set prior to learning process

### ROS 2 Specific
- **Node**: Process that performs computation in ROS
- **Topic**: Named bus over which nodes exchange messages
- **Publisher**: Node that sends messages on a topic
- **Subscriber**: Node that receives messages from a topic
- **Service**: Synchronous request-response communication pattern

## Mathematical Foundations

### Essential Equations

#### 1. Forward Kinematics
`T = f(θ₁, θ₂, ..., θₙ)`

#### 2. Inverse Kinematics
`θ = f⁻¹(T)`

#### 3. Newton-Euler Dynamics
`τ = H(q)q̈ + C(q, q̇)q̇ + g(q)`

#### 4. ZMP Calculation
`ZMP_x = (ΣF_z × COP_x - ΣM_y) / ΣF_z`

#### 5. Bellman Equation (RL)
`V(s) = max_a E[R(s,a) + γV(s')]`

### Coordinate Systems & Representations
- **Homogeneous Transformations**: 4x4 matrices for position and orientation
- **Quaternion**: 4-parameter representation of 3D rotation
- **Twist Coordinates**: 6-parameter representation of spatial velocity
- **Plücker Coordinates**: Representation of lines in 3D space

## Domain-Specific Models

### Humanoid Robot Models
- **Single Rigid Body Model**: Simplified balance model
- **Linear Inverted Pendulum**: Balance analysis model
- **3D Linear Inverted Pendulum**: Extension to 3D balance
- **Multi-body Model**: Full dynamic representation

### Learning Models
- **Markov Decision Process (MDP)**: Framework for sequential decision making
- **Partially Observable MDP (POMDP)**: MDP with partial observability
- **Actor-Critic**: Policy gradient RL method
- **Deep Q-Network (DQN)**: Deep RL for discrete action spaces

## Architectural Patterns

### Robot Software Architectures
- **Subsumption Architecture**: Hierarchical behavior-based approach
- **Three-Layer Architecture**: Reactivity, deliberation, execution layers
- **Behavior-Based Robotics**: Collection of concurrent behaviors
- **Task-Level Architecture**: High-level planning and decomposition

### Integration Patterns
- **Perception-Action Loop**: Continuous sensing and acting cycle
- **Model-Predictive Control**: Optimization-based control approach
- **Hierarchical Control**: Multi-level control structure
- **Distributed Control**: Decentralized control approach

## Methodology Concepts

### Evaluation Metrics
- **Tracking Error**: Difference between desired and actual trajectories
- **Stability Margins**: Robustness measures for control systems
- **Sample Efficiency**: Learning performance per training sample
- **Generalization**: Performance on unseen scenarios

### Experimental Design
- **Controlled Experiment**: Isolated variable testing
- **Cross-validation**: Model performance validation technique
- **A/B Testing**: Comparison of alternative approaches
- **Statistical Significance**: Confidence in experimental results

## Emerging Concepts

### Contemporary Research Areas
- **Neuromorphic Computing**: Brain-inspired computing for robotics
- **Continual Learning**: Lifelong learning without forgetting
- **Multi-task Learning**: Learning multiple related tasks together
- **Meta-learning**: Learning to learn efficiently

### Integration Challenges
- **Simulation-to-Reality Gap**: Differences between sim and real systems
- **Catastrophic Forgetting**: Loss of previous knowledge during learning
- **Safety in Learning**: Ensuring safe exploration during learning
- **Human-Robot Collaboration**: Effective teaming with humans

## Concept Relationships & Dependencies

### Prerequisite Relationships
- Kinematics → Dynamics → Control
- Basic RL → Deep RL → Robot Learning
- Perception → Planning → Control
- Single Body Dynamics → Multi-body Dynamics

### Integration Points
- Mechanical design constraints → Control limitations
- Perception uncertainty → Planning robustness
- Learning algorithms → Real-time requirements
- Safety requirements → System architecture

---

**Next Task**: Task 1.5 - Draft High-Level Chapter Map