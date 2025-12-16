# Data Model: Physical AI & Humanoid Robotics Book

**Date**: December 15, 2025  
**Feature**: 001-physical-ai-book  
**Status**: Design Complete

## Overview

This document defines the data structures and entities used throughout the Physical AI & Humanoid Robotics book project. These models provide the foundation for organizing content, tracking progress, and maintaining consistency across all chapters and modules.

## Entity Definitions

### Chapter Entity

**Description**: Represents a single chapter in the book with its associated content, exercises, and metadata.

**Properties**:
- `id` (string): Unique identifier following format "ch{number}-{topic}" (e.g., "ch03-nodes-topics-services")
- `title` (string): Descriptive title of the chapter
- `module` (string): Module identifier this chapter belongs to
- `number` (integer): Chapter number in sequence
- `learning_objectives` (array of strings): Specific learning outcomes for the chapter
- `prerequisites` (array of strings): Knowledge required before reading this chapter
- `content` (string): Markdown formatted content of the chapter
- `exercises` (array of Exercise): Hands-on activities for the chapter
- `code_examples` (array of CodeExample): Implementation examples
- `diagrams` (array of strings): List of diagram identifiers used in chapter
- `key_terms` (array of strings): Important terminology defined in the chapter
- `citations` (array of Citation): Academic references used in the chapter
- `estimated_completion_time` (integer): Time in minutes to complete chapter with exercises
- `difficulty_level` (enum: beginner, intermediate, advanced): Pedagogical difficulty rating

### Exercise Entity

**Description**: Represents a hands-on activity that reinforces concepts from a chapter.

**Properties**:
- `id` (string): Unique identifier within the chapter context
- `title` (string): Brief descriptive title
- `description` (string): Detailed explanation of the exercise
- `prerequisites` (array of strings): What the reader needs to complete the exercise
- `steps` (array of strings): Step-by-step instructions
- `success_criteria` (array of strings): How to verify the exercise was completed successfully
- `difficulty` (enum: beginner, intermediate, advanced): Implementation difficulty rating
- `estimated_time` (integer): Time in minutes to complete
- `related_concepts` (array of strings): Book concepts reinforced by this exercise
- `troubleshooting_tips` (array of strings): Common issues and solutions
- `validation_commands` (array of strings): Terminal commands to verify completion

### CodeExample Entity

**Description**: Represents a code snippet or complete program example from the book.

**Properties**:
- `id` (string): Unique identifier within the chapter context
- `language` (string): Programming language (e.g., "python", "cpp", "launch", "urdf")
- `purpose` (string): What concept or technique this example demonstrates
- `code` (string): The actual code with proper formatting
- `setup_instructions` (string): How to set up the environment for this example
- `execution` (string): How to run the code
- `expected_output` (string): What output to expect from the code
- `explanation` (string): Line-by-line or conceptual explanation
- `related_exercises` (array of strings): Exercises that use or extend this example
- `dependencies` (array of strings): ROS packages or Python libraries required
- `file_path` (string): Where this example should be placed in the workspace

### Diagram Entity

**Description**: Represents a visual element that illustrates concepts in the book.

**Properties**:
- `id` (string): Unique identifier for the diagram
- `title` (string): Descriptive title
- `description` (string): What the diagram illustrates
- `speckitplus_source` (string): Source definition for SpeckitPlus generation
- `type` (enum: architectural, flowchart, sequence, system_interaction, sensor_data, ai_pipeline): Diagram category
- `related_chapters` (array of strings): Chapters that reference this diagram
- `caption` (string): Caption for the diagram in the book
- `alternative_text` (string): Accessibility description

### Citation Entity

**Description**: Represents an academic reference or source cited in the book.

**Properties**:
- `id` (string): Unique citation identifier
- `type` (enum: academic_paper, book, online_resource, documentation, thesis): Reference type
- `authors` (array of strings): Author names in "Lastname, First Initial" format
- `title` (string): Work title
- `journal` (string): Journal name (for papers)
- `publisher` (string): Publisher name (for books)
- `year` (integer): Publication year
- `doi` (string): Digital Object Identifier (for papers)
- `url` (string): Link to online resources
- `access_date` (string): Date when online resource was accessed (for non-papers)
- `apa_format` (string): Fully formatted APA citation
- `related_concepts` (array of strings): Book concepts this source relates to

### Module Entity

**Description**: Represents one of the five major modules in the book.

**Properties**:
- `id` (string): Module identifier (e.g., "m1-robotic-nervous-system")
- `name` (string): Module name
- `description` (string): Overview of what the module covers
- `learning_outcomes` (array of strings): What students will be able to do after module
- `chapters` (array of strings): Chapter IDs belonging to this module
- `prerequisites` (array of strings): What's needed before starting this module
- `capstone_project` (string): Description of module's capstone component
- `estimated_duration` (integer): Time in weeks to complete module

## Relationship Mappings

### Chapter Relationships
- One Module contains many Chapters (1:M)
- One Chapter contains many Exercises (1:M)
- One Chapter contains many CodeExamples (1:M)
- One Chapter references many Diagrams (1:M)
- One Chapter uses many Citations (1:M)

### Cross-module Relationships
- CodeExamples from early modules may be extended in later modules
- Diagrams may be referenced across multiple chapters in different modules
- Exercises in later modules may build upon concepts from earlier modules
- Capstone projects integrate concepts across multiple modules

## Validation Rules

### Content Validation
- Each Chapter must have 1-3 learning objectives
- Each Chapter must include at least 1 Exercise
- Each Chapter must include at least 1 CodeExample for implementation-focused chapters
- Each Chapter must include at least 1 Citation
- Difficulty levels must align with content complexity

### Academic Validation
- At least 50% of Citations must be peer-reviewed academic sources
- All Citations must be in proper APA format
- Each technical concept must be supported by at least one Citation
- All claims must be verifiable through Citations or CodeExamples

### Technical Validation
- All CodeExamples must be syntactically correct
- All CodeExamples must be executable in the described environment
- All Exercise success criteria must be measurable
- All Diagrams must be reproducible with SpeckitPlus

## State Transitions

### Chapter States
```
Draft → In Review → Approved → Published
```

- **Draft**: Initial content creation stage
- **In Review**: Peer review and technical verification
- **Approved**: Quality assurance complete
- **Published**: Included in final book

### Exercise States
```
Proposed → Designed → Tested → Validated
```

- **Proposed**: Idea stage
- **Designed**: Complete instructions created
- **Tested**: Verified in clean environment
- **Validated**: Success criteria confirmed

## Implementation Guidelines

### Naming Conventions
- All IDs use lowercase with hyphens as separators
- Chapter and module IDs follow the pattern: `{type}{number}-{description}`
- File names match entity IDs when appropriate
- CodeExample files include language extension (`.py`, `.cpp`, etc.)

### Versioning
- Entities may have version numbers for tracking changes
- Major content changes trigger new version numbers
- Academic Citations should track version of referenced work

## Example Instances

### Example Chapter
```
{
  "id": "ch03-nodes-topics-services",
  "title": "ROS 2 Nodes, Topics, and Services",
  "module": "m1-robotic-nervous-system",
  "number": 3,
  "learning_objectives": [
    "Create and run ROS 2 nodes",
    "Implement publisher-subscriber communication",
    "Design and use service-based interactions"
  ],
  "prerequisites": ["Basic Python programming", "Chapter 1-2 concepts"],
  "exercises": ["exercise-nodes", "exercise-pubsub", "exercise-services"],
  "code_examples": ["minimal_publisher", "minimal_subscriber", "add_two_ints_server"],
  "diagrams": ["ros2_computational_graph", "pubsub_pattern", "service_interaction"],
  "citations": ["quigley-ros-book", "ros2-design-paper"],
  "estimated_completion_time": 120,
  "difficulty_level": "intermediate"
}
```

### Example Exercise
```
{
  "id": "exercise-pubsub",
  "title": "Temperature Monitoring System",
  "description": "Implement a ROS 2 system that publishes temperature readings and subscribes to them to trigger alerts",
  "success_criteria": [
    "Publisher node successfully publishes temperature data",
    "Subscriber node receives and processes messages",
    "System triggers alert when temperature exceeds threshold"
  ],
  "estimated_time": 45,
  "difficulty": "intermediate"
}
```

---

## Notes

- This data model supports the SpeckitPlus-driven development approach used in the book
- Entities are designed to support both automated processing and human readability
- Relationships enable navigation between concepts for enhanced learning
- Validation rules ensure academic and technical quality standards