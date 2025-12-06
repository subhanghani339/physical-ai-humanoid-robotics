# Physical AI & Humanoid Robotics Textbook

## Project Goal
Create a complete Docusaurus-based textbook with the following structure:

## Setup Requirements
1. Initialize Docusaurus v3 classic template
2. Project name: "Physical AI & Humanoid Robotics"
3. Configure for GitHub Pages deployment
4. Enable Mermaid diagrams for technical illustrations
5. Add code syntax highlighting for Python, C++, and YAML

## Book Structure

Create the following pages in the docs/ folder:

### 1. docs/intro.md
- Welcome message
- Course overview
- What students will learn
- What students will build (autonomous humanoid robot)

### 2. docs/hardware-requirements.md
- Workstation requirements (RTX GPU, RAM, etc.)
- Edge computing kit (Jetson Orin Nano)
- Robot options (Unitree Go2, G1)
- Cost breakdown tables

### 3. docs/module-1-ros2/intro-physical-ai.md
Title: "Introduction to Physical AI (Weeks 1-2)"

Content to include:
- What is Physical AI and embodied intelligence
- Difference between digital AI and Physical AI
- Real-world examples: self-driving cars, warehouse robots, humanoid assistants
- Overview of sensor systems:
  * LIDAR - explain with diagram
  * RGB Cameras
  * Depth Cameras (Intel RealSense)
  * IMU (Inertial Measurement Unit)
  * Force/Torque sensors

Include a Mermaid diagram showing how sensors feed data to AI systems.

### 4. docs/module-1-ros2/ros2-fundamentals.md
Title: "ROS 2 Fundamentals (Weeks 3-5)"

Content to include:
- ROS 2 architecture overview
- Nodes, Topics, Services, and Actions explained
- Code example: Simple publisher node in Python
- Code example: Simple subscriber node in Python
- Explanation of launch files
- Practical exercise: Build a temperature monitoring system

Include Python code blocks with comments.

### 5. docs/module-2-simulation/gazebo.md
Title: "Robot Simulation with Gazebo (Weeks 6-7)"

Content to include:
- Why simulation matters
- Gazebo architecture
- URDF robot description format
- Example URDF file for a simple robot
- Physics simulation basics
- Sensor simulation
- Introduction to Unity as alternative

Include code examples of URDF and launch files.

### 6. docs/module-3-isaac/nvidia-isaac.md
Title: "NVIDIA Isaac Platform (Weeks 8-10)"

Content to include:
- Overview of NVIDIA Isaac SDK
- Isaac Sim for photorealistic simulation
- Isaac ROS for hardware acceleration
- AI-powered perception pipeline
- Reinforcement learning basics
- Sim-to-real transfer techniques

### 7. docs/module-4-humanoid/humanoid-development.md
Title: "Humanoid Robot Development (Weeks 11-12)"

Content to include:
- Humanoid kinematics and dynamics
- Bipedal locomotion and balance
- Manipulation and grasping
- Natural human-robot interaction

### 8. docs/module-4-humanoid/conversational-robotics.md
Title: "Conversational Robotics (Week 13)"

Content to include:
- Integrating GPT models with robots
- OpenAI Whisper for voice commands
- Natural language to robot actions
- Multi-modal interaction

### 9. docs/capstone-project.md
Title: "Capstone Project: The Autonomous Humanoid"

Content to include:
- Project overview and requirements
- Step-by-step implementation guide
- Voice command → Planning → Navigation → Object manipulation
- Assessment criteria
- Submission guidelines

## Docusaurus Configuration

Configure docusaurus.config.js with:
- Site title: "Physical AI & Humanoid Robotics"
- Tagline: "Bridging Digital Intelligence and Physical Embodiment"
- Dark mode enabled by default
- Navbar with links to GitHub repo
- Footer with course information

## Sidebar Configuration

Create sidebars.js with this structure:
- Introduction
- Hardware Requirements
- Module 1: ROS 2
  - Introduction to Physical AI
  - ROS 2 Fundamentals
- Module 2: Simulation
  - Gazebo & Unity
- Module 3: NVIDIA Isaac
  - Isaac Platform
- Module 4: Humanoids
  - Humanoid Development
  - Conversational Robotics
- Capstone Project

## Styling
- Use professional blue/purple gradient theme
- Add custom CSS for code blocks
- Make it mobile responsive
- Add a floating chatbot button (placeholder for now)