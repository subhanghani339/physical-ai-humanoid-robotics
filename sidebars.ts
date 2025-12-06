import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    'hardware-requirements',
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module-1-ros2/intro-physical-ai',
        'module-1-ros2/ros2-fundamentals',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      items: ['module-2-simulation/gazebo'],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      items: ['module-3-isaac/nvidia-isaac'],
    },
    {
      type: 'category',
      label: 'Module 4: Humanoids',
      items: [
        'module-4-humanoid/humanoid-development',
        'module-4-humanoid/conversational-robotics',
      ],
    },
    'capstone-project',
  ],
};

export default sidebars;
