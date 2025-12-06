import React, { useEffect, useState } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    setIsVisible(true);
  }, []);

  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroContent}>
        <div className={`${styles.heroText} ${isVisible ? styles.fadeInUp : ''}`}>
          <h1 className={styles.heroTitle}>
            Master Physical AI & <br />
            <span className={styles.gradient}>Humanoid Robotics</span>
          </h1>
          <p className={styles.heroSubtitle}>
            From ROS 2 to Autonomous Humanoids — Build the Future of Embodied Intelligence
          </p>
          <div className={styles.buttons}>
            <Link
              className={styles.buttonPrimary}
              to="/docs/intro">
              Start Learning →
            </Link>
            <Link
              className={styles.buttonSecondary}
              to="/docs/intro">
              View Curriculum
            </Link>
          </div>
        </div>
        <div className={styles.heroAnimation}>
          <div className={styles.robotIcon}>🤖</div>
        </div>
      </div>
      <div className={styles.scrollIndicator}>
        <span>Scroll to explore</span>
        <div className={styles.scrollArrow}>↓</div>
      </div>
    </header>
  );
}

function CourseOverview() {
  const features = [
    {
      icon: '📚',
      title: '4 Core Modules',
      description: 'Master ROS 2, Gazebo, NVIDIA Isaac, and VLA systems',
    },
    {
      icon: '🤖',
      title: 'Hands-On Projects',
      description: 'Build autonomous humanoid robots from simulation to reality',
    },
    {
      icon: '📅',
      title: '13-Week Curriculum',
      description: 'Structured learning path from basics to advanced capstone',
    },
    {
      icon: '🛠️',
      title: 'Industry Tools',
      description: 'Learn NVIDIA Isaac, ROS 2, Gazebo, and modern AI frameworks',
    },
  ];

  return (
    <section className={styles.overview}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Why This Course?</h2>
        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <div key={idx} className={styles.featureCard}>
              <div className={styles.featureIcon}>{feature.icon}</div>
              <h3>{feature.title}</h3>
              <p>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ModuleShowcase() {
  const modules = [
    {
      number: '01',
      title: 'The Robotic Nervous System',
      subtitle: 'ROS 2 Fundamentals',
      description: 'Master the middleware that powers modern robots. Learn nodes, topics, services, and build your first robotic system.',
      link: '/docs/module-1-ros2/intro-physical-ai',
      visual: '🔧',
    },
    {
      number: '02',
      title: 'The Digital Twin',
      subtitle: 'Simulation with Gazebo & Unity',
      description: 'Create photorealistic robot simulations. Master physics engines, URDF modeling, and sensor simulation.',
      link: '/docs/module-2-simulation/gazebo',
      visual: '🎮',
    },
    {
      number: '03',
      title: 'The AI-Robot Brain',
      subtitle: 'NVIDIA Isaac Platform',
      description: "Leverage NVIDIA's AI-powered robotics platform. Learn perception, navigation, and sim-to-real transfer.",
      link: '/docs/module-3-isaac/nvidia-isaac',
      visual: '🧠',
    },
    {
      number: '04',
      title: 'Vision-Language-Action',
      subtitle: 'Conversational Humanoids',
      description: 'Integrate GPT models with robots. Build voice-controlled humanoids that understand and act on natural language.',
      link: '/docs/module-4-humanoid/conversational-robotics',
      visual: '💬',
    },
  ];

  return (
    <section className={styles.modules}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Course Modules</h2>
        <p className={styles.sectionSubtitle}>
          A comprehensive journey from fundamentals to advanced robotics
        </p>
        {modules.map((module, idx) => (
          <div
            key={idx}
            className={`${styles.moduleCard} ${idx % 2 === 1 ? styles.moduleReverse : ''}`}>
            <div className={styles.moduleVisual}>
              <div className={styles.moduleIcon}>{module.visual}</div>
              <div className={styles.moduleNumber}>{module.number}</div>
            </div>
            <div className={styles.moduleContent}>
              <h3 className={styles.moduleTitle}>{module.title}</h3>
              <h4 className={styles.moduleSubtitle}>{module.subtitle}</h4>
              <p className={styles.moduleDescription}>{module.description}</p>
              <Link to={module.link} className={styles.moduleLink}>
                Explore Module {module.number} →
              </Link>
            </div>
          </div>
        ))}
      </div>
    </section>
  );
}

function TechStack() {
  const technologies = [
    'ROS 2',
    'NVIDIA Isaac',
    'Gazebo',
    'Unity',
    'OpenAI',
    'Python',
    'C++',
    'Jetson',
  ];

  return (
    <section className={styles.techStack}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Technology Stack</h2>
        <p className={styles.sectionSubtitle}>
          Learn industry-standard tools used by leading robotics companies
        </p>
        <div className={styles.techGrid}>
          {technologies.map((tech, idx) => (
            <div key={idx} className={styles.techCard}>
              <span>{tech}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function HardwareRequirements() {
  return (
    <section className={styles.hardware}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>What You'll Need</h2>
        <div className={styles.hardwareGrid}>
          <div className={styles.hardwareCard}>
            <h3>💻 Digital Twin Setup</h3>
            <ul>
              <li>✓ RTX 4070+ GPU</li>
              <li>✓ 64GB RAM</li>
              <li>✓ Ubuntu 22.04</li>
              <li>✓ High-performance workstation</li>
            </ul>
          </div>
          <div className={styles.hardwareCard}>
            <h3>🤖 Physical Lab (Optional)</h3>
            <ul>
              <li>✓ Jetson Orin Nano</li>
              <li>✓ Intel RealSense Camera</li>
              <li>✓ Unitree Robot</li>
              <li>✓ Development kit</li>
            </ul>
          </div>
        </div>
        <div className={styles.hardwareCta}>
          <Link to="/docs/hardware-requirements" className={styles.linkButton}>
            View Full Requirements →
          </Link>
        </div>
      </div>
    </section>
  );
}

function CapstonePreview() {
  return (
    <section className={styles.capstone}>
      <div className={styles.container}>
        <div className={styles.capstoneContent}>
          <span className={styles.capstoneBadge}>FINAL PROJECT</span>
          <h2 className={styles.capstoneTitle}>The Autonomous Humanoid</h2>
          <p className={styles.capstoneDescription}>
            Build a simulated humanoid that receives voice commands, plans paths,
            navigates obstacles, and manipulates objects using computer vision.
          </p>
          <div className={styles.capstoneFeatures}>
            <div className={styles.capstoneFeature}>
              <span className={styles.checkmark}>✓</span>
              <span>Voice command processing with Whisper</span>
            </div>
            <div className={styles.capstoneFeature}>
              <span className={styles.checkmark}>✓</span>
              <span>LLM-powered action planning</span>
            </div>
            <div className={styles.capstoneFeature}>
              <span className={styles.checkmark}>✓</span>
              <span>SLAM navigation</span>
            </div>
            <div className={styles.capstoneFeature}>
              <span className={styles.checkmark}>✓</span>
              <span>Object detection and manipulation</span>
            </div>
          </div>
          <Link to="/docs/capstone-project" className={styles.buttonPrimary}>
            See Capstone Details →
          </Link>
        </div>
      </div>
    </section>
  );
}

function FinalCTA() {
  return (
    <section className={styles.finalCta}>
      <div className={styles.container}>
        <h2 className={styles.ctaTitle}>Ready to Build the Future?</h2>
        <p className={styles.ctaSubtitle}>
          Start your journey into Physical AI and Humanoid Robotics today
        </p>
        <div className={styles.ctaButtons}>
          <Link to="/docs/intro" className={styles.buttonPrimary}>
            Begin Learning Now
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Master Physical AI and Humanoid Robotics - From ROS 2 to Autonomous Humanoids">
      <HomepageHeader />
      <main>
        <CourseOverview />
        <ModuleShowcase />
        <TechStack />
        <HardwareRequirements />
        <CapstonePreview />
        <FinalCTA />
      </main>
    </Layout>
  );
}