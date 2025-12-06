# Homepage Redesign Specification - Physical AI & Humanoid Robotics

## Project Goal
Transform the default Docusaurus homepage into a modern, eye-catching landing page that showcases the Physical AI & Humanoid Robotics textbook with stunning visuals, animations, and compelling content.

## Design Philosophy
- Modern, futuristic tech aesthetic
- Purple/blue gradient theme (matching robotics/AI feel)
- Bold typography and hero sections
- Smooth animations and transitions
- Interactive elements
- Mobile-first responsive design
- Dark mode optimized

## Homepage Structure

### 1. Hero Section (Above the Fold)
Location: Top of page, full viewport height

Visual Design:
- Large animated gradient background (purple #667eea → blue #764ba2)
- Particle animation or geometric shapes in background
- Center-aligned content with dramatic typography

Content:
- Main Headline: "Master Physical AI & Humanoid Robotics"
- Subheadline: "From ROS 2 to Autonomous Humanoids - Build the Future of Embodied Intelligence"
- Two prominent CTA buttons:
  - Primary: "Start Learning" (links to /docs/intro)
  - Secondary: "View Curriculum" (links to course structure)
- Animated robot illustration or 3D model preview
- Scroll indicator at bottom

Technical Requirements:
- Hero height: 100vh (full viewport)
- Text animation on load (fade in + slide up)
- Background gradient animation (subtle movement)
- Responsive text sizes (clamp function for fluid typography)

### 2. Course Overview Section
Visual Design:
- Clean white/dark background (theme-aware)
- Grid layout for key features
- Icon + title + description cards
- Hover effects on cards

Content:
Display 4 key highlights in card format:

Card 1: "4 Core Modules"
- Icon: 📚
- Description: "Master ROS 2, Gazebo, NVIDIA Isaac, and VLA systems"

Card 2: "Hands-On Projects"
- Icon: 🤖
- Description: "Build autonomous humanoid robots from simulation to reality"

Card 3: "13-Week Curriculum"
- Icon: 📅
- Description: "Structured learning path from basics to advanced capstone"

Card 4: "Industry Tools"
- Icon: 🛠️
- Description: "Learn NVIDIA Isaac, ROS 2, Gazebo, and modern AI frameworks"

Technical Requirements:
- CSS Grid: 2x2 on desktop, 1 column on mobile
- Cards with shadow and hover lift effect
- Smooth transitions (transform + box-shadow)
- Icons can be emoji or lucide-react icons

### 3. Module Showcase Section
Visual Design:
- Alternating layout (image left/right)
- Large section titles
- Visual previews for each module
- Gradient accents

Content:
Display all 4 modules with previews:

Module 1: The Robotic Nervous System
- Title: "ROS 2 Fundamentals"
- Description: "Master the middleware that powers modern robots. Learn nodes, topics, services, and build your first robotic system."
- Visual: Code snippet preview or ROS 2 diagram
- Link: "Explore Module 1 →"

Module 2: The Digital Twin
- Title: "Simulation with Gazebo & Unity"
- Description: "Create photorealistic robot simulations. Master physics engines, URDF modeling, and sensor simulation."
- Visual: Gazebo screenshot or 3D render
- Link: "Explore Module 2 →"

Module 3: The AI-Robot Brain
- Title: "NVIDIA Isaac Platform"
- Description: "Leverage NVIDIA's AI-powered robotics platform. Learn perception, navigation, and sim-to-real transfer."
- Visual: Isaac Sim interface or robot vision
- Link: "Explore Module 3 →"

Module 4: Vision-Language-Action
- Title: "Conversational Humanoids"
- Description: "Integrate GPT models with robots. Build voice-controlled humanoids that understand and act on natural language."
- Visual: Humanoid robot + speech bubbles
- Link: "Explore Module 4 →"

Technical Requirements:
- Alternating flex layout (reverse every other module)
- Image width: 50% on desktop, 100% on mobile
- Lazy loading for images
- Smooth scroll reveal animations (intersection observer)
- Each module links to its docs section

### 4. Technology Stack Section
Visual Design:
- Dark background with glowing cards
- Logo grid layout
- Subtle glow effects on hover

Content:
Display logos/names of key technologies:
- ROS 2
- NVIDIA Isaac
- Gazebo
- Unity
- OpenAI / GPT
- Python
- C++
- Jetson

Technical Requirements:
- Grid layout: 4 columns on desktop, 2 on mobile
- Cards with border glow on hover
- Technology icons (use simple text or import logos)

### 5. Hardware Requirements Callout
Visual Design:
- Gradient background section
- Two-column layout
- Icon-based list

Content:
Left Column:
- Title: "What You'll Need"
- Brief description of hardware requirements

Right Column - Two options:
Option 1: Digital Twin Setup
- RTX 4070+ GPU
- 64GB RAM
- Ubuntu 22.04

Option 2: Physical Lab
- Jetson Orin Nano
- Intel RealSense Camera
- Unitree Robot (optional)

CTA: "View Full Requirements →" (links to hardware-requirements page)

### 6. Capstone Project Preview
Visual Design:
- Full-width section with video/gif background
- Overlay with semi-transparent background
- Center-aligned content

Content:
- Title: "Your Capstone: The Autonomous Humanoid"
- Description: "Build a simulated humanoid that receives voice commands, plans paths, navigates obstacles, and manipulates objects using computer vision."
- Features list:
  - ✓ Voice command processing with Whisper
  - ✓ LLM-powered action planning
  - ✓ SLAM navigation
  - ✓ Object detection and manipulation
- CTA: "See Capstone Details →"

### 7. Call-to-Action Section
Visual Design:
- Bold gradient background
- Large centered content
- High contrast

Content:
- Headline: "Ready to Build the Future?"
- Subtext: "Start your journey into Physical AI and Humanoid Robotics today"
- Large button: "Begin Learning Now"
- Secondary link: "Download Syllabus"

### 8. Footer
Content:
- Course name and tagline
- Quick links (modules, hardware, capstone)
- GitHub repository link
- Copyright notice

## File Structure

Create/Modify these files:
```
src/
├── pages/
│   └── index.tsx              # Main homepage component
├── css/
│   ├── custom.css             # Global styles (update)
│   └── homepage.module.css    # Homepage-specific styles
└── components/
    └── HomepageFeatures/
        ├── index.tsx          # Feature cards component
        └── styles.module.css  # Feature styles
```

## Technical Implementation Requirements

### Typography
- Headings: Use font weights 700-900
- Main headline: 3.5-4rem (mobile: 2.5rem)
- Section titles: 2.5-3rem (mobile: 2rem)
- Body text: 1.1-1.2rem
- Use CSS clamp() for fluid typography

### Colors
Primary Gradient: 
- Start: #667eea (purple)
- End: #764ba2 (deeper purple)

Accent Colors:
- Blue: #4F46E5
- Purple: #7C3AED
- Cyan: #06B6D4

Background:
- Light mode: #ffffff, #f9fafb
- Dark mode: #1a1a1a, #2d2d2d

### Animations
Use CSS animations and React hooks:
- Fade in on scroll (intersection observer)
- Hero text: slide up + fade in (0.8s ease-out)
- Cards: scale + lift on hover
- Gradient backgrounds: subtle movement
- Smooth scroll behavior

### Responsive Breakpoints
- Mobile: < 768px
- Tablet: 768px - 1024px
- Desktop: > 1024px

Use Tailwind-style approach or CSS Grid/Flexbox

### Performance
- Optimize images (WebP format, lazy loading)
- Code splitting for homepage
- Minimize animations on mobile
- Use CSS transforms (GPU accelerated)

### Accessibility
- Semantic HTML (header, main, section, nav)
- ARIA labels where needed
- Keyboard navigation support
- Sufficient color contrast (WCAG AA)
- Focus states on interactive elements

## Code Style

### React/TypeScript
- Use functional components with hooks
- TypeScript for type safety
- Proper prop typing
- Clean component structure

### CSS
- Use CSS Modules for scoped styles
- BEM-like naming for clarity
- Mobile-first media queries
- CSS custom properties for theming

## Integration with Existing Docusaurus

- Keep Docusaurus navbar and footer
- Ensure dark mode toggle works
- Maintain consistency with docs styling
- Add custom homepage to src/pages/index.tsx
- Don't break existing documentation pages

## Inspiration References
- Modern SaaS landing pages
- Tech education platforms (Frontend Masters, Egghead)
- AI/ML product pages (OpenAI, Anthropic, Hugging Face)
- Robotics company sites (Boston Dynamics, Unitree)

## Success Criteria
- [ ] Homepage loads in < 2 seconds
- [ ] All sections visible and functional
- [ ] Smooth animations without jank
- [ ] Mobile responsive (looks great on phone)
- [ ] Dark mode works perfectly
- [ ] All links navigate correctly
- [ ] Professional, modern appearance
- [ ] Draws attention and encourages exploration