import clsx from "clsx";
import Heading from "@theme/Heading";
import styles from "./styles.module.css";

type FeatureItem = {
  title: string;
  icon: string;
  description: JSX.Element;
  link: string;
  highlights: string[];
};

const FeatureList: FeatureItem[] = [
  {
    title: "Module 1: ROS 2 Fundamentals",
    icon: "⚙️",
    link: "/docs/module-1",
    highlights: ["Nodes & Topics", "Services & Actions", "URDF for Humanoids"],
    description: (
      <>
        Master the robotic nervous system with ROS 2 Humble. Build the
        foundation for intelligent robot control and communication.
      </>
    ),
  },
  {
    title: "Module 2: Digital Twin Simulation",
    icon: "🎮",
    link: "/docs/module-2",
    highlights: ["Gazebo Physics", "Unity Rendering", "Sensor Simulation"],
    description: (
      <>
        Create realistic digital twins with Gazebo and Unity. Test robots in
        virtual environments before hardware deployment.
      </>
    ),
  },
  {
    title: "Module 3: NVIDIA Isaac Platform",
    icon: "🟩",
    link: "/docs/module-3",
    highlights: ["Isaac Sim", "Synthetic Data", "Isaac ROS & Lab"],
    description: (
      <>
        Leverage NVIDIA's ecosystem for photorealistic simulation, synthetic
        data generation, and hardware-accelerated robotics.
      </>
    ),
  },
  {
    title: "Module 4: Vision-Language-Action",
    icon: "🧠",
    link: "/docs/module-4",
    highlights: ["Computer Vision", "LLM Planning", "VLA Control"],
    description: (
      <>
        Integrate AI models for vision, language understanding, and action
        planning. Build truly intelligent humanoid robots.
      </>
    ),
  },
];

function Feature({ title, icon, description, link, highlights }: FeatureItem) {
  return (
    <div className={clsx("col col--3")}>
      <a href={link} className={clsx(styles.featureCard, "glass-card")}>
        <div className={styles.featureIcon}>{icon}</div>
        <Heading as="h3" className={styles.featureTitle}>
          {title}
        </Heading>
        <p className={styles.featureDescription}>{description}</p>
        <ul className={styles.featureHighlights}>
          {highlights.map((item, idx) => (
            <li key={idx}>• {item}</li>
          ))}
        </ul>
        <div className={styles.featureLink}>Learn More →</div>
      </a>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Complete Learning Path
          </Heading>
          <p className={styles.sectionSubtitle}>
            Four comprehensive modules covering the full stack of modern
            humanoid robotics
          </p>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
