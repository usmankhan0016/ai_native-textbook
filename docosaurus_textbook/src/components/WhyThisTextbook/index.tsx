import Heading from "@theme/Heading";
import styles from "./styles.module.css";

type BenefitItem = {
  icon: string;
  title: string;
  description: string;
};

const BenefitsList: BenefitItem[] = [
  {
    icon: "🤖",
    title: "AI-Native Approach",
    description:
      "Built from the ground up for the era of embodied AI and humanoid robots, not just traditional robotics.",
  },
  {
    icon: "🔄",
    title: "Hands-On Learning",
    description:
      "40+ hours of practical, project-based learning with real code, simulations, and hardware integration.",
  },
  {
    icon: "🏢",
    title: "Industry-Ready Stack",
    description:
      "Master ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models used by leading robotics companies.",
  },
  {
    icon: "📚",
    title: "Comprehensive Curriculum",
    description:
      "From fundamentals to advanced AI integration—4 modules, 13 weeks, covering the full modern robotics stack.",
  },
  {
    icon: "⚡",
    title: "Future-Focused",
    description:
      "Designed for the next generation of robotics engineers who want to build intelligent, autonomous systems.",
  },
  {
    icon: "🎯",
    title: "Capstone Projects",
    description:
      "Build end-to-end robotics applications combining vision, language understanding, and autonomous control.",
  },
];

function BenefitItem({ icon, title, description }: BenefitItem) {
  return (
    <div className={styles.benefitCard}>
      <div className={styles.benefitIcon}>{icon}</div>
      <Heading as="h3" className={styles.benefitTitle}>
        {title}
      </Heading>
      <p className={styles.benefitDescription}>{description}</p>
    </div>
  );
}

export default function WhyThisTextbook(): JSX.Element {
  return (
    <section className={styles.whySection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Why This Textbook is Different
          </Heading>
          <p className={styles.sectionSubtitle}>
            An AI-Native curriculum designed for engineers building the future of humanoid robotics
          </p>
        </div>
        <div className={styles.benefitsGrid}>
          {BenefitsList.map((benefit, idx) => (
            <BenefitItem key={idx} {...benefit} />
          ))}
        </div>
      </div>
    </section>
  );
}
