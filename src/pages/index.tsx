import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";
import Heading from "@theme/Heading";

import styles from "./index.module.css";

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx("hero", styles.heroBanner)}>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <Heading as="h1" className={styles.heroTitle}>
            <span className="gradient-text">Master Physical AI &</span>  Humanoid
            Robotics
          </Heading>
          <p className={styles.heroSubtitle}>
            Learn ROS 2, Digital Twin Simulation, NVIDIA Isaac, and
            Vision-Language-Action Models in this comprehensive, hands-on
            textbook.
          </p>
          <div className={styles.heroStats}>
            <div className={styles.stat}>
              <span className={styles.statNumber}>4</span>
              <span className={styles.statLabel}>Modules</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>40+</span>
              <span className={styles.statLabel}>Hours</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>∞</span>
              <span className={styles.statLabel}>Projects</span>
            </div>
          </div>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro"
            >
              🚀 Start Learning
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/module-1"
            >
              📘 View Modules
            </Link>
          </div>
        </div>
        <div className={styles.heroVisual}>
          <div className={styles.gradientOrb}></div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Learn Physical AI and Humanoid Robotics with ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action models"
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
