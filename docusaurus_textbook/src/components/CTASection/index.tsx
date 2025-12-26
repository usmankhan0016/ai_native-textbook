import Link from "@docusaurus/Link";
import Heading from "@theme/Heading";
import styles from "./styles.module.css";

export default function CTASection(): JSX.Element {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2" className={styles.ctaHeading}>
            Begin Your Robotics Journey
          </Heading>
          <p className={styles.ctaSubtitle}>
            The future belongs to physical AI, embodied intelligence, and humanoid robotics. Start mastering it today.
          </p>
          <Link
            className={styles.ctaButton}
            to="/docs/intro"
          >
            Start Reading →
          </Link>
        </div>
      </div>
    </section>
  );
}
