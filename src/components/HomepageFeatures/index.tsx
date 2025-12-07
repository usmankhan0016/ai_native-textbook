import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Module 1: ROS 2 Fundamentals',
    description: (
      <>
        Master the robotic nervous system with ROS 2 Humble. Learn nodes, topics,
        services, actions, and URDF for humanoid robots.
      </>
    ),
  },
  {
    title: 'Module 2: Digital Twin Simulation',
    description: (
      <>
        Build realistic simulations with Gazebo and Unity. Create digital twins
        for your humanoid robots with physics and sensors.
      </>
    ),
  },
  {
    title: 'Module 3 & 4: Advanced AI Robotics',
    description: (
      <>
        Integrate NVIDIA Isaac for synthetic data and Vision-Language-Action models
        for intelligent robot control.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
