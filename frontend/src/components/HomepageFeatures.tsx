import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './HomepageFeatures.module.css';

type FeatureItem = {
  title: string;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Hands-on Code Examples',
    description: (
      <>
        Practical implementations with Python, NumPy, and robotics simulation frameworks
        to reinforce theoretical concepts with real-world applications.
      </>
    ),
  },
  {
    title: 'Simulation Tutorials',
    description: (
      <>
        Step-by-step guides for Gazebo, Isaac Sim, and other simulation environments
        to practice robotics concepts in safe, controlled settings.
      </>
    ),
  },
  {
    title: 'Exercises & Labs',
    description: (
      <>
        Comprehensive exercises and laboratory assignments designed to deepen understanding
        of Physical AI and humanoid robotics principles.
      </>
    ),
  },
];

function Feature({ title, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <div className={styles.featureIcon}>
          <svg className={styles.icon} viewBox="0 0 24 24" width="64" height="64">
            <path fill="currentColor" d="M12 2L2 7l10 5 10-5-10-5zM2 17l10 5 10-5M2 12l10 5 10-5" />
          </svg>
        </div>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
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