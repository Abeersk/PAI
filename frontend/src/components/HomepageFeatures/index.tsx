import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';

import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: string;
  icon: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Hands-On Implementation',
    description: '15+ working Python examples with PyBullet simulations. Copy, run, and understand real robotics code.',
    icon: '</>',
  },
  {
    title: 'Academic Rigor',
    description: '30+ peer-reviewed sources, APA 7 citations. Formulas validated with numerical examples.',
    icon: '🎓',
  },
  {
    title: 'End-to-End Pipeline',
    description: 'Complete robotics stack: Kinematics → Dynamics → Control → AI → Sim-to-Real Transfer',
    icon: '🔄',
  },
  {
    title: 'Hackathon-Ready',
    description: 'Quick-reference cheat sheets. Build a bipedal balancing robot in 48 hours.',
    icon: '🚀',
  },
];

function Feature({ title, description, icon }: FeatureItem) {
  return (
    <div className={clsx('col col--3', styles.feature)}>
      <div className={styles.featureCard}>
        <div className={styles.iconContainer}>
          <span className={styles.icon}>{icon}</span>
        </div>
        <h3 className={styles.featureTitle}>{title}</h3>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): React.JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('text--center', styles.featuresTitle)}>
              What Makes This Textbook Different
            </Heading>
          </div>
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