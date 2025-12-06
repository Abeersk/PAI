import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';

import styles from './styles.module.css';

type StatItem = {
  label: string;
  value: string;
  icon: string;
};

const StatsList: StatItem[] = [
  {
    label: '10 Chapters',
    value: 'Complete curriculum covering fundamentals to advanced topics',
    icon: '📚',
  },
  {
    label: '20+ Code Examples',
    value: 'Working Python implementations with PyBullet',
    icon: '💻',
  },
  {
    label: '35+ Figures',
    value: 'Professional diagrams and visualizations',
    icon: '📊',
  },
  {
    label: '30+ Citations',
    value: 'Peer-reviewed sources and academic rigor',
    icon: '✅',
  },
];

export default function AboutSection(): React.JSX.Element {
  return (
    <section className={styles.about}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <Heading as="h2" className={styles.aboutTitle}>
              Built for the AI-Native Era
            </Heading>
            <p className={styles.aboutDescription}>
              This open-source textbook bridges the gap between classical robotics and modern AI. Created for students, researchers, and hackathon participants who want to understand both theory and practice.
            </p>
            <p className={styles.aboutDescription}>
              Every concept is backed by working code, validated formulas, and real-world context. From DH parameters to reinforcement learning, you'll gain hands-on experience with the tools shaping the future of robotics.
            </p>
            <div className={styles.statsGrid}>
              {StatsList.map((stat, index) => (
                <div key={index} className={styles.statItem}>
                  <div className={styles.statIcon}>{stat.icon}</div>
                  <div className={styles.statContent}>
                    <div className={styles.statLabel}>{stat.label}</div>
                    <div className={styles.statValue}>{stat.value}</div>
                  </div>
                </div>
              ))}
            </div>
          </div>
          <div className="col col--6">
            <div className={styles.illustration}>
              <svg viewBox="0 0 400 300" className={styles.robotIllustration}>
                <rect x="50" y="50" width="300" height="200" rx="20" fill="none" stroke="currentColor" strokeWidth="2" />
                <circle cx="100" cy="100" r="20" fill="none" stroke="currentColor" strokeWidth="2" />
                <circle cx="300" cy="100" r="20" fill="none" stroke="currentColor" strokeWidth="2" />
                <rect x="150" y="130" width="100" height="40" rx="10" fill="none" stroke="currentColor" strokeWidth="2" />
                <line x1="100" y1="150" x2="100" y2="200" stroke="currentColor" strokeWidth="3" />
                <line x1="300" y1="150" x2="300" y2="200" stroke="currentColor" strokeWidth="3" />
                <line x1="80" y1="200" x2="120" y2="230" stroke="currentColor" strokeWidth="3" />
                <line x1="280" y1="200" x2="320" y2="230" stroke="currentColor" strokeWidth="3" />
                <circle cx="200" cy="80" r="15" fill="currentColor" opacity="0.3" />
                <circle cx="150" cy="60" r="10" fill="currentColor" opacity="0.2" />
                <circle cx="250" cy="60" r="10" fill="currentColor" opacity="0.2" />
              </svg>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}