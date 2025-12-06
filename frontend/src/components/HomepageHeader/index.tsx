import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Heading from '@theme/Heading';

import styles from './styles.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className={clsx('hero__title', styles.heroTitle)}>
              {siteConfig.title}
            </Heading>
            <p className={clsx('hero__subtitle', styles.heroSubtitle)}>
              Master the Future of Embodied Intelligence
            </p>
            <p className={styles.heroDescription}>
              From Theory to Implementation: Kinematics, Control, AI, and Real-World Deployment
            </p>
            <div className={styles.statsBar}>
              <span>10 Chapters</span>
              <span>•</span>
              <span>90+ Pages</span>
              <span>•</span>
              <span>20+ Code Examples</span>
              <span>•</span>
              <span>35+ Figures</span>
            </div>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/chapter-1-introduction">
                Start Learning →
              </Link>
              <Link
                className="button button--outline button--lg"
                to="https://github.com/specify/physical-ai-textbook">
                View on GitHub
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            {/* Robot silhouette placeholder */}
            <div className={styles.robotVisual}>
              <svg viewBox="0 0 200 300" className={styles.robotSvg}>
                <path
                  d="M100,50 C120,40 140,40 160,50 C170,60 170,80 160,90 C140,110 120,120 100,120 C80,120 60,110 40,90 C30,80 30,60 40,50 C60,40 80,40 100,50 Z"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                />
                <circle cx="85" cy="70" r="5" fill="currentColor" />
                <circle cx="115" cy="70" r="5" fill="currentColor" />
                <rect x="75" y="90" width="50" height="8" rx="4" fill="currentColor" />
                <rect x="40" y="120" width="120" height="100" rx="10" fill="none" stroke="currentColor" strokeWidth="2" />
                <line x1="40" y1="150" x2="160" y2="150" stroke="currentColor" strokeWidth="1" />
                <line x1="40" y1="180" x2="160" y2="180" stroke="currentColor" strokeWidth="1" />
                <line x1="100" y1="220" x2="100" y2="280" stroke="currentColor" strokeWidth="4" />
                <line x1="70" y1="280" x2="130" y2="280" stroke="currentColor" strokeWidth="4" />
              </svg>
            </div>
          </div>
        </div>
        <div className={styles.scrollIndicator}>
          <div className={styles.chevron}></div>
          <div className={styles.chevron}></div>
          <div className={styles.chevron}></div>
        </div>
      </div>
    </header>
  );
}

export default HomepageHeader;