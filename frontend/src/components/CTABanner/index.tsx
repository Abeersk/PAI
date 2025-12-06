import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';

import styles from './styles.module.css';

export default function CTABanner(): React.JSX.Element {
  return (
    <section className={styles.ctaBanner}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('text--center', styles.ctaTitle)}>
              Ready to Build the Future?
            </Heading>
            <p className={clsx('text--center', styles.ctaSubtitle)}>
              Start your journey into Physical AI and humanoid robotics today.
            </p>
            <div className={styles.buttonGroup}>
              <Link
                className="button button--primary button--lg"
                to="/docs/chapter-1-introduction">
                Get Started
              </Link>
              <Link
                className="button button--outline button--lg"
                to="https://github.com/specify/physical-ai-textbook">
                View on GitHub
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}