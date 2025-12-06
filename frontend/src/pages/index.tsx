import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import HomepageHeader from '../components/HomepageHeader';
import HomepageFeatures from '../components/HomepageFeatures';
import ChapterCards from '../components/ChapterCards';
import LearningPath from '../components/LearningPath';
import AboutSection from '../components/AboutSection';
import TechnologiesGrid from '../components/TechnologiesGrid';
import CTABanner from '../components/CTABanner';

import styles from './index.module.css';

export default function Home(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="AI-Native Textbook on Physical AI & Humanoid Robotics - A comprehensive guide combining theory with hands-on implementation"
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <ChapterCards />
        <LearningPath />
        <AboutSection />
        <TechnologiesGrid />
        <CTABanner />
      </main>
    </Layout>
  );
}