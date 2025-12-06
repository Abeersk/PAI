import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';

import styles from './styles.module.css';

type ChapterItem = {
  id: number;
  title: string;
  description: string;
  topics: string[];
  badge: string;
  path: string;
};

const ChapterList: ChapterItem[] = [
  {
    id: 1,
    title: 'Chapter 1: Introduction to Physical AI',
    description: 'Discover what makes Physical AI different from traditional robotics. Learn about embodied intelligence, the sense-think-act loop, and why humanoid robots are the future.',
    topics: ['Embodied Intelligence', 'Sense-Think-Act', 'Sim-to-Real'],
    badge: 'START HERE',
    path: '/docs/chapter-1-introduction',
  },
  {
    id: 4,
    title: 'Chapter 4: Kinematics & Dynamics',
    description: 'Master the mathematical foundations of robot motion. From DH parameters to Jacobian matrices, understand how robots move and interact with their environment.',
    topics: ['DH Parameters', 'Forward/Inverse Kinematics', 'Jacobians', 'Dynamics'],
    badge: 'CORE THEORY',
    path: '/docs/chapter-4-kinematics',
  },
  {
    id: 7,
    title: 'Chapter 7: Control Systems',
    description: 'Implement real-time control algorithms. Design PID controllers, state-space systems, and trajectory planners that make robots move smoothly and precisely.',
    topics: ['PID Control', 'LQR', 'Trajectory Planning', 'Real-Time Systems'],
    badge: 'PRACTICAL',
    path: '/docs/chapter-7-control',
  },
  {
    id: 8,
    title: 'Chapter 8: Locomotion & Gait',
    description: 'Unlock the secrets of bipedal walking. Learn ZMP stability, gait generation, and how to make humanoid robots walk, run, and balance dynamically.',
    topics: ['ZMP Stability', 'Bipedal Walking', 'Central Pattern Generators', 'Balance Control'],
    badge: 'ADVANCED',
    path: '/docs/chapter-8-locomotion',
  },
  {
    id: 9,
    title: 'Chapter 9: AI Integration',
    description: 'Integrate modern AI with classical robotics. Explore reinforcement learning for locomotion, path planning algorithms, and behavior trees for decision-making.',
    topics: ['Reinforcement Learning', 'RRT Path Planning', 'Behavior Trees', 'AI Planning'],
    badge: 'CUTTING-EDGE',
    path: '/docs/chapter-9-ai-integration',
  },
];

function ChapterCard({ title, description, topics, badge, path }: ChapterItem) {
  return (
    <div className={clsx('col col--4', styles.chapterCard)}>
      <div className={styles.chapterCardContent}>
        <div className={styles.badge}>{badge}</div>
        <h3 className={styles.chapterTitle}>{title}</h3>
        <p className={styles.chapterDescription}>{description}</p>
        <div className={styles.topics}>
          {topics.map((topic, index) => (
            <span key={index} className={styles.topic}>
              {topic}
            </span>
          ))}
        </div>
        <Link to={path} className={styles.chapterLink}>
          Read Chapter →
        </Link>
      </div>
    </div>
  );
}

export default function ChapterCards(): React.JSX.Element {
  return (
    <section className={styles.chapters}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('text--center', styles.chaptersTitle)}>
              Explore the Curriculum
            </Heading>
            <p className={clsx('text--center', styles.chaptersSubtitle)}>
              10 comprehensive chapters from fundamentals to advanced integration
            </p>
          </div>
        </div>
        <div className="row">
          {ChapterList.map((props) => (
            <ChapterCard key={props.id} {...props} />
          ))}
        </div>
        <div className="row">
          <div className="col col--12">
            <div className={styles.viewAllButton}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                View All Chapters
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}