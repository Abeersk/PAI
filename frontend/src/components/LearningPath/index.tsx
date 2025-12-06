import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';

import styles from './styles.module.css';

type PhaseItem = {
  id: number;
  title: string;
  icon: string;
  duration: string;
  chapters: string;
  outcome: string;
  progress: number;
};

const PhaseList: PhaseItem[] = [
  {
    id: 1,
    title: 'Phase 1: Foundations',
    icon: '📚',
    duration: '2-3 weeks',
    chapters: 'Introduction, Safety, Morphology',
    outcome: 'Understand Physical AI principles, safety standards, and humanoid design',
    progress: 30,
  },
  {
    id: 2,
    title: 'Phase 2: Core Systems',
    icon: '⚙️',
    duration: '4-5 weeks',
    chapters: 'Kinematics, Actuation, Sensing, Control',
    outcome: 'Implement complete perception and control pipelines',
    progress: 40,
  },
  {
    id: 3,
    title: 'Phase 3: Integration',
    icon: '🤖',
    duration: '2-3 weeks',
    chapters: 'Locomotion, AI Integration, Sim-to-Real',
    outcome: 'Build autonomous humanoid control systems',
    progress: 30,
  },
];

function PhaseCard({ title, icon, duration, chapters, outcome, progress }: PhaseItem) {
  return (
    <div className={clsx('col col--4', styles.phaseCard)}>
      <div className={styles.phaseCardContent}>
        <div className={styles.phaseHeader}>
          <div className={styles.iconContainer}>
            <span className={styles.icon}>{icon}</span>
          </div>
          <div className={styles.phaseInfo}>
            <h3 className={styles.phaseTitle}>{title}</h3>
            <div className={styles.duration}>{duration}</div>
          </div>
        </div>
        <div className={styles.phaseDetails}>
          <div className={styles.chaptersList}>Chapters: {chapters}</div>
          <div className={styles.outcome}>Outcome: {outcome}</div>
        </div>
        <div className={styles.progressContainer}>
          <div className={styles.progressLabel}>Progress: {progress}%</div>
          <div className={styles.progressBar}>
            <div
              className={styles.progressFill}
              style={{ width: `${progress}%` }}
            ></div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function LearningPath(): React.JSX.Element {
  return (
    <section className={styles.learningPath}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('text--center', styles.learningPathTitle)}>
              Your Journey to Mastery
            </Heading>
            <p className={clsx('text--center', styles.learningPathSubtitle)}>
              A structured path from fundamentals to advanced integration
            </p>
          </div>
        </div>
        <div className="row">
          {PhaseList.map((props) => (
            <PhaseCard key={props.id} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}