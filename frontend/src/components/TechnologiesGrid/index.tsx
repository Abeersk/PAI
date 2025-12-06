import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';

import styles from './styles.module.css';

type TechItem = {
  name: string;
  description: string;
};

const TechList: TechItem[] = [
  {
    name: 'Python',
    description: 'Primary programming language',
  },
  {
    name: 'NumPy/SciPy',
    description: 'Scientific computing and linear algebra',
  },
  {
    name: 'PyBullet',
    description: 'Physics simulation and testing',
  },
  {
    name: 'ROS2',
    description: 'Robot Operating System',
  },
  {
    name: 'Matplotlib',
    description: 'Data visualization and plotting',
  },
  {
    name: 'OpenCV',
    description: 'Computer vision and image processing',
  },
  {
    name: 'Jupyter',
    description: 'Interactive notebooks and tutorials',
  },
  {
    name: 'Git/GitHub',
    description: 'Version control and collaboration',
  },
];

export default function TechnologiesGrid(): React.JSX.Element {
  return (
    <section className={styles.technologies}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('text--center', styles.technologiesTitle)}>
              Built With Modern Tools
            </Heading>
            <p className={clsx('text--center', styles.technologiesSubtitle)}>
              Industry-standard technologies for learning and implementation
            </p>
          </div>
        </div>
        <div className="row">
          {TechList.map((tech, index) => (
            <div key={index} className={clsx('col col--3', styles.techItem)}>
              <div className={styles.techCardContent}>
                <div className={styles.techName}>{tech.name}</div>
                <div className={styles.techDescription}>{tech.description}</div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}