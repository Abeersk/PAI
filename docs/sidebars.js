// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      items: [
        'intro',
        'chapter-1-introduction',
        'chapter-2-safety',
        'chapter-3-morphology',
        'chapter-4-kinematics',
        'chapter-5-actuation',
        'chapter-6-sensing',
        'chapter-7-control',
        'chapter-8-locomotion',
        'chapter-9-ai-integration',
        'chapter-10-sim-to-real',
      ],
    },
  ],
};

module.exports = sidebars;