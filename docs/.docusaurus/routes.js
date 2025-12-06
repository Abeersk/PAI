import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'cd6'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '781'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '706'),
            routes: [
              {
                path: '/docs/chapter-1-introduction',
                component: ComponentCreator('/docs/chapter-1-introduction', 'dca'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/chapter-10-sim-to-real',
                component: ComponentCreator('/docs/chapter-10-sim-to-real', '5c4'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/chapter-2-safety',
                component: ComponentCreator('/docs/chapter-2-safety', 'd0b'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/chapter-3-morphology',
                component: ComponentCreator('/docs/chapter-3-morphology', '5f8'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/chapter-4-kinematics',
                component: ComponentCreator('/docs/chapter-4-kinematics', '5bf'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/chapter-5-actuation',
                component: ComponentCreator('/docs/chapter-5-actuation', 'b5c'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/chapter-6-sensing',
                component: ComponentCreator('/docs/chapter-6-sensing', '0f0'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/chapter-7-control',
                component: ComponentCreator('/docs/chapter-7-control', '476'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/chapter-8-locomotion',
                component: ComponentCreator('/docs/chapter-8-locomotion', 'c39'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/chapter-9-ai-integration',
                component: ComponentCreator('/docs/chapter-9-ai-integration', '625'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '362'),
                exact: true,
                sidebar: "textbookSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
