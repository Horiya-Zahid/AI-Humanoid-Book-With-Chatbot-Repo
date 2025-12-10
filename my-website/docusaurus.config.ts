import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  // tagline: 'Comprehensive textbook on Physical AI, ROS 2, Digital Twins, Isaac Platform, and Vision-Language-Action Systems',
  favicon: 'img/favicon.ico',

  future: { v4: true },

  url: 'https://your-physical-ai-book-site.example.com',
  baseUrl: '/',

  organizationName: 'your-org',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/your-org/physical-ai-textbook',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },

    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Textbook',
        },

        // CHAT BUTTON IN NAVBAR (100% safe, no crash)
        {
          type: 'html',
          position: 'right',
          value: `
            <button
              onclick="window.openRagChat && window.openRagChat()"
              style="background:none;border:none;color:#39ff14;font-weight:600;font-size:1rem;cursor:pointer;padding:0.5rem 1rem;"
            >
              Chat
            </button>
          `,
        },

        // GitHub link
        {
          href: 'https://github.com/your-org/physical-ai-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            { label: 'Introduction', to: '/docs/intro' },
            { label: 'Module 1: Robotic Nervous System', to: '/docs/module-1-robotic-nervous-system/week-1' },
            { label: 'Module 2: Digital Twin', to: '/docs/module-2-digital-twin/week-6' },
            { label: 'Module 3: AI-Robot Brain', to: '/docs/module-3-isaac-brain/week-8' },
            { label: 'Module 4: VLA Capstone', to: '/docs/module-4-vla-capstone/week-11' },
          ],
        },
        {
          title: 'Resources',
          items: [
            { label: 'ROS 2 Documentation', href: 'https://docs.ros.org/en/humble/' },
            { label: 'NVIDIA Isaac', href: 'https://developer.nvidia.com/isaac-ros-gems' },
            { label: 'Docusaurus', href: 'https://docusaurus.io/' },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'GitHub', href: 'https://github.com/your-org/physical-ai-textbook' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },

    prism: {
      theme: prismThemes.oneLight,
      darkTheme: prismThemes.oneDark,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;