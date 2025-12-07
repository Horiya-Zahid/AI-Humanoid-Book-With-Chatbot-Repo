<!-- Sync Impact Report:
Version change: None (Initial Creation)
Modified principles: None
Added sections: Core Mission, Governing Principles, Content Architecture, Bonus Features, Acceptance Criteria
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/sp.constitution.md: ✅ updated
  - .specify/templates/commands/sp.phr.md: ⚠ pending
Follow-up TODOs: None
-->
# Project1 – AI-Native Book Creation with Spec-Kit Plus & Claude Code Constitution

## Core Mission
Build a complete, high-quality, AI-native book using only Docusaurus 3.x as the static site generator, written entirely with the help of Spec-Kit Plus and Claude Code, and deployed to GitHub Pages. The book must be fully driven by specifications, plans, and tasks generated through the Spec-Kit Plus workflow. The final product is a polished, professional, mobile-responsive, accessible book that serves as a real-world demonstration of AI-assisted spec-driven authorship.

## Core Principles

### I. Spec-Kit Plus First
Every single piece of content, code, and decision MUST originate from a .specify workflow (constitution → specs → plans → tasks). No manual writing of markdown files outside the Spec-Kit Plus agent loop. All generated files must be committed via the Spec-Kit Plus memory system.

### II. Docusaurus 3.x Immutability
Framework: Docusaurus 3.x only (latest stable version). Deployment target: GitHub Pages only. No Next.js, Astro, Gatsby, or any other SSG allowed. Classic preset with blog + docs + pages enabled.

### III. AI-Assisted but Human-Owned
Primary authoring tools: Spec-Kit Plus + Claude Code (Anthropic). Human author retains final approval on every commit. Every chapter must show clear evidence of spec → plan → task traceability.

### IV. Aesthetic & Branding
Clean, modern, professional look (default Docusaurus “Classic” theme is acceptable). Primary color scheme: Author’s choice (you may propose one, but keep it consistent). Mobile-first responsive design mandatory. WCAG 2.1 AA compliance for readability and navigation.

### V. Content Excellence
Progressive learning path from beginner to advanced. Every technical concept includes at least one practical example. Rich visuals: diagrams, screenshots, architecture drawings (generated via Mermaid, Excalidraw, or Claude artifacts). Each chapter ends with “Key Takeaways” and “Further Reading”.

### VI. Code Quality & Documentation
All custom code (if any plugins/scripts): TypeScript with strict mode. Markdown: Consistent MDX usage, proper frontmatter, readable headings. Full Open Graph + SEO metadata on every page. Automated table of contents and search (Docusaurus built-in).

### VII. Performance & Correctness
Lighthouse score ≥ 95 on mobile for performance, accessibility, best practices, SEO. Full build time under 4 minutes on GitHub Actions. Zero broken links (checked via Docusaurus link checker).

## Content Architecture
Book Title: “Learn [Your Topic] with AI-Native Spec-Driven Development”
Suggested chapters:
1. Introduction to AI-Native Writing
2. Why Spec-Driven Development Wins
3. Setting Up Spec-Kit Plus
4. Writing Your First Specification
5. From Spec to Plan to Tasks
6. Generating Chapters with Claude Code
7. Customizing Docusaurus Theme
8. Adding Diagrams and Interactive Content
9. Testing and Quality Assurance
10. Deployment to GitHub Pages
11. Maintaining and Versioning Your Book
12. Conclusion & Next Steps

## Bonus Features
- Dark/Light mode toggle with persistent preference (50 pts)
- Built-in search with Algolia DocSearch (50 pts)
- PDF export button per chapter (50 pts)
- Urdu/Hindi dual-language support with RTL (100 pts)
- Progress tracking + localStorage checklist (50 pts)

## Acceptance Criteria – Base Book
- Complete Docusaurus 3.x site with custom domain or github.io URL
- At least 8 fully written chapters following the spec-driven workflow
- All content generated and tracked via Spec-Kit Plus memory
- Mobile-responsive, accessible, fast-loading
- Deployed and publicly accessible on GitHub Pages
- README with clear build and contribution instructions
- Evidence of full spec → plan → task chain in .specify/memory/

## Governance
This Constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews MUST verify compliance. Complexity must be justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06

---
## Reflection and Next Prompts

### Reflection
The constitution has been successfully generated, outlining the core mission, governing principles, content architecture, bonus features, and acceptance criteria for the AI-Native Book Creation project. It also includes the governance rules for amendments.

### Next Prompts
The next step is to create the Prompt History Record (PHR) for this action, and then proceed with setting up the project based on the constitution, likely starting with the initial Docusaurus setup and generating the first specifications.
