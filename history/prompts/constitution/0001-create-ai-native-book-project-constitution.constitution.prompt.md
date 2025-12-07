---
id: 0001
title: Create AI-Native Book Project Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: [project-setup, constitution]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

Create the Project Constitution for my new book project using Spec-Kit Plus.

Project Name: Project1 – AI-Native Book Creation with Spec-Kit Plus & Claude Code
Repository will be deployed with Docusaurus + GitHub Pages

Core Mission:
Build a complete, high-quality, AI-native book using only Docusaurus 3.x as the static site generator, written entirely with the help of Spec-Kit Plus and Claude Code, and deployed to GitHub Pages. The book must be fully driven by specifications, plans, and tasks generated through the Spec-Kit Plus workflow. The final product is a polished, professional, mobile-responsive, accessible book that serves as a real-world demonstration of AI-assisted spec-driven authorship.

Governing Principles (these are non-negotiable):

1. Spec-Kit Plus First
   - Every single piece of content, code, and decision MUST originate from a .specify workflow (constitution → specs → plans → tasks)
   - No manual writing of markdown files outside the Spec-Kit Plus agent loop
   - All generated files must be committed via the Spec-Kit Plus memory system

2. Docusaurus 3.x Immutability
   - Framework: Docusaurus 3.x only (latest stable version)
   - Deployment target: GitHub Pages only
   - No Next.js, Astro, Gatsby, or any other SSG allowed
   - Classic preset with blog + docs + pages enabled

3. AI-Assisted but Human-Owned
   - Primary authoring tools: Spec-Kit Plus + Claude Code (Anthropic)
   - Human author retains final approval on every commit
   - Every chapter must show clear evidence of spec → plan → task traceability

4. Aesthetic & Branding
   - Clean, modern, professional look (default Docusaurus “Classic” theme is acceptable)
   - Primary color scheme: Author’s choice (you may propose one, but keep it consistent)
   - Mobile-first responsive design mandatory
   - WCAG 2.1 AA compliance for readability and navigation

5. Content Excellence
   - Progressive learning path from beginner to advanced
   - Every technical concept includes at least one practical example
   - Rich visuals: diagrams, screenshots, architecture drawings (generated via Mermaid, Excalidraw, or Claude artifacts)
   - Each chapter ends with “Key Takeaways” and “Further Reading”

6. Code Quality & Documentation
   - All custom code (if any plugins/scripts): TypeScript with strict mode
   - Markdown: Consistent MDX usage, proper frontmatter, readable headings
   - Full Open Graph + SEO metadata on every page
   - Automated table of contents and search (Docusaurus built-in)

7. Performance & Correctness
   - Lighthouse score ≥ 95 on mobile for performance, accessibility, best practices, SEO
   - Full build time under 4 minutes on GitHub Actions
   - Zero broken links (checked via Docusaurus link checker)

Content Architecture (example – replace with your actual book topic):
Book Title: “Learn [Your Topic] with AI-Native Spec-Driven Development”
Suggested chapters (you can change):
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

Bonus Features (optional but scored):
- Dark/Light mode toggle with persistent preference (50 pts)
- Built-in search with Algolia DocSearch (50 pts)
- PDF export button per chapter (50 pts)
- Urdu/Hindi dual-language support with RTL (100 pts)
- Progress tracking + localStorage checklist (50 pts)

Acceptance Criteria – Base Book (150 points):
- Complete Docusaurus 3.x site with custom domain or github.io URL
- At least 8 fully written chapters following the spec-driven workflow
- All content generated and tracked via Spec-Kit Plus memory
- Mobile-responsive, accessible, fast-loading
- Deployed and publicly accessible on GitHub Pages
- README with clear build and contribution instructions
- Evidence of full spec → plan → task chain in .specify/memory/

Bonus Total: up to 300 additional points possible

Output Format:
Write the constitution in clean markdown and save it as .specify/memory/constitution.md
version 1.0.0
Include a Sync Impact Report at the top
End with the standard Reflection and Next prompts section exactly as Spec-Kit Plus expects.

Now generate the complete Project Constitution.

## Response snapshot

The Project Constitution for "Project1 – AI-Native Book Creation with Spec-Kit Plus & Claude Code" has been generated and saved to .specify/memory/constitution.md. It includes the core mission, seven governing principles, content architecture, bonus features, acceptance criteria, and governance rules. The constitution is version 1.0.0, ratified and last amended on 2025-12-06.

## Outcome

- ✅ Impact: Constitution for the AI-Native Book Project successfully created and saved.
- 🧪 Tests: No explicit tests were run for constitution generation.
- 📁 Files: .specify/memory/constitution.md created/modified.
- 🔁 Next prompts: Set up Docusaurus project; generate initial specifications.
- 🧠 Reflection: The constitution has been successfully generated, outlining the core mission, governing principles, content architecture, bonus features, and acceptance criteria for the AI-Native Book Creation project. It also includes the governance rules for amendments.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
