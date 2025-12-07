# Implementation Plan: Module 4 – Vision-Language-Action (VLA) & Capstone

**Branch**: `004-module-4-vla` | **Date**: 2025-12-06 | **Spec**: [specs/004-module-4-vla-capstone/spec.md](specs/004-module-4-vla-capstone/spec.md)
**Input**: Feature specification from `/specs/004-module-4-vla-capstone/spec.md`

## Summary

Implementation of the culminating Vision-Language-Action (VLA) module that unites vision, language, and action components. Students build a simulated humanoid that accepts natural-language voice commands and executes them end-to-end using Whisper → LLM planner → ROS 2 actions. The module features a purple + neon theme, 100% embedded content, and strict RAG-only Q&A system with proper citations. Includes bonus features like live voice-to-action demo and Urdu translation.

## Technical Context

**Language/Version**: Python 3.11 (backend), JavaScript/TypeScript (frontend), Node.js 18+
**Primary Dependencies**: Docusaurus 3.x, FastAPI, OpenAI SDK, Google Generative AI SDK, Qdrant Client, ChatKit SDK, Better-Auth
**Storage**: Qdrant Cloud (vector DB), Neon Serverless Postgres (user data, sessions), GitHub Pages (static content)
**Testing**: pytest (backend), Jest/React Testing Library (frontend), contract tests (API)
**Target Platform**: Web application (desktop/mobile browsers), deployed to GitHub Pages (frontend) + Vercel/Fly.io (backend)
**Project Type**: Web application with static frontend and dynamic backend API
**Performance Goals**: <2.0s page load on 3G (Lighthouse ≥95), RAG response <3s (95th percentile), vector search <500ms, full static build <5 minutes
**Constraints**: Embedding generation batch max 100 chunks/request, WCAG 2.1 AA compliance, RAG-only responses (no hallucination), purple/neon theme consistency
**Scale/Scope**: Support 100 concurrent users, 4 modules with 13-week syllabus, 500-token chunks + 100-token overlap, ≥85% capstone project success rate

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Verification

1. **Spec-Kit Plus First**: ✅ All content originates from .specify workflow (constitution → specs → plans → tasks)
2. **Docusaurus 3.x Immutability**: ✅ Using Docusaurus 3.x with GitHub Pages deployment as required
3. **AI-Assisted but Human-Owned**: ✅ Using Spec-Kit Plus + Claude Code with human approval on commits
4. **Aesthetic & Branding**: ✅ Purple + neon theme implementation via Docusaurus CSS variables and swizzling
5. **Content Excellence**: ✅ Progressive learning path with practical examples and rich visuals
6. **Code Quality & Documentation**: ✅ TypeScript with strict mode, proper MDX usage, SEO metadata
7. **Performance & Correctness**: ✅ Targeting Lighthouse ≥95, build time <4 minutes, zero broken links

**Status**: All constitution principles PASS - Implementation plan complies with all requirements.

## Project Structure

### Documentation (this feature)

```text
specs/004-module-4-vla-capstone/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── chat-api.yaml
│   ├── selection-api.yaml
│   ├── embed-api.yaml
│   ├── auth-api.yaml
│   ├── personalize-api.yaml
│   └── translate-api.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── content_chunk.py
│   │   ├── user.py
│   │   ├── chat_session.py
│   │   └── personalization.py
│   ├── services/
│   │   ├── embedding_service.py
│   │   ├── rag_service.py
│   │   ├── auth_service.py
│   │   ├── translation_service.py
│   │   └── personalization_service.py
│   ├── api/
│   │   ├── routes/
│   │   │   ├── chat.py
│   │   │   ├── selection.py
│   │   │   ├── embed.py
│   │   │   ├── auth.py
│   │   │   ├── personalize.py
│   │   │   └── translate.py
│   │   └── main.py
│   ├── utils/
│   │   ├── validators.py
│   │   ├── security.py
│   │   └── helpers.py
│   └── config/
│       └── settings.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
└── requirements.txt

frontend/
├── src/
│   ├── components/
│   │   ├── ChatInterface/
│   │   ├── Theme/
│   │   ├── Accessibility/
│   │   └── VLA/
│   ├── pages/
│   │   ├── Module4/
│   │   ├── Capstone/
│   │   └── Auth/
│   ├── services/
│   │   ├── api.js
│   │   └── auth.js
│   ├── styles/
│   │   ├── custom.css
│   │   └── theme.css
│   └── utils/
│       └── helpers.js
├── docs/
│   ├── module-4-vla/
│   └── capstone-project/
├── static/
└── docusaurus.config.js

scripts/
├── generate_embeddings.py
├── setup_db.py
└── deploy.sh

.history/
├── prompts/
└── adr/
```

**Structure Decision**: Web application with separate backend (FastAPI) and frontend (Docusaurus) following the Option 2 structure. This separation allows for optimal performance with static content delivery via GitHub Pages and dynamic API operations via serverless backend.

## Architecture & Design

### System Architecture

The system follows a hybrid static + serverless architecture:

```
┌─────────────────┐    HTTP/API    ┌──────────────────┐
│   Frontend      │ ─────────────▶ │   Backend API    │
│  (GitHub Pages) │                │  (Vercel/Fly.io) │
│               ◀─┼────────────────┼─────────┐        │
│  Docusaurus 3.x │    SSE/HTTP    │ FastAPI │        │
└─────────────────┤                │ Python  │        │
                  │                └─────────┼────────┘
                  │                          │
                  │                ┌─────────▼────────┐
                  │                │  Qdrant Cloud    │
                  │                │  (Vector DB)     │
                  │                └──────────────────┘
                  │                          │
                  │                ┌─────────▼────────┐
                  │                │ Neon Postgres    │
                  │                │  (User Data)     │
                  └────────────────►                  │
                                   └──────────────────┘
```

### Key Architecture Decisions

1. **Hybrid Static + Serverless**: Static Docusaurus frontend on GitHub Pages with FastAPI backend on serverless platform (Vercel/Fly.io) for optimal performance and cost.

2. **Pre-generated Embeddings**: Embeddings generated at build time rather than runtime to ensure fast RAG responses and consistent content availability.

3. **RAG-only Enforcement**: System prevents hallucination through strict system prompts, temperature=0 settings, and citation validation middleware ensuring all responses cite actual textbook content.

4. **Text-Selection Query Handling**: Frontend captures selected text via `window.getSelection()` and sends to `/api/chat/selection` endpoint for contextual responses.

5. **Purple/Neon Theme Implementation**: Custom Docusaurus theme using CSS variables and component swizzling to achieve consistent #7048e8 → #a78bfa + #39ff14 accent styling.

6. **Chunking + Metadata Strategy**: Content stored in 500-token chunks with 100-token overlap, including module/week/section/path metadata in Qdrant payloads for precise citation.

### API Design

#### Core Endpoints
- `POST /api/chat` - Main RAG query endpoint with strict textbook-only responses
- `POST /api/chat/selection` - Text selection query handling with context
- `POST /api/embed` - Build-time only endpoint for generating content embeddings

#### Bonus Endpoints
- `POST /api/auth/*` - User authentication and management
- `POST /api/personalize` - Per-chapter personalization (complexity adjustment)
- `GET /api/translate/:chunkId/:lang` - Urdu translation with RTL support

### Data Flow

1. **Content Creation**: Textbook content written in Docusaurus MDX format
2. **Embedding Generation**: Build process chunks content (500-token + 100-token overlap) and generates OpenAI embeddings
3. **Storage**: Embeddings stored in Qdrant Cloud with metadata; user data in Neon Postgres
4. **Query Processing**: User query → vector similarity search → context retrieval → Gemini response generation → citation formatting
5. **Response Delivery**: RAG response with proper citations: "Source: Module X → Week Y → Section Z"

## Implementation Phases

### Phase 0: Research & Validation (Completed)
- Validated all 8 fixed technologies (Docusaurus, FastAPI, OpenAI, Qdrant, Gemini, ChatKit, Better-Auth, Neon Postgres)
- Researched best practices for each technology
- Identified potential risks and mitigation strategies

### Phase 1: Core Infrastructure (Week 1)
- Set up Docusaurus frontend with custom purple/neon theme
- Implement FastAPI backend with basic RAG functionality
- Connect to Qdrant Cloud for vector storage
- Implement basic chat interface with ChatKit SDK
- Ensure WCAG 2.1 AA compliance

### Phase 2: VLA Content & Capstone (Week 2)
- Create Module 4 content covering Vision-Language-Action pipeline
- Develop capstone project walkthrough with ROS 2 integration
- Implement voice-to-action processing concepts
- Add cognitive planning with LLMs content
- Ensure all content is properly chunked and embedded

### Phase 3: Advanced Features (Week 3)
- Implement text-selection query mode
- Add proper citation system ("Source: Module X → Week Y → Section Z")
- Implement RAG-only enforcement mechanisms
- Add performance optimizations
- Conduct thorough testing

### Phase 4: Bonus Features (Week 3-4)
- Implement Better-Auth for user authentication
- Add personalization features (complexity adjustment)
- Implement Urdu translation system with RTL support
- Create live voice-to-action demo (WebRTC + Whisper.cpp)

## Risk Mitigation

### Technical Risks

1. **Qdrant Free-Tier Limits**
   - Mitigation: Monitor usage, implement caching, plan for upgrade if needed
   - Monitoring: Track query volume and vector storage usage

2. **OpenAI Rate Limits**
   - Mitigation: Implement request queuing and retry logic, batch processing
   - Fallback: Cache common queries, implement rate limiting

3. **Gemini Context Window vs Chunk Size**
   - Mitigation: Test with maximum content, implement chunk overlap strategy
   - Validation: Ensure 500-token chunks work within context limits

4. **Serverless Cold Starts**
   - Mitigation: Use Vercel/Fly.io with cold start optimization
   - Design: Handle occasional delays gracefully in UI

### Performance Risks

1. **RAG Response Time > 3s**
   - Mitigation: Optimize vector search, use efficient retrieval algorithms
   - Monitoring: Track 95th percentile response times

2. **Page Load Time > 2s on 3G**
   - Mitigation: Optimize Docusaurus build, implement proper asset compression
   - Testing: Regular Lighthouse audits

## Deployment Strategy

### Frontend: GitHub Pages
- Static Docusaurus build deployed to GitHub Pages
- Custom domain support
- CDN delivery for optimal performance
- Automated deployment via GitHub Actions

### Backend: Vercel Serverless or Fly.io
- FastAPI application deployed to serverless platform
- Auto-scaling based on demand
- Environment-specific configurations
- Health checks and monitoring

### CI/CD: GitHub Actions
- Automated testing on pull requests
- Automatic deployment to staging on main branch
- Manual promotion to production
- Embedding generation on content updates

## Testing Strategy

### Unit Tests
- Backend service functions (pytest)
- Frontend component logic (Jest/RTL)
- Data model validations

### Integration Tests
- API endpoint functionality
- Database operations
- External service integrations (OpenAI, Qdrant, Gemini)

### Contract Tests
- API schema validation
- Request/response format verification
- Error handling verification

### Performance Tests
- Load testing for concurrent users
- Response time validation (<3s requirement)
- Page load performance on various network conditions

## Monitoring & Observability

### Backend Monitoring
- Response time tracking (goal: <3s for 95th percentile)
- Error rate monitoring
- API usage metrics
- Qdrant query performance

### Frontend Monitoring
- Page load performance (Lighthouse scores)
- User interaction tracking
- Error reporting
- Accessibility compliance checks

### Business Metrics
- User engagement with VLA content
- Capstone project completion rates
- Q&A system usage patterns
- Citation accuracy verification

## Success Metrics

### Technical Metrics
- ✅ Page load < 2.0s on 3G (Lighthouse ≥ 95)
- ✅ RAG response time < 3s (95th percentile)
- ✅ Vector search < 500ms
- ✅ Full static build < 5 minutes
- ✅ Embedding generation: batch max 100 chunks/request

### Business Metrics
- ✅ Capstone project achieves ≥ 85% success rate in test scenes
- ✅ RAG can fully explain any step of capstone when text is selected
- ✅ Citation format includes: "Source: Module X → Week Y → Section Z"
- ✅ Support for 100 concurrent users
- ✅ WCAG 2.1 AA compliance verified

## Architecture Decision Records (ADRs)

### ADR-001: Hybrid Static + Serverless Architecture
**Context**: Need to balance performance, cost, and scalability for educational textbook platform
**Decision**: Use static Docusaurus frontend on GitHub Pages with serverless FastAPI backend
**Rationale**: Optimal for read-heavy content with dynamic RAG operations; cost-effective; highly scalable
**Status**: Accepted

### ADR-002: Pre-generated Embeddings Strategy
**Context**: Need to ensure fast RAG responses while maintaining content consistency
**Decision**: Generate embeddings at build time rather than runtime
**Rationale**: Reduces query latency; ensures consistent availability; meets performance requirements
**Status**: Accepted

### ADR-003: RAG-only Enforcement Mechanism
**Context**: Need to prevent hallucination while maintaining educational accuracy
**Decision**: Combine system prompts, temperature control, and citation validation middleware
**Rationale**: Ensures responses are based solely on textbook content; maintains educational integrity
**Status**: Accepted

### ADR-004: Text-Selection Query Handling
**Context**: Need to provide contextual Q&A based on user-selected text
**Decision**: Frontend captures selection via window.getSelection() → backend /api/chat/selection
**Rationale**: Provides intuitive user experience; maintains context between frontend and backend
**Status**: Accepted

### ADR-005: Purple/Neon Theme Implementation
**Context**: Need consistent branding across all educational content
**Decision**: Custom Docusaurus theme using CSS variables and component swizzling
**Rationale**: Maintains brand consistency; ensures accessibility compliance; mobile-responsive
**Status**: Accepted

## Operational Readiness

### Deployment
- Automated GitHub Actions pipeline
- Staging environment for testing
- Rollback procedures
- Environment-specific configurations

### Monitoring
- Real-time performance dashboards
- Error alerting
- Usage analytics
- Content availability checks

### Maintenance
- Regular dependency updates
- Performance optimization
- Content refresh procedures
- User feedback integration

## Final Outcome

This implementation plan provides a comprehensive roadmap for building the Vision-Language-Action (VLA) & Capstone module of the Physical AI & Humanoid Robotics textbook. The solution leverages the required technology stack to deliver a world-class, accessible, and performant educational platform with strict RAG-only Q&A capabilities and proper citation system.

The plan ensures all performance requirements are met while maintaining the required aesthetic and accessibility standards. The architecture is designed to scale to 100 concurrent users while providing fast response times and reliable content delivery.

## Reflection

The planning phase has successfully identified all necessary components and validated the technical approach against the project requirements. The hybrid architecture balances performance and cost while meeting all specified constraints. The risk mitigation strategies address potential challenges with the free-tier services and performance requirements.

## Next Steps

1. Execute the implementation phases as outlined
2. Generate detailed tasks from this plan using `/sp.tasks`
3. Begin Phase 1: Core Infrastructure implementation
4. Create Prompt History Record for this planning session
