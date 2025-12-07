# Data Model: Module 4 – Vision-Language-Action (VLA) & Capstone

## Entity Schemas

### 1. Content Chunk Entity (Qdrant Collection)

**Collection Name**: `textbook_chunks`

**Fields**:
- `id` (string): Unique identifier for the chunk
- `module_id` (string): Module identifier (e.g., "004-module-4-vla")
- `week_number` (integer): Week number in the 13-week syllabus
- `section_path` (string): Path to the specific section
- `section_title` (string): Title of the section
- `content` (string): The chunked content (max 500 tokens)
- `embedding` (array of floats): OpenAI embedding vector
- `overlap_start` (integer): Start position of overlap in original content
- `overlap_end` (integer): End position of overlap in original content
- `source_url` (string): URL reference to the original content
- `created_at` (datetime): Timestamp of creation
- `updated_at` (datetime): Timestamp of last update

**Relationships**:
- One-to-many with Module (one module has many content chunks)
- One-to-many with Week (one week has many content chunks)

**Validation Rules**:
- `content` must be ≤ 500 tokens
- `embedding` must be a valid OpenAI embedding vector
- `module_id` must reference an existing module
- `week_number` must be between 1-13

### 2. Module Entity (Qdrant Collection)

**Collection Name**: `modules`

**Fields**:
- `id` (string): Module identifier (e.g., "004-module-4-vla")
- `title` (string): Module title
- `description` (string): Module description
- `start_week` (integer): Starting week number
- `end_week` (integer): Ending week number
- `total_chunks` (integer): Number of content chunks in the module
- `created_at` (datetime): Timestamp of creation
- `updated_at` (datetime): Timestamp of last update

**Relationships**:
- One-to-many with Content Chunk (one module has many content chunks)
- One-to-many with Week (one module spans multiple weeks)

**Validation Rules**:
- `id` must follow format "###-module-X-[name]"
- `start_week` must be ≤ `end_week`
- `start_week` and `end_week` must be between 1-13

### 3. User Entity (Neon Postgres Table)

**Table Name**: `users`

**Fields**:
- `id` (UUID): Primary key, auto-generated
- `email` (string): User email address (unique)
- `name` (string): User's full name
- `created_at` (timestamp): Account creation timestamp
- `updated_at` (timestamp): Last update timestamp
- `email_verified` (boolean): Whether email is verified
- `preferences` (JSONB): User preferences including language, accessibility settings

**Relationships**:
- One-to-many with Chat Session (one user has many chat sessions)
- One-to-many with Personalization Settings (one user has many personalization settings)

**Validation Rules**:
- `email` must be a valid email address and unique
- `name` must not be empty
- `email_verified` defaults to false

### 4. Chat Session Entity (Neon Postgres Table)

**Table Name**: `chat_sessions`

**Fields**:
- `id` (UUID): Primary key, auto-generated
- `user_id` (UUID): Foreign key to users table
- `module_id` (string): Module identifier for the session
- `created_at` (timestamp): Session creation timestamp
- `updated_at` (timestamp): Last interaction timestamp
- `title` (string): Auto-generated title from first query

**Relationships**:
- Many-to-one with User (many chat sessions belong to one user)
- One-to-many with Chat Message (one chat session has many messages)

**Validation Rules**:
- `user_id` must reference an existing user
- `module_id` must be a valid module identifier

### 5. Chat Message Entity (Neon Postgres Table)

**Table Name**: `chat_messages`

**Fields**:
- `id` (UUID): Primary key, auto-generated
- `session_id` (UUID): Foreign key to chat_sessions table
- `role` (string): Message role ("user" or "assistant")
- `content` (text): Message content
- `sources` (JSONB): Array of source citations in format [{"module": "X", "week": "Y", "section": "Z", "url": "URL"}]
- `created_at` (timestamp): Message creation timestamp
- `token_count` (integer): Number of tokens in the message

**Relationships**:
- Many-to-one with Chat Session (many messages belong to one session)

**Validation Rules**:
- `role` must be either "user" or "assistant"
- `session_id` must reference an existing chat session
- `sources` must follow the specified format when present

### 6. Personalization Settings Entity (Neon Postgres Table)

**Table Name**: `personalization_settings`

**Fields**:
- `id` (UUID): Primary key, auto-generated
- `user_id` (UUID): Foreign key to users table
- `module_id` (string): Module identifier
- `complexity_level` (integer): Complexity level (1-5 scale)
- `preferred_language` (string): Preferred language code (default: "en")
- `accessibility_features` (JSONB): Accessibility preferences
- `created_at` (timestamp): Creation timestamp
- `updated_at` (timestamp): Last update timestamp

**Relationships**:
- Many-to-one with User (many personalization settings belong to one user)

**Validation Rules**:
- `user_id` must reference an existing user
- `complexity_level` must be between 1-5
- `preferred_language` must be a valid language code

### 7. Translation Entity (Neon Postgres Table)

**Table Name**: `translations`

**Fields**:
- `id` (UUID): Primary key, auto-generated
- `content_chunk_id` (string): Reference to content chunk ID
- `language_code` (string): Target language code (e.g., "ur" for Urdu)
- `translated_content` (text): Translated content
- `status` (string): Translation status ("pending", "reviewed", "published")
- `translator_notes` (text): Notes from translator
- `created_at` (timestamp): Creation timestamp
- `updated_at` (timestamp): Last update timestamp

**Relationships**:
- Many-to-one with Content Chunk (many translations for one content chunk)

**Validation Rules**:
- `language_code` must be a valid language code
- `status` must be one of the allowed values
- `content_chunk_id` must reference an existing content chunk

## Qdrant Collections Schema

### 1. textbook_chunks Collection Schema

```json
{
  "vector_size": 1536,
  "distance": "Cosine",
  "hnsw_config": {
    "m": 16,
    "ef_construct": 100
  },
  "quantization_config": null,
  "on_disk_payload": true
}
```

**Payload Schema**:
```json
{
  "id": {"type": "keyword"},
  "module_id": {"type": "keyword"},
  "week_number": {"type": "integer"},
  "section_path": {"type": "keyword"},
  "section_title": {"type": "text"},
  "content": {"type": "text"},
  "overlap_start": {"type": "integer"},
  "overlap_end": {"type": "integer"},
  "source_url": {"type": "keyword"},
  "created_at": {"type": "datetime"}
}
```

### 2. modules Collection Schema

```json
{
  "vector_size": 384,
  "distance": "Cosine",
  "hnsw_config": {
    "m": 16,
    "ef_construct": 100
  },
  "quantization_config": null,
  "on_disk_payload": true
}
```

**Payload Schema**:
```json
{
  "id": {"type": "keyword"},
  "title": {"type": "text"},
  "description": {"type": "text"},
  "start_week": {"type": "integer"},
  "end_week": {"type": "integer"},
  "total_chunks": {"type": "integer"},
  "created_at": {"type": "datetime"}
}
```

## Postgres Database Schema

### 1. Users Table
```sql
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  name VARCHAR(255) NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  email_verified BOOLEAN DEFAULT FALSE,
  preferences JSONB DEFAULT '{}'
);
```

### 2. Chat Sessions Table
```sql
CREATE TABLE chat_sessions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  module_id VARCHAR(255) NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  title VARCHAR(255)
);
```

### 3. Chat Messages Table
```sql
CREATE TABLE chat_messages (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
  role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
  content TEXT NOT NULL,
  sources JSONB DEFAULT '[]',
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  token_count INTEGER
);
```

### 4. Personalization Settings Table
```sql
CREATE TABLE personalization_settings (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  module_id VARCHAR(255) NOT NULL,
  complexity_level INTEGER CHECK (complexity_level BETWEEN 1 AND 5),
  preferred_language VARCHAR(10) DEFAULT 'en',
  accessibility_features JSONB DEFAULT '{}',
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);
```

### 5. Translations Table
```sql
CREATE TABLE translations (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  content_chunk_id VARCHAR(255) NOT NULL,
  language_code VARCHAR(10) NOT NULL,
  translated_content TEXT NOT NULL,
  status VARCHAR(20) DEFAULT 'pending' CHECK (status IN ('pending', 'reviewed', 'published')),
  translator_notes TEXT,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);
```

## State Transitions

### Chat Session States
- `active`: Session is currently being used
- `inactive`: No activity for >30 minutes
- `archived`: Session is old and moved to archive

### Translation States
- `pending`: Translation in progress
- `reviewed`: Translation reviewed by human
- `published`: Translation is live and available

## Indexes

### Postgres Indexes
```sql
-- Users table
CREATE INDEX idx_users_email ON users(email);

-- Chat sessions table
CREATE INDEX idx_chat_sessions_user_id ON chat_sessions(user_id);
CREATE INDEX idx_chat_sessions_module_id ON chat_sessions(module_id);

-- Chat messages table
CREATE INDEX idx_chat_messages_session_id ON chat_messages(session_id);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at);

-- Personalization settings table
CREATE INDEX idx_personalization_user_module ON personalization_settings(user_id, module_id);

-- Translations table
CREATE INDEX idx_translations_chunk_lang ON translations(content_chunk_id, language_code);
CREATE INDEX idx_translations_status ON translations(status);
```

### Qdrant Indexes
- Payload indexes on `module_id`, `week_number`, and `section_path` for efficient filtering
- Vector index for similarity search on embeddings