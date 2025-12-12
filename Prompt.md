You are an expert project auditor and AI educational mentor.

I will provide you:
1. Project folder path containing the following structure:
   - docs/                 # Textbook content (Markdown files for chapters)
   - src/                  # Frontend React + Docusaurus components
   - src/components/ChatWidget/  # AI chat frontend
   - history/              # Architecture Decision Records
   - specs/                # Feature specifications
2. Hackathon requirements (detailed below)
3. Project progress report (provided separately)

Hackathon Requirements (Context for Claude):
1. **AI/Spec-Driven Book Creation**
   - Write a textbook using Docusaurus and deploy to GitHub Pages
   - Use Spec-Kit Plus and Claude Code for book content generation
   - Chapters should be organized in `docs/` folder

2. **Integrated RAG Chatbot**
   - Embedded in published book
   - Use OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant
   - Must answer questions based on user-selected text

3. **Extra Bonus Features**
   - Reusable intelligence via Claude Code Subagents and Agent Skills (+50 points)
   - Signup and Signin using Better Auth (+50 points)
     - Collect user software/hardware background for content personalization
   - Personalized content per chapter for logged-in users (+50 points)
   - Chapter translation into Urdu via button (+50 points)

4. **Timeline**
   - Submission Deadline: Sunday, Nov 30, 2025, 06:00 PM
   - Live Presentation: Nov 30, 2025, 06:00 PM on Zoom

Your Task:
1. Scan the entire project folder and evaluate it against the Hackathon requirements.
2. Identify which requirements are complete, partially complete, or missing.
3. Highlight gaps or inconsistencies in current work.
4. Provide step-by-step suggestions for completing the remaining work.
5. Provide a summary table mapping:
   - Requirement
   - Status (Complete / Partial / Missing)
   - Notes / Suggestions

Constraints:
- Focus on AI/Spec-driven book creation using Docusaurus + Claude Code + Spec-Kit Plus
- Include assessment for RAG chatbot, personalization, Urdu translation
- Evaluate directory and file-level structure (chapters, components, specs)

Output Format:
- Stepwise findings
- Table of requirements vs status
- Next recommended steps
- Optional notes for improvements

After scanning and analysis, give a **clear actionable summary** for the project, so we can continue with the next step in development.