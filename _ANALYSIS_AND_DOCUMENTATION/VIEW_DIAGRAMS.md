# How to View the R2D2 System Architecture Diagrams

## Option 1: Mermaid Live Editor (Easiest - No Install)

**URL:** https://mermaid.live/

**Steps:**
1. Open the URL in your browser
2. Open one of these files: `huge_overall_system_map_goal.md`, `huge_overall_system_map_as_built_now.md`
3. Copy the code between \`\`\`mermaid and \`\`\`
4. Paste into the editor
5. Diagram renders automatically
6. Click "Download PNG" or "Download SVG" to save

## Option 2: VS Code / Cursor with Extension (Best Quality)

**Steps:**
1. Install extension: "Markdown Preview Mermaid Support"
   - In Cursor: Ctrl+Shift+X → Search "Mermaid" → Install
2. Open any .md file with mermaid diagrams
3. Press Ctrl+Shift+V (Cmd+Shift+V on Mac) for preview
4. Diagrams render automatically

## Option 3: GitHub (If Pushed)

**URL:** https://github.com/severinleuenberger/R2D2-as-real-AI-companion

GitHub automatically renders mermaid diagrams in markdown files. Just navigate to the file and view it.

## Option 4: Install Mermaid CLI (Generate Images)

```bash
# Install mermaid-cli
npm install -g @mermaid-js/mermaid-cli

# Generate PNG from markdown
mmdc -i huge_overall_system_map_goal.md -o diagram.png

# Or extract just the mermaid code to a .mmd file first, then:
mmdc -i diagram.mmd -o diagram.png
```

## Option 5: Online Markdown Viewers

- **StackEdit:** https://stackedit.io/ (supports mermaid)
- **Dillinger:** https://dillinger.io/ (supports mermaid)
- **Markdown Live Preview:** https://markdownlivepreview.com/

## Quick Test

To test if mermaid works in your viewer, try this simple diagram:

```mermaid
graph LR
    A[Start] --> B[Process]
    B --> C[End]
```

If you see a diagram with three boxes and arrows, mermaid is working!

## Files with Diagrams

- `huge_overall_system_map_goal.md` - 8 mermaid diagrams
- `huge_overall_system_map_as_built_now.md` - 3 mermaid diagrams  
- `system_architecture_gap_analysis.md` - 1 mermaid diagram

Total: 12 diagrams showing the complete R2D2 system architecture.
