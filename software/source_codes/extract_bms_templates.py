import bms_viewer
import os

TEMPLATE_DIR = "bms_viewer_tool/templates"
os.makedirs(TEMPLATE_DIR, exist_ok=True)

templates = {
    "index.html": bms_viewer.HTML_VIEWER,
    "dashboard.html": bms_viewer.DASHBOARD_HTML,
    "sd_analyzer.html": bms_viewer.SD_ANALYZER_HTML
}

print(f"Extracting templates to {TEMPLATE_DIR}...")

for filename, content in templates.items():
    # Sanitize using the function from bms_viewer to ensure we get the clean versions
    # Note: bms_viewer.sanitize_html takes a string and returns a string
    sanitized = bms_viewer.sanitize_html(content)
    
    path = os.path.join(TEMPLATE_DIR, filename)
    with open(path, "w", encoding="utf-8") as f:
        f.write(sanitized)
    
    print(f"  - Extracted {filename} ({len(sanitized)} chars)")

print("Done.")
