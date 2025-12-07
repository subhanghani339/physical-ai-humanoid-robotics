import os
import requests
from pathlib import Path

# Path to your docs folder
DOCS_PATH = Path("../docs")
API_URL = "http://localhost:8000/api/index"

def index_markdown_file(file_path):
    """Read and index a markdown file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Skip frontmatter
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                content = parts[2].strip()
        
        # Skip very short content
        if len(content) < 100:
            print(f"⏭️  Skipping {file_path.name} (too short)")
            return
        
        # Split into chunks (every 1000 characters)
        chunk_size = 1000
        chunks = [content[i:i+chunk_size] for i in range(0, len(content), chunk_size)]
        
        for i, chunk in enumerate(chunks):
            response = requests.post(
                API_URL,
                json={
                    "text": chunk,
                    "source": f"{file_path.name} (chunk {i+1})"
                }
            )
            
            if response.status_code == 200:
                print(f"✅ Indexed {file_path.name} chunk {i+1}/{len(chunks)}")
            else:
                print(f"❌ Failed to index {file_path.name}: {response.text}")
    
    except Exception as e:
        print(f"❌ Error indexing {file_path}: {e}")

def index_all_docs():
    """Index all markdown files in docs folder"""
    print("🚀 Starting to index book content...\n")
    
    markdown_files = list(DOCS_PATH.rglob("*.md"))
    
    if not markdown_files:
        print(f"❌ No markdown files found in {DOCS_PATH}")
        return
    
    print(f"📚 Found {len(markdown_files)} markdown files\n")
    
    for md_file in markdown_files:
        index_markdown_file(md_file)
    
    print(f"\n✅ Indexing complete! Indexed {len(markdown_files)} files")

if __name__ == "__main__":
    # Check if backend is running
    try:
        response = requests.get("http://localhost:8000/health")
        if response.status_code == 200:
            print("✅ Backend is running\n")
            index_all_docs()
        else:
            print("❌ Backend is not responding correctly")
    except:
        print("❌ Backend is not running!")
        print("Please start the backend first with: python main.py")