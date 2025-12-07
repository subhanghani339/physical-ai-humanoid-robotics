import os
from dotenv import load_dotenv

load_dotenv()

api_key = os.getenv("GEMINI_API_KEY")

if api_key:
    print(f"✅ API Key found!")
    print(f"First 20 characters: {api_key[:20]}...")
    print(f"Last 10 characters: ...{api_key[-10:]}")
    print(f"Total length: {len(api_key)} characters")
else:
    print("❌ No GEMINI_API_KEY found in .env!")

print("\n📁 Looking for .env file in:")
print(os.getcwd())