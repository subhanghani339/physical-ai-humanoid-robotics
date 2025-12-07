import google.generativeai as genai
import pytz
from datetime import datetime, timedelta
import os
import json

# ====== CONFIG ======
API_KEY = "AIzaSyB6tlL3lLFxH7IG9UboYucpvY-vhgQQF_8"
DAILY_LIMIT = 1500     # default free quota
LOG_FILE = "quota_log.json"
USER_TIMEZONE = "Asia/Karachi"
# ====================

genai.configure(api_key=API_KEY)


def load_today_count():
    if not os.path.exists(LOG_FILE):
        return 0
    with open(LOG_FILE, "r") as f:
        logs = json.load(f)

    today = datetime.now().date().isoformat()
    return logs.get(today, 0)


def increase_count():
    today = datetime.now().date().isoformat()
    logs = {}
    if os.path.exists(LOG_FILE):
        logs = json.load(open(LOG_FILE))
    logs[today] = logs.get(today, 0) + 1
    with open(LOG_FILE, "w") as f:
        json.dump(logs, f)


def next_reset_time():
    tz = pytz.timezone(USER_TIMEZONE)
    now = datetime.now(tz)
    tomorrow = (now + timedelta(days=1)).replace(hour=0, minute=0, second=0, microsecond=0)
    return tomorrow


def make_test_call():
    model = genai.GenerativeModel("gemini-2.5-flash")
    response = model.generate_content("Hello! API test.")
    increase_count()
    return response.text


if __name__ == "__main__":
    used = load_today_count()
    remaining = DAILY_LIMIT - used

    print("\n=== GEMINI QUOTA TRACKER ===")
    print(f"Used today:      {used}")
    print(f"Remaining:       {remaining}")
    print(f"Daily limit:     {DAILY_LIMIT}")

    reset_time = next_reset_time()
    print(f"Resets at:       {reset_time}")
    print("==============================\n")

    text = make_test_call()
    print("API Test Response:")
    print(text)
