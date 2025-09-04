# test_run.py
import requests
from bs4 import BeautifulSoup

def test_scraping():
    url = "https://www.bbc.com/news"
    print(f"🧪 Testing fetch from {url}...")
    try:
        response = requests.get(url, timeout=10, headers={'User-Agent': 'TestBot'})
        assert response.status_code == 200
        soup = BeautifulSoup(response.text, 'html.parser')
        title = soup.find('title')
        assert title is not None
        print(f"✅ Success! Page title: {title.get_text()[:50]}...")
    except Exception as e:
        print(f"❌ Test failed: {e}")
        exit(1)

if __name__ == "__main__":
    test_scraping()
