# app.py
import requests
from bs4 import BeautifulSoup
import ollama
import time

# ================= CONFIG =================
NEWS_SITES = [
    "https://www.bbc.com/news",
    "https://techcrunch.com",
    "https://arstechnica.com",
]

INTERESTED_TOPIC = "artificial intelligence"

MODEL = "llama3.2"  # Make sure you've run: ollama run llama3
MAX_ARTICLES_PER_SITE = 5
PREVIEW_LENGTH = 500
# ==========================================

def fetch_page(url):
    try:
        response = requests.get(url, timeout=10, headers={'User-Agent': 'Mozilla/5.0'})
        return response.text
    except Exception as e:
        print(f"❌ Failed to load {url}: {e}")
        return ""

def extract_articles(html, base_url):
    soup = BeautifulSoup(html, 'html.parser')
    articles = []
    for link in soup.find_all('a', href=True)[:MAX_ARTICLES_PER_SITE]:
        href = link['href']
        title = link.get_text(strip=True)
        if len(title) < 10:
            continue
        if href.startswith('/'):
            href = base_url.rstrip('/') + href
        elif not href.startswith('http'):
            continue
        articles.append({'title': title, 'url': href})
    return articles

def contains_topic(text, topic):
    return topic.lower() in text.lower()

def main():
    print("🔍 Fetching latest news...")
    relevant_articles = []

    for site in NEWS_SITES:
        print(f"📡 Checking {site}...")
        html = fetch_page(site)
        articles = extract_articles(html, site)

        for article in articles:
            page_html = fetch_page(article['url'])
            if not page_html:
                continue

            soup = BeautifulSoup(page_html, 'html.parser')
            body = soup.find('body')
            text = body.get_text()[:2000] if body else ""

            if contains_topic(text, INTERESTED_TOPIC) or contains_topic(article['title'], INTERESTED_TOPIC):
                relevant_articles.append({
                    'title': article['title'],
                    'url': article['url'],
                    'preview': text[:PREVIEW_LENGTH]
                })
                print(f"✅ Match: {article['title'][:60]}...")

            time.sleep(0.5)  # Be respectful

        time.sleep(1)

    if not relevant_articles:
        print(f"No articles found about '{INTERESTED_TOPIC}'.")
        return

    # === Summarize with Ollama ===
    # Build article content first
    article_texts = []
    for a in relevant_articles:
        article_texts.append(f"Title: {a['title']}\nContent: {a['preview']}...")

    articles_str = "\n\n".join(article_texts)

    prompt = f"""Think step by step. Summarize key points from these {len(relevant_articles)} articles about '{INTERESTED_TOPIC}'.
Be concise, under 150 words.

{articles_str}

Summary:"""

    print("\n📝 Prompt sent to Ollama:\n")
    print(prompt)
    print("\n---\n")

    print("🧠 Asking Ollama to summarize...")
    stream = ollama.chat(
        model=MODEL,
        messages=[{'role': 'user', 'content': prompt}],
        stream=True,
    )

    print("\n📝 Daily Digest:")
    for chunk in stream:
        print(chunk['message']['content'], end='', flush=True)
    print("\n")
