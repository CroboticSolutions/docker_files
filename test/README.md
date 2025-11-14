## README

####  Build image with: 
```
docker build -t news_digest_img .
```

#### Run image with: 
```
docker run -it --network=host news_digest_img sh
```

#### In docker run: 
```
python3 app.py 
```


#### Observations: 

1. Scraping is not working as expected 
2. LLM API works as expected 

