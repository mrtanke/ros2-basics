import threading
import requests

class Download:
    def download(self, url, callback_world_count):
        print(f'Thread {threading.current_thread().name} is downloading from {url}')
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback_world_count(url, response.text)

    def start_download(self, url, callback_world_count):
        # self.download(url, callback_world_count)
        thread = threading.Thread(target=self.download, args=(url, callback_world_count))
        thread.start()


def world_count(url, result):
    """
    normal function, used for callback
    """
    print(f"Downloaded from {url}: {result[:5]} words")


def main():
    download = Download()
    download.start_download("http://localhost:8000/novel/novel1.txt", world_count)
    download.start_download("http://localhost:8000/novel/novel2.txt", world_count)
    download.start_download("http://localhost:8000/novel/novel3.txt", world_count)