import asyncio
import threading
import time
import unittest


class TestAsyncio(unittest.TestCase):
    def test_threading_vs_asyncio(self):
        def fetch_data(api):
            # print(f"Fetching from {api}")
            time.sleep(2)  # Simulating network delay
            # print(f"Done fetching from {api}")

        start = time.time()
        threads = [threading.Thread(target=fetch_data, args=(f"API-{i}",)) for i in range(5)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        threading_time = time.time() - start

        # print(f"Total time: {threading_time:.2f} seconds")

        async def fetch_data(api):
            # print(f"Fetching from {api}")
            await asyncio.sleep(2)  # Simulating network delay
            # print(f"Done fetching from {api}")

        async def main():
            tasks = [fetch_data(f"API-{i}") for i in range(5)]
            await asyncio.gather(*tasks)

        start = time.time()
        asyncio.run(main())

        asyncio_time = time.time() - start
        # print(f"Total time: {asyncio_time:.2f} seconds")
        if threading_time < asyncio_time:
            print("Threading is faster than asyncio by", asyncio_time - threading_time)
        else:
            print("Asyncio is faster than threading by", threading_time - asyncio_time)
