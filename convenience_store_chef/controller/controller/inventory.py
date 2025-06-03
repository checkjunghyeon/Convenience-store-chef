from collections import deque


class Inventory:
    def __init__(self):
        self.stocks = {
            '김밥': deque([
                [-147.09, 643.04, -109.75, 171.20, 179.43, 170.53],
                [-78.62, 643.61, -110.59, 19.88, -178.57, 19.32],
            ]),
            
            '삼각김밥': deque([
                [-116.53, 473.10, -145.24, 32.63, -179.08, 31.78],
                [-39.87, 483.96, -144.34, 30.12, -179.16, 29.53],
            ]),

            '라면': deque([
                [350, -150, 120, 0, 90, 0],         # 임시좌표
                [360, -150, 120, 0, 90, 0],         # 임시좌표
            ]),
        }

    def get_next(self, item: str):
        if item not in self.stocks or len(self.stocks[item]) == 0:
            raise ValueError(f"'{item}' 재고 없음!")
        return self.stocks[item].popleft()

    def add_stock(self, item: str, position):
        if item not in self.stocks:
            self.stocks[item] = deque()
        self.stocks[item].append(position)

    def get_remaining(self, item: str) -> int:
        return len(self.stocks.get(item, []))