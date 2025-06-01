import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import threading
import time

# ROS2 관련 모듈
import rclpy
from rclpy.node import Node
from Convenience_store_chef_msgs.srv import OrderService
"""
에러 처리 
서비스 서버와 연결이 안되고 키오스크는 정상 작동
but 서버로부터 success를 받지 못하면 서버와 연결이 안된다 뜨고 결제는 불가능
+ 버튼을 누르다 재고 수량 보다 크게 클릭하면 에러문이 나옴 
서비스 메시지 주위 사항
quantities가 [1, 2, 3]이면
request.quantities  # => array('i', [1, 2, 3]) int32[] 메시지 타입 출력하면 이렇게 나옴
리스트 형식으로 바꿔서 사용
"""

# 초기 재고 설정
stock = {
    "김밥": 5,
    "삼각김밥": 5,
    "라면": 5
}

prices = {
    "김밥": 2000,
    "삼각김밥": 1200,
    "라면": 3500
}

quantities = {item: 0 for item in stock}

class OrderClientNode(Node):
    def __init__(self):
        super().__init__('order_client_node')
        self.cli = self.create_client(OrderService, '/order_service')
        self.connected = False

        def wait_for_service():
            while rclpy.ok() and not self.cli.wait_for_service(timeout_sec=1.0):
                print("[ROS2] 서비스 서버 대기 중...")
            self.connected = True
            print("[ROS2] 서비스 서버 연결 완료")

        self._thread = threading.Thread(target=wait_for_service, daemon=True)
        self._thread.start()

    def send_order(self, items, qtys):
        if not self.connected:
            return None
        request = OrderService.Request()
        request.items = items
        request.quantities = qtys
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    ros_node = OrderClientNode()

    root = tk.Tk()
    root.title("편의점 키오스크")
    root.geometry("1600x1200")

    for i in range(7):
        root.grid_rowconfigure(i, weight=1)
    for i in range(6):
        root.grid_columnconfigure(i, weight=1)

    def load_image(filename):
        try:
            img = Image.open(filename)
            img = img.resize((150, 150))
            return ImageTk.PhotoImage(img)
        except:
            return None

    images = {
        "김밥": load_image("/home/seokhwan/test/kimbap.png"),
        "삼각김밥": load_image("/home/seokhwan/test/3_kimbap.png"),
        "라면": load_image("/home/seokhwan/test/ramen.png")
    }

    qty_labels = {}
    stock_labels = {}

    def create_menu_item(name, row):
        tk.Label(root, text=f"{name} - {prices[name]}원", font=("Arial", 20)).grid(row=row, column=0, pady=20, sticky="w")
        if images[name]:
            tk.Label(root, image=images[name]).grid(row=row, column=1)
        tk.Button(root, text="-", font=("Arial", 18), width=3,
                  command=lambda: update_quantity(name, -1)).grid(row=row, column=2)
        qty_labels[name] = tk.Label(root, text="0", width=5, font=("Arial", 18))
        qty_labels[name].grid(row=row, column=3)
        tk.Button(root, text="+", font=("Arial", 18), width=3,
                  command=lambda: update_quantity(name, 1)).grid(row=row, column=4)
        stock_labels[name] = tk.Label(root, text=f"재고: {stock[name]}", fg="blue", font=("Arial", 16))
        stock_labels[name].grid(row=row, column=5)

    def update_quantity(name, delta):
        if delta == 1:
            if quantities[name] < stock[name]:
                quantities[name] += 1
            else:
                messagebox.showwarning("재고 부족", f"{name}의 재고가 부족합니다.")
        elif delta == -1 and quantities[name] > 0:
            quantities[name] -= 1
        qty_labels[name].config(text=str(quantities[name]))

    def place_order():
        selected_items = [(item, quantities[item]) for item in quantities if quantities[item] > 0]
        if not selected_items:
            messagebox.showinfo("알림", "하나 이상의 상품을 선택하세요.")
            return

        total = sum(prices[item] * qty for item, qty in selected_items)
        summary_lines = [f"{item} {qty}개 = {prices[item]*qty}원" for item, qty in selected_items]

        confirm_win = tk.Toplevel(root)
        confirm_win.title("주문 확인")
        confirm_win.geometry("500x400")
        confirm_win.grab_set()

        tk.Label(confirm_win, text="주문 내용을 확인해주세요", font=("Arial", 16, "bold")).pack(pady=10)
        summary_text = "\n".join(summary_lines) + f"\n\n총 금액: {total}원"
        tk.Label(confirm_win, text=summary_text, font=("Arial", 14)).pack(pady=10)

        def on_confirm():
            item_names = [item for item, qty in selected_items]
            item_qtys = [qty for item, qty in selected_items]

            def ros_request():
                result = ros_node.send_order(item_names, item_qtys)
                if result is None:
                    messagebox.showerror("서비스 오류", "ROS2 서비스에 연결할 수 없습니다.")
                elif not result.success:
                    messagebox.showerror("주문 실패", "서버가 주문을 처리하지 못했습니다.")
                else:
                    for item, qty in selected_items:
                        stock[item] -= qty
                        stock_labels[item].config(text=f"재고: {stock[item]}")
                    result_label.config(text=f"총 금액: {total}원\n결제가 완료되었습니다.")
                    disable_buttons()
                    threading.Thread(target=delayed_reset, daemon=True).start()

            threading.Thread(target=ros_request, daemon=True).start()
            confirm_win.destroy()

        def on_cancel():
            confirm_win.destroy()
            messagebox.showinfo("주문 취소", "주문을 수정해주세요.")

        button_frame = tk.Frame(confirm_win)
        button_frame.pack(pady=20)
        tk.Button(button_frame, text="예", font=("Arial", 14), bg="green", fg="white", width=10, command=on_confirm).pack(side="left", padx=20)
        tk.Button(button_frame, text="아니오", font=("Arial", 14), bg="red", fg="white", width=10, command=on_cancel).pack(side="right", padx=20)

    def disable_buttons():
        order_button.config(state="disabled")
        for item in qty_labels:
            qty_labels[item].config(state="disabled")

    def delayed_reset():
        time.sleep(5)
        reset_all()
        order_button.config(state="normal")
        for item in qty_labels:
            qty_labels[item].config(state="normal")

    def order_more():
        more_clicked[0] = True
        for item in quantities:
            quantities[item] = 0
            qty_labels[item].config(text="0")
        result_label.config(text="")
        more_button.grid_remove()

    def reset_all():
        for item in quantities:
            quantities[item] = 0
            qty_labels[item].config(text="0")
        result_label.config(text="")
        more_button.grid_remove()

    for idx, name in enumerate(["김밥", "삼각김밥", "라면"]):
        create_menu_item(name, idx)

    order_button = tk.Button(root, text="주문하기", command=place_order, bg="green", fg="white", font=("Arial", 20), width=15, height=2)
    order_button.grid(row=5, column=0, columnspan=3, pady=20)

    global result_label
    result_label = tk.Label(root, text="", font=("Arial", 20))
    result_label.grid(row=5, column=3, columnspan=3)

    global more_clicked, more_button
    more_clicked = [False]
    more_button = tk.Button(root, text="더 주문하시겠습니까?", command=order_more, bg="orange", font=("Arial", 18), width=25, height=2)
    more_button.grid(row=6, column=0, columnspan=6, pady=10)
    more_button.grid_remove()

    root.mainloop()

if __name__ == '__main__':
    main()
