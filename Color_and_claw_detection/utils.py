class Utils:

    def __init__(self):
        self.count = 1

    def debug(self, thing):
        print(f"[DEBUG {self.count}] {thing}\n")
        self.count+=1

    def reset(self):
        self.count = 1

    def quick_doc(self, topic:str):
        print("=============================")
        print(f"{topic.capitalize()}")
        print("=============================\n")

    def p(self, x):
        print(x)

