from collections import deque


class Logger:

    def __init__(self):
        # queue object
        self.q = deque()
        self.counter = 0
    
    def log(self, item):
        if item not in self.q:
            if len(self.q) > 4:
                self.remove_top()
                self.q.append(f"[0{self.counter}] - "+str(item))

            else:
                self.q.append(f"[0{self.counter}] - "+str(item))
            
            self.counter += 1
    
    def remove_top(self):
        self.q.popleft()

    def get_logs(self):
        logs = []
        for log in self.q:
            logs.append(log)
        return logs