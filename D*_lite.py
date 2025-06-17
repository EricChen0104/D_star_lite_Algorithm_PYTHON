import matplotlib.pyplot as plt
import numpy as np
import heapq

class Node:
    def __init__(self, x, y):
        self.pos = (x, y)
        self.g = float('inf')
        self.rhs = float('inf')
        self.is_obstacle = False

    def __eq__(self, other):
        return isinstance(other, Node) and self.pos == other.pos

    def __hash__(self):
        return hash(self.pos)

    def __lt__(self, other):
        return self.pos < other.pos

grid_size = 20
grid = np.zeros((grid_size, grid_size))  # 0: free, 1: obstacle
node_dict = {}
for x in range(grid_size):
    for y in range(grid_size):
        node_dict[(x, y)] = Node(x, y)

# 障礙物設定
grid[5:10, 8] = 1
grid[15, 5:15] = 1
grid[12, 2:7] = 1
grid[8, 12:16] = 1
grid[1:4, 10] = 1
for (x, y), node in node_dict.items():
    if grid[x][y] == 1:
        node.is_obstacle = True

start = node_dict[(2, 3)]
goal = node_dict[(17, 14)]

def h(a, b):
    return abs(a.pos[0] - b.pos[0]) + abs(a.pos[1] - b.pos[1])

def cost(u, v):
    return float('inf') if v.is_obstacle else 1

def key(s):
    return (min(s.g, s.rhs) + h(start, s), min(s.g, s.rhs))

open_list = []
open_set = set()
visited_nodes = set()  # 記錄所有試圖拜訪的節點

def push_to_open_list(node):
    if node not in open_set:
        heapq.heappush(open_list, (key(node), node))
        open_set.add(node)
        visited_nodes.add(node)

def pop_from_open_list():
    while open_list:
        k, node = heapq.heappop(open_list)
        if node in open_set:
            open_set.remove(node)
            visited_nodes.add(node)
            return node
    return None

def neighbor(u):
    dirs = [[0, 1], [1, 0], [-1, 0], [0, -1]]
    ns = []
    for dx, dy in dirs:
        nx, ny = u.pos[0] + dx, u.pos[1] + dy
        if 0 <= nx < grid_size and 0 <= ny < grid_size:
            ns.append(node_dict[(nx, ny)])
    return ns

def update_vertex(u):
    if u != goal:
        u.rhs = min(cost(u, s) + s.g for s in neighbor(u))
    if u in open_set:
        open_set.remove(u)
    if u.g != u.rhs:
        push_to_open_list(u)

def compute_shortest_path():
    while open_list and (key(start) > open_list[0][0] or start.rhs != start.g):
        u = pop_from_open_list()
        if u is None:
            break
        if u.g > u.rhs:
            u.g = u.rhs
            for s in neighbor(u):
                update_vertex(s)
        else:
            u.g = float('inf')
            update_vertex(u)
            for s in neighbor(u):
                update_vertex(s)

def get_next_move(current):
    candidates = []
    for s in neighbor(current):
        if not s.is_obstacle:
            candidates.append((cost(current, s) + s.g, s))
    if not candidates:
        return None
    return min(candidates)[1]

def detect_new_obstacles(current):
    new_obs = []
    for s in neighbor(current):
        if grid[s.pos[0]][s.pos[1]] == 1 and not s.is_obstacle:
            s.is_obstacle = True
            new_obs.append(s)
    return new_obs

fig, ax = plt.subplots(figsize=(6, 6))
plt.ion()  # 開啟交互模式

def update_plot(current_path, visited_nodes, new_obstacles=None):
    ax.clear()
    display_grid = np.zeros((grid_size, grid_size))
    # 繪製試圖拜訪的節點
    max_g = max([n.g for n in visited_nodes if n.g != float('inf')] or [1])
    for node in visited_nodes:
        g_value = node.g if node.g != float('inf') else 0
        normalized_g = 0.3 + 0.5 * (g_value / max_g)
        display_grid[node.pos[0]][node.pos[1]] = normalized_g

    # 標記起點、終點和障礙物
    display_grid[start.pos[0]][start.pos[1]] = 0.2
    display_grid[goal.pos[0]][goal.pos[1]] = 0.1
    display_grid[grid == 1] = 1.0

    # 繪製地圖
    ax.imshow(display_grid, cmap='Greens')
    ax.set_title("D* Lite Dynamic Path Planning")
    ax.set_xticks(np.arange(0, grid_size, 1))
    ax.set_yticks(np.arange(0, grid_size, 1))
    ax.set_xticks(np.arange(-0.5, grid_size, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid_size, 1), minor=True)
    ax.grid(which='minor', color='black', linewidth=0.5)
    ax.invert_yaxis()

    # 繪製當前路徑（紅線）
    path_x = [node.pos[1] for node in current_path]
    path_y = [node.pos[0] for node in current_path]
    ax.plot(path_x, path_y, 'r-', linewidth=2, label='current path')
    ax.scatter(path_x[0], path_y[0], c='red', s=100, marker='o', label='start')
    ax.scatter(path_x[-1], path_y[-1], c='blue', s=100, marker='*', label='goal')

    # 標記新障礙物（如果有）
    if new_obstacles:
        for obs in new_obstacles:
            ax.scatter(obs.pos[1], obs.pos[0], c='black', s=100, marker='x', label='new obstacle')

    ax.legend()
    plt.draw()
    plt.pause(0.5)  # 暫停以顯示動畫效果

new_obstacles = []
def on_click(event):
    if event.inaxes == ax and event.button == 1:  # 左鍵點擊
        x, y = int(event.ydata + 0.5), int(event.xdata + 0.5)
        if (x, y) != start.pos and (x, y) != goal.pos and grid[x, y] == 0:
            grid[x, y] = 1
            node_dict[(x, y)].is_obstacle = True
            new_obstacles.append(node_dict[(x, y)])
            print(f"新增障礙物: ({x}, {y})")

fig.canvas.mpl_connect('button_press_event', on_click)

goal.rhs = 0
push_to_open_list(goal)
compute_shortest_path()

current = start
path = [current]
step_count = 0

print("按左鍵點擊地圖添加新障礙物，按任意鍵繼續下一步，關閉窗口結束模擬。")
while current != goal:
    # 計算當前路徑
    next_node = get_next_move(current)
    if next_node is None:
        print("無路徑到達終點！")
        break
    current = next_node
    path.append(current)
    step_count += 1

    # 檢查新障礙物（包括點擊添加的）
    detected_obs = detect_new_obstacles(current) + new_obstacles
    if detected_obs:
        for u in detected_obs:
            for s in neighbor(u):
                update_vertex(s)
            update_vertex(u)
        compute_shortest_path()
        new_obstacles = []  # 清空點擊添加的障礙物

    # 更新路徑顯示
    temp_path = [current]
    temp_node = current
    while temp_node != goal:
        next_node = get_next_move(temp_node)
        if next_node is None:
            break
        temp_path.append(next_node)
        temp_node = next_node

    # 顯示當前進度
    print(f"\n步驟 {step_count}: 當前位置 {current.pos}")
    print("當前路徑節點：")
    for node in temp_path:
        print(f"節點: {node.pos}, g 分數: {node.g}")
    update_plot(temp_path, visited_nodes, detected_obs)

    # 等待用戶輸入以繼續
    plt.waitforbuttonpress()

print("\n最終走過的路徑節點（從起點到終點）：")
for node in path:
    print(f"節點: {node.pos}, g 分數: {node.g}")
print("\n所有試圖拜訪的節點：")
for node in visited_nodes:
    print(f"節點: {node.pos}, g 分數: {node.g}")

plt.ioff()
plt.show()