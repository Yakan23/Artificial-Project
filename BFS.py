from Path import *
from collections import deque



def bfs(draw, grid, start, end):
    open_set = deque()
    open_set.append(start)
    came_from = {}
    open_set_hash = {start}
    #came_from[start]=None
    while open_set:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.popleft()
       	open_set_hash.remove(current)

        if current == end:
           reconstruct_path(came_from, end, draw)
           end.make_end()
           return True

        for neighbor in current.neighbors:
            if neighbor not in came_from:
                open_set_hash.add(neighbor)
                open_set.append(neighbor)
                came_from[neighbor] = current
                if neighbor != start and neighbor != end:
                    neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()

    return False
