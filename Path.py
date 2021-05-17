import pygame
from Grid import draw_grid

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Path Finding Algorithm")

WHITE = (255, 255, 255)


def reconstruct_path(came_from, current, draw):  # Constructing the final path
	while current in came_from:
		current = came_from[current]
		current.make_path()
		draw()


def draw(win, grid, rows, width):
	win.fill(WHITE)

	for row in grid:
		for spot in row:
			spot.draw(win)

	draw_grid(win, rows, width)
	pygame.display.update()
