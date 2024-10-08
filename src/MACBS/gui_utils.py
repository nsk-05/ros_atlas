import pygame
import math

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREY = (200, 200, 200)
ORANGE = (255, 165, 0)
clr=[GREEN,RED,BLUE]
Algo="CBS"

def draw_grid(screen, grid, cell_size, width):
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            color=255*(1-grid[row, col])
            color=((color,color,color))
            pygame.draw.rect(screen, color, (col * cell_size, row * cell_size, cell_size, cell_size))

def draw_start_goal(screen, start, goal, cell_size):
    for i in range(len(start)):
        pygame.draw.circle(screen, clr[i], (start[i][1] * cell_size + cell_size // 2, start[i][0] * cell_size + cell_size // 2), cell_size // 2)
    for i in range(len(goal)):
        pygame.draw.circle(screen, clr[i], (goal[i][1] * cell_size + cell_size // 2, goal[i][0] * cell_size + cell_size // 2), cell_size // 4)

def draw_path(screen, paths, cell_size):
    i=0
    if len(paths) > 1:
        for agent,path in paths.items():
            path_points = [(point.y * cell_size + cell_size / 2, point.x * cell_size + cell_size / 2) for point in path]
            pygame.draw.lines(screen, clr[i], False, path_points, 3)
            i+=1

def draw_side_panel(screen, start_x, width, setting_start, setting_goal, setting_obstacle, clearing_obstacle):
    pygame.draw.rect(screen, GREY, (start_x, 0, width, screen.get_height()))
    draw_button(screen, start_x + 10, 100, "Set Start", setting_start)
    draw_button(screen, start_x + 10, 150, "Set Goal", setting_goal)
    draw_button(screen, start_x + 10, 200, "Add Obstacle", setting_obstacle,button_size=(200, 30))
    draw_button(screen, start_x + 10, 250, "Clear Obstacle", clearing_obstacle,button_size=(200, 30))
    draw_button(screen, start_x + 10, 300, "Clear All", False)
    draw_button(screen, start_x + 10, 350, "Search", False,button_size=(100, 40), color= BLACK)

def draw_button(screen, x, y, text, active,button_size=(120, 30),color=RED):
    font = pygame.font.Font(None, 36)
    color = GREEN if active else color
    pygame.draw.rect(screen, color, (x, y, button_size[0],button_size[1]))
    text_surface = font.render(text, True, WHITE)
    screen.blit(text_surface, (x + 10, y + 5))