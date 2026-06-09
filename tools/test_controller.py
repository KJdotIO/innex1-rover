"""Quick test: does pygame see your Xbox controller input?

Run this, click the window, and move sticks / press buttons.
If you see values change, the controller works.
"""

import sys
import time

import pygame

pygame.init()
pygame.joystick.init()

screen = pygame.display.set_mode((500, 350))
pygame.display.set_caption("Controller Test - CLICK THIS WINDOW FIRST")
font = pygame.font.SysFont("consolas", 16)
clock = pygame.time.Clock()

count = pygame.joystick.get_count()
if count == 0:
    print("ERROR: No controller found!")
    sys.exit(1)

js = pygame.joystick.Joystick(0)
js.init()
print(f"Controller: {js.get_name()}")
print(f"Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}")
print("Click the pygame window and move sticks / press buttons.")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.JOYBUTTONDOWN:
            print(f"  >>> BUTTON {event.button} DOWN")
        if event.type == pygame.JOYBUTTONUP:
            print(f"      button {event.button} up")

    screen.fill((20, 20, 20))

    y = 15
    # Title
    surf = font.render(f"Controller: {js.get_name()}", True, (100, 200, 255))
    screen.blit(surf, (15, y)); y += 30

    # Axes
    surf = font.render("--- Axes ---", True, (180, 180, 180))
    screen.blit(surf, (15, y)); y += 25
    for a in range(js.get_numaxes()):
        val = js.get_axis(a)
        color = (60, 255, 60) if abs(val) > 0.15 else (120, 120, 120)
        label = {0: "L-Stick X", 1: "L-Stick Y", 2: "R-Stick X", 3: "R-Stick Y", 4: "L-Trigger", 5: "R-Trigger"}.get(a, f"Axis {a}")
        surf = font.render(f"  {label} (axis {a}): {val:+.3f}", True, color)
        screen.blit(surf, (15, y)); y += 22

    y += 10
    # Buttons
    surf = font.render("--- Buttons ---", True, (180, 180, 180))
    screen.blit(surf, (15, y)); y += 25
    pressed = [b for b in range(js.get_numbuttons()) if js.get_button(b)]
    if pressed:
        surf = font.render(f"  PRESSED: {pressed}", True, (255, 255, 60))
    else:
        surf = font.render("  (none pressed)", True, (120, 120, 120))
    screen.blit(surf, (15, y)); y += 25

    # Hat
    if js.get_numhats() > 0:
        hat = js.get_hat(0)
        color = (60, 255, 60) if hat != (0, 0) else (120, 120, 120)
        surf = font.render(f"  D-pad: {hat}", True, color)
        screen.blit(surf, (15, y)); y += 25

    # Reminder
    surf = font.render("Close window or Ctrl+C to quit", True, (80, 80, 80))
    screen.blit(surf, (15, 320))

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
