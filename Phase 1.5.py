import pygame
import math
import sys
# from decimal import *
from collections import deque


def find_slope(agl):  # Converts angle into a slope with dx and dy
    agl = agl % 360
    if agl == 90:
        return [1, 0]
    elif agl == 270:
        return [-1, 0]
    elif agl == 180:
        return [0, 1]
    if agl == 0:
        return [0, -1]

    elif agl <= 90:
        dy = -1
        dx = math.tan(math.radians(agl))
    elif agl <= 180:
        dy = 1
        dx = 1 / math.tan(math.radians(agl - 90))
    elif agl <= 270:
        dy = 1
        dx = math.tan(math.radians(agl - 180)) * -1
    elif agl <= 360:  # Error
        dy = -1
        dx = 1 / math.tan(math.radians(agl - 270)) * -1

    # dx, dy = round(dx, 5)
    return [dx, dy]


def angle(dx, dy):  # Converts a dx and dy input into an angle
    dx, dy =dx, dy
    if dx == 0 and dy == 0: return "Point Error" # Error since it seems that the line is actually a point
    elif dx == 0:
        if dy > 0: return 180
        else: return 0
    elif dy == 0:
        if dx > 0: return 90
        else: return 270
    else:
        if dx > 0:
            if dy > 0: agl = 90 + math.degrees(math.atan(abs(dy/dx)))
            elif dy < 0: agl = 90 - math.degrees(math.atan(abs(dy/dx)))
        elif dx < 0:
            if dy > 0: agl = 270 - math.degrees(math.atan(abs(dy/dx)))
            elif dy < 0: agl = 270 + math.degrees(math.atan(abs(dy/dx)))

    return agl


def intersection(r_px, r_py, r_dx, r_dy, s_px, s_py, s_dx,
                 s_dy):  # Finds the T1 value which correponds to the length the input ray travels in order to hit the input wall

    if r_dx == 0 and s_dx == 0:  # Same slope specifically to avoid "ZERODIVISIONERROR"
        if r_px == s_px:
            if abs(s_py - r_py) < abs(s_py + s_dy - r_py):
                T1 = (s_py - r_py) / r_dy
            else:
                T1 = (s_py + s_dy - r_py) / r_dy
            x, y = r_px + r_dx * T1, r_py + r_dy * T1
            T2 = (y - s_py) / s_dy
        else:
            return "parallel", "_"
    elif r_dy == 0 and s_dy == 0:  # Same slope specifically to avoid "ZERODIVISIONERROR"
        if r_py == s_py:
            if abs(s_px - r_px) < abs(s_px + s_dx - r_px):
                T1 = (s_px - r_px) / r_dx
            else:
                T1 = (s_px + s_dx - r_px) / r_dx
            x, y = r_px + r_dx * T1, r_py + r_dy * T1
            T2 = (x - s_px) / s_dx
        else:
            return "parallel", "_"
    elif r_dx == 0 and s_dy == 0:
        T1 = (s_py - r_py) / r_dy
        x, y = r_px + r_dx * T1, r_py + r_dy * T1
        T2 = (x - s_px) / s_dx
    elif s_dx == 0 and r_dy == 0:
        T1 = (s_px - r_px) / r_dx
        x, y = r_px + r_dx * T1, r_py + r_dy * T1
        T2 = (y - s_py) / s_dy
    elif s_dy == 0:
        T1 = (s_py - r_py) / r_dy
        x, y = r_px + r_dx * T1, r_py + r_dy * T1
        T2 = (x - s_px) / s_dx
    elif s_dx == 0:
        T1 = (s_px - r_px) / r_dx
        x, y = r_px + r_dx * T1, r_py + r_dy * T1
        T2 = (y - s_py) / s_dy

    elif r_dx == 0:
        T2 = (r_px - s_px) / s_dx
        x, y = s_px + s_dx * T2, s_py + s_dy * T2
        T1 = (y - r_py) / r_dy

    elif r_dy == 0:
        T2 = (r_py - s_py) / s_dy
        x, y = s_px + s_dx * T2, s_py + s_dy * T2
        T1 = (x - r_px) / r_dx
    elif (r_dx / r_dy) == (
            s_dx / s_dy):  # This case is needed to by pass same slopes that do not give "ZERODIVISIONERROR"
        # We cn do a y = mx + b and find if b's are equal all around
        if r_py - (r_dy / r_dx) * r_px == s_py - (s_dy / s_dx) * s_px: return 0, 0  # Touching (Prob gonna give errors)
        return "parallel", "_"
    else:
        T2 = (r_dx * (s_py - r_py) + r_dy * (r_px - s_px)) / (s_dx * r_dy - s_dy * r_dx)
        T1 = (s_px + s_dx * T2 - r_px) / r_dx
        x, y = r_px + r_dx * T1, r_py + r_dy * T1

    # Check to see to make sure that the ray is actually shooting towards and not opposite of walls
    if abs(s_px + s_dx * T2 - x) > .000000001 or abs(s_py + s_dy * T2 - y) > .000000001: return "no", "collision"
    if abs(r_px + r_dx * T1 - x) > .000000001 or abs(r_py + r_dy * T1 - y) > .000000001:
        return "no", "collision"
    # Make sure Ray (T1) goes in assigned direction (Doesn't go left while being shot right) and make sure that T2 is within the wall boundaries
    elif -0.000001 <= T1 and -0.000001 <= T2 <= 1.000001:
        return T1, T2  # Slightly adjusted for minor calculation errors
    # Base-case break
    else:
        return "no", "collision"



pygame.init()
W, H = 700, 525
x, y = W / 2, H / 2
x, y = 274, 200
screen = pygame.display.set_mode((W, H))
clock = pygame.time.Clock()

segments = [[1, 1, 100, 100]
    , [50, 50, 100, 25]
    , [35, 41, 81, 125]
    , [282, 34, 547, 155]
    , [100, 100, 200, 100], [100, 100, 100, 200], [100, 200, 200, 200], [200, 200, 200, 100]
    , [0,0,W,0], [0,0,0,H],[0,H,W,H],[W,0,W,H]]

# Find best case t1
def check_t1(r_px, r_py, r_dx, r_dy, segs):
    bestcase_t1 = 10 ** 4
    related_wall = ""
    for seg in segs:
        s_px, s_py, s_px2, s_py2 = seg
        s_dx, s_dy = s_px2 - s_px, s_py2 - s_py
        t1, t2 = intersection(r_px, r_py, r_dx, r_dy, s_px, s_py, s_dx, s_dy)
        if t1 + t2 != "nocollision" and [t1, t2] != ["parallel", "_"]:
            bestcase_t1 = min(t1, bestcase_t1)
            related_wall = seg

    return bestcase_t1, related_wall


flag = True  # Used to monitor adding new line segs.
pos = []
while True:
    screen.fill((255, 255, 255))


    angles = []
    for i in range(len(segments)):
        x1, y1, x2, y2 = segments[i]
        dx1, dy1 = x1 - x, y1 - y
        dx2, dy2 = x2 - x, y2 - y
        agl1, agl2 = angle(dx1, dy1), angle(dx2, dy2)
        angles += [agl1, agl2]
    angles.sort() # Sort all angles



    for agl in angles:

        # Clock-wise check
        r_dx, r_dy = find_slope(agl + 0.01)
        t1, t2 = check_t1(x, y, r_dx, r_dy, segments)
        pygame.draw.aaline(screen, (250, 128, 114), (x, y), (x + r_dx * t1, y + r_dy * t1))

        # Counter clock-wise check
        r_dx, r_dy = find_slope(agl - 0.01)
        t1, t2 = check_t1(x, y, r_dx, r_dy, segments)
        pygame.draw.aaline(screen, (250, 128, 114), (x, y), (x + r_dx * t1, y + r_dy * t1))

    # Draw all walls
    for seg in segments:
        pygame.draw.aaline(screen, (0, 0, 0), seg[:2], seg[2:])

    # Check inputs
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        if event.type == pygame.MOUSEBUTTONUP:
            mx, my = pygame.mouse.get_pos()
            pos += [mx, my]
            if flag:
                flag = False
            else:
                segments.append(pos)
                print(pos, "new", x, y)
                pos = []
                flag = True

    # Move player
    keys = pygame.key.get_pressed()
    if keys[pygame.K_RIGHT]: x += 2.001  # NOTE THAT VALUES ARE ADJUSTED TO IGNORE CASE WHERE PLAYER I TOUCHING A SEG
    if keys[pygame.K_LEFT]: x -= 2.001
    if keys[pygame.K_DOWN]: y += 2.001
    if keys[pygame.K_UP]:  y -= 2.001

    pygame.display.update()
    clock.tick(70)  # Fps (Don't know why/how it does it)
