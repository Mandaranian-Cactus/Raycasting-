import pygame
import math
import sys


# The general idea of the program is to set up an optimzied raycasting alhorithim
# Time complexity: O(2(2n * n))  where n is the # of walls
# Pseudo-code
# walls = input()
# agls  = []
# for wall in walls:
#     split wall into 2 endpoints
#     agl1, agl2 = angles of endpoints (Relative to the origin)
#     append agl1 and agl2 into agls
#
# for wall in walls:
#     intercepts = calculate all intersections between this wall and all other walls
#     for intercept in intercepts:
#          agl = angle of point (Relative to the origin)
#          agls.append(agl)
#
# sort agls (Increasing order)
#
# find closest_t1 with a vertical ray
# set prev intercept to be associated with closest t1
#
# for agl in agls:
#     find closest t1 (Rotated slightly clockwise)
#     find cloest_t1 (Rotated slightly counter clockwise)
#
#     find clockwise intercept using clockwise t1
#     find counter clockwise intercept using counter clockwise t1
#
#     make triangle from prev_intercept to center to counter clockwise intercept
#     prev intercept = clockwise intercept


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
    dx, dy = dx, dy
    if dx == 0 and dy == 0:
        return 0  # Error since it seems that the line is actually a point
    elif dx == 0:
        if dy > 0:
            return 180
        else:
            return 0
    elif dy == 0:
        if dx > 0:
            return 90
        else:
            return 270
    else:
        if dx > 0:
            if dy > 0:
                agl = 90 + math.degrees(math.atan(abs(dy / dx)))
            elif dy < 0:
                agl = 90 - math.degrees(math.atan(abs(dy / dx)))
        elif dx < 0:
            if dy > 0:
                agl = 270 - math.degrees(math.atan(abs(dy / dx)))
            elif dy < 0:
                agl = 270 + math.degrees(math.atan(abs(dy / dx)))

    return agl


def seg_intersection(r_px, r_py, r_dx, r_dy, s_px, s_py, s_dx,
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

    # Make sure Ray (T1) doesn't go opposite to assigned direction (Does down while being shot up) and make sure that T2 is within the wall boundaries
    elif -0.000001 <= T1 <= 1.000001 and -0.000001 <= T2 <= 1.000001:  # Slightly adjusted for calculation imperfections
        return T1, T2
    else:
        return "no", "collision"


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

    # Make sure Ray (T1) goes in assigned direction (Doesn't go down while being shot up) and
    # make sure that wall (T2) is within the wall boundaries
    elif -0.000001 <= T1 and -0.000001 <= T2 <= 1.000001:  # Slightly adjusted for minor calculation errors
        return T1, T2
    # Base-case break
    else:
        return "no", "collision"


def point_adjust(x, y, agl, clockwise=True):
    agl = agl % 360  # Used for debugging, not sure if necessary
    interval = 0.1

    if clockwise:
        if 0 <= agl < 90:
            x += interval
            y += interval
        elif 90 <= agl < 180:
            x -= interval
            y += interval
        elif 180 <= agl < 270:
            y -= interval
            x -= interval
        else:
            x += interval
            y -= interval
    else:
        if 0 <= agl < 90:
            x -= interval
            y -= interval
        elif 90 <= agl < 180:
            x += interval
            y -= interval
        elif 180 <= agl < 270:
            y += interval
            x += interval
        else:
            x -= interval
            y += interval

    return x, y


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


def load_intercepts(segments, x, y):  # Finds all intercepts between walls
    ints = set()

    for i in range(len(segments)):
        s_px1, s_py1, s_px3, s_py3 = segments[i]
        s_dx1, s_dy1 = s_px3 - s_px1, s_py3 - s_py1
        for j in range(i + 1, len(segments)):
            s_px2, s_py2, s_px4, s_py4 = segments[j]  # Wrong, we have points, no displacement
            s_dx2, s_dy2 = s_px4 - s_px2, s_py4 - s_py2
            t1, t2 = seg_intersection(s_px1, s_py1, s_dx1, s_dy1, s_px2, s_py2, s_dx2, s_dy2)
            if t1 + t2 != "nocollision" and [t1, t2] != ["parallel", "_"]:
                x_int, y_int = s_px1 + s_dx1 * t1, s_py1 + s_dy1 * t1
                ints.add(tuple([x_int, y_int, x_int + 0.001, y_int + 0.001]))

    # print("ints", ints)
    return ints


def border_check(x, y, w, h):
    if 0 <= x <= w and 0 <= y <= h:
        return True
    else:
        if x > w:
            x = w
        elif x < 0:
            x = 0

        if y > h:
            y = h
        elif y < 0:
            y = 0

        return [x, y]


pygame.init()
W, H = 700, 525
r_px, r_py = 334.3212258796822, 153.52926901248583
screen = pygame.display.set_mode((W, H))
clock = pygame.time.Clock()

segments = [[1, 1, 100, 100]
    , [50, 50, 100, 25]
    , [35, 41, 81, 125]
    , [282, 34, 547, 155]
    , [100, 100, 200, 100], [100, 100, 100, 200], [100, 200, 200, 200], [200, 200, 200, 100]
    , [0, 0, W, 0], [0, 0, 0, H], [0, H, W, H], [W, 0, W, H]
    , [233, 137, 345, 142], [450, 450, 330, 130]]

ints = load_intercepts(segments, r_px, r_py)
for intercept in ints:
    segments.append(intercept)

flag = True  # Used to monitor adding new line segments.
pos = []
while True:
    screen.fill((255, 255, 255))
    angles = set()

    for i in range(len(segments)):
        x1, y1, x2, y2 = segments[i]
        dx1, dy1 = x1 - r_px, y1 - r_py
        dx2, dy2 = x2 - r_px, y2 - r_py
        agl1, agl2 = angle(dx1, dy1), angle(dx2, dy2)
        angles.add(agl1)
        angles.add(agl2)
    angles = list(angles)
    angles.sort()  # Sort all angles

    # Initial check (Vertical ray)
    r_dx, r_dy = 0, -1
    t1, related_wall = check_t1(r_px, r_py, r_dx, r_dy, segments)
    cw_int_x, cw_int_y = r_px + r_dx * t1, r_py + r_dy * t1
    origin_int_x, origin_int_y = cw_int_x, cw_int_y

    for agl in angles:
        # Counter clock-wise check
        r_dx, r_dy = find_slope(agl - 0.01)
        t1, related_wall = check_t1(r_px, r_py, r_dx, r_dy, segments)
        t1 = round(t1, 2)  # Rounded to help out with smoothness
        x_int, y_int = r_px + r_dx * t1, r_py + r_dy * t1
        x_int, y_int = point_adjust(x_int, y_int, agl, True)
        pygame.draw.polygon(screen, (250, 126, 100), ([r_px, r_py], [x_int, y_int], [cw_int_x, cw_int_y]))

        # Clock-wise check
        r_dx, r_dy = find_slope(agl + 0.01)
        t1, related_wall = check_t1(r_px, r_py, r_dx, r_dy, segments)
        t1 = round(t1, 2)  # Rounded to help out with smoothness
        x_int, y_int = r_px + r_dx * t1, r_py + r_dy * t1
        x_int, y_int = point_adjust(x_int, y_int, agl, False)

        cw_int_x, cw_int_y = x_int, y_int

    # Connect final polygon from prev intercept back to 1st intercept
    pygame.draw.polygon(screen, (250, 126, 100), ([r_px, r_py], [origin_int_x, origin_int_y], [cw_int_x, cw_int_y]))

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

                if pos[:2] == pos[2:]:
                    # Player has inputed point and not wall
                    # Information of the point will be deleted
                    print("Denied (Point Error)")
                else:
                    print(pos, "new", r_px, r_py)

                    # Calculate any intercepts between new wall and any pre-existing wall
                    for seg in segments[:]:
                        x1, y1, x2, y2 = seg
                        dx1, dy1 = x2 - x1, y2 - y1
                        x3, y3, x4, y4 = pos
                        dx3, dy3 = x4 - x3, y4 - y3
                        t1, t2 = seg_intersection(x1, y1, dx1, dy1, x3, y3, dx3, dy3)
                        if t1 + t2 != "nocollision" and [t1, t2] != ["parallel", "_"]:
                            x_int, y_int = x1 + dx1 * t1, y1 + dy1 * t1
                            segments.append([x_int, y_int, x_int + 0.001, y_int + 0.001])

                    segments.append(pos)  # Add new wall

                pos = []
                flag = True

    # Move player
    keys = pygame.key.get_pressed()
    if keys[pygame.K_RIGHT]: r_px += 2.001  # NOTE THAT VALUES ARE ADJUSTED TO IGNORE CASE WHERE PLAYER I TOUCHING A SEG
    if keys[pygame.K_LEFT]: r_px -= 2.001
    if keys[pygame.K_DOWN]: r_py += 2.001
    if keys[pygame.K_UP]:  r_py -= 2.001

    pygame.draw.rect(screen, (5, 10, 60), (r_px - 5, r_py - 5, 10, 10))
    # print(r_px, r_py)

    # Border check
    result = border_check(r_px, r_py, W, H)
    if result != True:
        r_px, r_py = result

    pygame.display.update()
    clock.tick(67)  # Fps (Don't know why/how it does it)
