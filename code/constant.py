

# ==============================================================================
# ---- class Color ----
# encoded as RGB
# ==============================================================================
class Color:
    BLACK  = (0, 0, 0)
    WHITE  = (255, 255, 255)
    GREEN  = (0, 255, 0)
    RED    = (255, 0, 0)
    BLUE   = (0, 0, 255)
    YELLOW = (255, 255, 0)
    FOREST_GREEN = (34, 139, 34)
    CRIMSON = (220, 20, 60)
    MAGENTA = (255, 0, 255)
    SGI_GRAY_52 = (132, 132, 132)
    ORANGE = (255, 128, 0)
    TURQUOISE = (64, 224, 208)
    SILVER = (192, 192, 192)


class Const:
    TOTAL_LENGTH_OF_5_SEG_ROAD = 1500  # total length of road [m] corresponds to 5 segments
    length = TOTAL_LENGTH_OF_5_SEG_ROAD
    # spec of speed limits on road: each tuple: (from [m], to [m], speed [km/hr])
    SPEED_SPEC1 = [(0, length / 2, 120),  # for the 1st half of the road the legal speed limit in 120 km/hr
                   (length / 2, length * 3 / 4, 120),
                   (length * 3 / 4, length, 120)]

    SPEED_SPEC2 = [(0, length / 2, 100),
                   (length / 2, length * 3 / 4, 80),
                   (length * 3 / 4, length, 100)]

    USED_SPEED_SPEC = SPEED_SPEC2
