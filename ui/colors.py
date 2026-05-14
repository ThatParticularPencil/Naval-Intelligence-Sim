
# --- anchors (RGB) -------------------------------------------------------------
STONE = (0x99, 0x9B, 0x7E) 
SAND = (0xA9, 0x9B, 0x8E)  # A99B8E — rock / taupe
MIST = (0xCA, 0xC2, 0xBD)  # CAC2BD — mist / light UI text
SEA_MINT = (0x95, 0xBF, 0xB8)  # 95BFB8 — highlights, predictions
SEA_MID = (0x60, 0xA6, 0x9F)  # 60A69F — mid teal, accents
SEA_DEEP = (0x3F, 0x8C, 0x8C)  # 3F8C8C — deep teal, structure

#Hex: #D97757 or #E68A5C and #D9A036
ORANGE1 = (0xD9, 0x77, 0x57)
ORANGE2 = (0xE6, 0x8A, 0x5C)
GOLD = (0xD9, 0xA0, 0x36)
# --- map / world ---------------------------------------------------------------
BG_OCEAN = (24, 52, 52)  # deep water from SEA_DEEP
WAVE = (24, 70, 70)  # deep water from SEA_DEEP
GRID = (42, 78, 76)  # low-contrast grid on ocean

OBSTACLE = STONE
OBSTACLE_EDGE = MIST

VESSEL = SAND
VESSEL_HEADING = SEA_MINT
SENSOR_RING = STONE

# True contact: warm shift from STONE (still in family, distinct from teal tracks)
# TARGET_TRUE = (0xB5, 0x8A, 0x78)
TARGET_TRUE = ORANGE2
TARGET_PRED = SEA_MINT
OBS_NOISY = (0xD4, 0xE8, 0xE4)  # brightened SEA_MINT for “flash” hits

TRAIL_TRUE = (0x7A, 0x6E, 0x64)  # darkened STONE
TRAIL_PRED = (0x45, 0x78, 0x74)  # darkened SEA_MID

# --- HUD / chrome --------------------------------------------------------------
HUD_BG = (26, 30, 35)
HUD_TEXT = MIST
HUD_MUTED = STONE
ALERT = (0xC4, 0x7A, 0x6E)  # warm alert; harmonizes with STONE / TARGET_TRUE
SCORE_GLOW = (0xFF, 0xD8, 0x4A)
SCORE = (0xFF, 0xF2, 0xA3)

BUTTON_BG = (0x34, 0x5E, 0x5E)
BUTTON_BG_HOVER = (0x42, 0x72, 0x70)
BUTTON_BORDER = SEA_MID
BUTTON_TEXT = MIST
