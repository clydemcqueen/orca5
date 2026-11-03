# Buttons

## Xbox Button IDs

The Linux driver ids are mapped by SDL_GameControllerButton, which is compiled into QGC.

| SDL (BTN) | Linux driver | XBox button |
| :---: | :---: | :--- |
| 0 | 0 | A |
| 1 | 1 | B |
| 2 | 2 | X |
| 3 | 3 | Y |
| 4 | 6 | View (Back) |
| 5 | 8 | Guide (Xbox) |
| 6 | 7 | Menu (Start) |
| 7 | 9 | Left stick click (LS) |
| 8 | 10 | Right stick click (RS) |
| 9 | 4 | Left bumper (LB) |
| 10 | 5 | Right bumper (RB) |
| 11 | 11 | D-pad Up |
| 12 | 12 | D-pad Down |
| 13 | 13 | D-pad Left |
| 14 | 14 | D-pad Right |

## Seattle Aquarium Assignments

These are the typical mode / action button assignments:
~~~
BTN0_FUNCTION    1    # A                   shift
BTN0_SFUNCTION   1    # shift A             shift
BTN1_FUNCTION    5    # B                   MANUAL
BTN1_SFUNCTION   8    # shift B             POSHOLD
BTN2_FUNCTION    7    # X                   ALT_HOLD
BTN2_SFUNCTION   11   # shift X             GUIDED
BTN3_FUNCTION    6    # Y                   STABILIZE
BTN3_SFUNCTION   13   # shift Y             SURFTRAK
~~~

These script button assignments were used by surftrak.lua, and so we just kept them for use with newer scripts to avoid confusion.
~~~
BTN11_SFUNCTION  109  # shift-dpad-up       script_2
BTN12_SFUNCTION  110  # shift-dpad-down     script_3
BTN13_SFUNCTION  111  # shift-dpad-left     script_4
BTN14_SFUNCTION  108  # shift-dpad-right    script_1
~~~
