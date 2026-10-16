# Arm parameters (6-DOF EPDobot)

**Kind: Design parameters.** These values are copied from the paper as printed:
the mechanical-parameter figure (Fig. 3), the D-H parameter table (Table 2) and the
joint-actuation table (Table 5). They are not measurements. A machine-readable copy
is in [`dh_parameters.csv`](dh_parameters.csv).

## D-H parameters (paper Table 2)

Table caption: "EPDobot D-H Parameters (units m/rad; Q16.16 fixed-point; all joints
revolute)". The paper uses the standard D-H convention, with
`T(i-1,i) = Rot_z(theta_i) Trans_z(d_i) Trans_x(a_i) Rot_x(alpha_i)`.

| Joint i | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| d_i (m) | 0.166 | 0 | 0 | 0.192 | 0 | 0.055 |
| a_i (m) | 0.055 | 0.200 | 0.056 | 0 | 0 | 0 |
| alpha_i (rad) | pi/2 | 0 | pi/2 | -pi/2 | pi/2 | 0 |

**theta offset:** the paper does not print one. In the code of this repository,
theta_i is the joint angle q_i written by opcode 2, with no added offset
(`tools/fk_golden.py`; the RTL constants are in `rtl/impedance/part1_config.v`). The Q16.16 constants used
there, `d = [10879, 0, 0, 12583, 0, 3604]`, `a = [3604, 13107, 3670, 0, 0, 0]` and
`alpha = [102944, 0, 102944, -102944, 102944, 0]`, are the table values rounded to
Q16.16. This sentence describes the code; it is not printed in the paper.

## Mechanical-parameter figure (paper Fig. 3)

Caption: "Mechanical parameters of the 6-DOF EPDobot arm." The figure is a
side-view drawing with the reach envelope. These dimension labels are printed
on it:

| Label in the figure | Value as printed |
|---|---|
| base height | 166 |
| shoulder offset | 55 |
| upper-arm length | 200 |
| elbow offset | 56 |
| forearm length | 192 |
| wrist-to-flange length | 55 |
| envelope: horizontal reach | 358 |
| envelope: overall height | 699 |
| envelope: rear extent | -198 |
| envelope: lowest point | -99 |

The figure prints no unit. The six link dimensions equal the D-H lengths above in
millimetres (166, 55, 200, 56, 192, 55 mm vs 0.166, 0.055, 0.200, 0.056, 0.192,
0.055 m), so the figure is in mm. The descriptive names in the left column are
added here; the figure shows only the numbers.

## Joint actuation (paper Table 5)

| Joint | Motor frame | Gear ratio | Driver |
|---|---|---|---|
| J1 | NEMA-17, L48 | 25:1 | own CAN driver board |
| J2 | NEMA-17, L60 | 20:1 | own CAN driver board |
| J3 | NEMA-17, L34 | 25:1 | own CAN driver board |
| J4 | NEMA-17, L34 | 10:1 | own CAN driver board |
| J5, J6 | NEMA-17, L40 | direct drive | own CAN driver board |

Table note in the paper: "Each joint is a brushless DC motor driven by its own CAN
driver board." In the text: "The six joints are driven by NEMA-17 brushless DC
motors behind planetary gearheads". The joint markers J1–J6 appear in Fig. 6(a);
the unannotated photos are in `board_test/photos/`.
