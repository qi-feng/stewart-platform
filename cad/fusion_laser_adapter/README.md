# Stewart-platform laser adapter generator

This Fusion 360 script modifies a selected solid copy of
`stewart-platform_adapter` and leaves the source assembly unchanged.

It creates:

- a 10.6 mm central laser passage;
- a 14 mm underside cable-relief pocket;
- a 30 mm OD × 4 mm reinforcing boss;
- a separate 10.4 mm-bore split collar for a 10 mm laser module.

## Run in Fusion

1. Open `stewart_assy.STEP` in Fusion's Design workspace.
2. Open **Utilities → Add-Ins → Scripts and Add-Ins**.
3. Select the **Scripts** tab and click the green **+** beside *My Scripts*.
4. Select the `fusion_laser_adapter` folder itself, not an individual `.py` file.
5. Run `fusion_laser_adapter`.
6. When prompted, select the solid body for `stewart-platform_adapter`.
7. Hide the original assembly and inspect the two new components.

The clamp is a first-fit collar intended for a small hose clamp or zip tie. Do
not print the full adapter until the center cut, rib reinforcement, cable relief,
and exact laser fit have been inspected in Fusion.

Fusion uses centimetres internally in its API; all user-facing dimensions above
are millimetres.
