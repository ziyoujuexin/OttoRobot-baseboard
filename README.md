## Otto Robot - Baseboard Project

This project is the firmware for the baseboard of an Otto-like bipedal robot, based on the ESP32 platform.

### Servo Layout and Configuration

Below is the current servo connection layout and known issues. The channel numbers are critical for the motion control logic.

| Part Name      | Channel (L/R) | Angle Range        | Notes                                      |
|----------------|---------------|--------------------|--------------------------------------------|
| Ear (Front/Back) | 1 / 3         | 0-180              | Right ear angle needs adjustment.          |
| Ear (Up/Down)  | 0 / 2         | 0-180              | Left ear is completely stuck (stalling).   |
| Head (Up/Down) | 4             | 80-140             | -                                          |
| Head (Left/Right)| 5             | 0-180              | -                                          |
| Arm (Front/Back) | 6 / 8         | 0-180              | Left arm is reversed. Right arm gets stuck.|
| Arm (Up/Down)  | 7 / 9         | 80-120 / 60-100    | Left arm is reversed. Note opposite limits.|
| Leg (Rotation) | 10 / 12       | 60-150 / 30-120    | Feet are parallel when angles are complementary. |
| Ankle (Lift)   | 11 / 13       | 50-120             | Center of gravity foot ~100, lifted foot ~120. |

