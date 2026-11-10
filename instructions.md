The enclosed document @FinalTask contains eight variants of the final course. The course will be located on a table with a 8cm high edge and consists of four areas separated by wooden beams.

Implement a program so that the robot completes the course as follows:

1. In the bottom area, the robot counts the white lines and follows the black line to the branch that corresponds to the number of lines counted, thus finding the passage to the second area.
2. In the second area, the robot follows the black line and drives around the first can without touching it. Then it follows the line to the second can and touches it (causing the figure on it to fall). Finally, the robot follows the line to the passage to the third area.
3. In the third area, the robot follows the black line to a cardboard box (light grey shaded area) and pushes it sideways so that the robot can move through the passage to the top area.
4. In the top area, there is a small flashlight somewhere on the edge of the table and a small bell in front of it. The robot moves to the end of the line, searches for the flashlight and moves as close to it as possible until the bell rings.

Please note that the robot must not touch the edge of the table or the beams.

As a reference, the distance between the inner points of the bottle circles is 37 cm.
The distance from the start to the junction after the bottles is roughly 110 cm

We did a calibration:
- with a speed of 100, it moved 23 cm straight per second.
- With a speed of 90, it took rougly 650 ms to turn 90 degrees.