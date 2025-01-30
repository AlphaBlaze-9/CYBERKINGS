#include "vex.h"

void stopDurAuto() {
    left_motor_1.stop();
    left_motor_2.stop();
    left_motor_3.stop();
    right_motor_1.stop();
    right_motor_2.stop();
    right_motor_3.stop();
}


void turn90(std::string direction) {
    wait(0.5, seconds);

    if (direction == "left") {
        left_motor_1.spin(reverse, 80, percent);
        left_motor_2.spin(reverse, 80, percent);
        left_motor_3.spin(reverse, 80, percent);

        right_motor_1.spin(forward, 50, percent);
        right_motor_2.spin(forward, 50, percent);
        right_motor_3.spin(forward, 50, percent);
    }
    else if (direction == "right") {
        left_motor_1.spin(forward, 50, percent);
        left_motor_2.spin(forward, 50, percent);
        left_motor_3.spin(forward, 50, percent);

        right_motor_1.spin(reverse, 80, percent);
        right_motor_2.spin(reverse, 80, percent);
        right_motor_3.spin(reverse, 80, percent);
    }
    
    wait(0.305, seconds);
    stopDurAuto();
}

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */





void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

/**
 * Autonomous routine for Blue Left corner.
 */
void BlueLeft() {
  clamp.set(false);
  Brain.Screen.clearScreen();
  Brain.Screen.print("Autonomous Mode");

  // Drive forward
  left_motor_1.spin(forward, 50, percent);
  left_motor_2.spin(forward, 50, percent);
  left_motor_3.spin(forward, 50, percent);
  right_motor_1.spin(forward, 50, percent);
  right_motor_2.spin(forward, 50, percent);
  right_motor_3.spin(forward, 50, percent);
  wait(1, seconds);

  // Stop all motors
  left_motor_1.stop();
  left_motor_2.stop();
  left_motor_3.stop();
  right_motor_1.stop();
  right_motor_2.stop();
  right_motor_3.stop();

  // Clamp onto the Mobile Goal
  clamp.set(true);
  wait(1, seconds);

  Stage1.setVelocity(100, percent);
  belt.setVelocity(100, percent);
  Stage1.spin(forward);
  belt.spin(forward);
  wait(1, seconds);

  // Turn -40 degrees
  left_motor_1.spin(forward, 80, percent);
  left_motor_2.spin(forward, 80, percent);
  left_motor_3.spin(forward, 80, percent);
  right_motor_1.spin(reverse, 50, percent);
  right_motor_2.spin(reverse, 50, percent);
  right_motor_3.spin(reverse, 50, percent);
  wait(0.2, seconds);

  stopDurAuto();
  left_motor_1.spin(reverse, 50, percent);
  left_motor_2.spin(reverse, 50, percent);
  left_motor_3.spin(reverse, 50, percent);
  right_motor_1.spin(reverse, 50, percent);
  right_motor_2.spin(reverse, 50, percent);
  right_motor_3.spin(reverse, 50, percent);
  wait(0.75, seconds);

  turn_90("left");
  stopDurAuto();

  left_motor_1.spin(reverse, 50, percent);
  left_motor_2.spin(reverse, 50, percent);
  left_motor_3.spin(reverse, 50, percent);
  right_motor_1.spin(reverse, 50, percent);
  right_motor_2.spin(reverse, 50, percent);
  right_motor_3.spin(reverse, 50, percent);
  turn_90("right");
}


void BlueRight() {
    clamp.set(false);
    Brain.Screen.clearScreen();
    Brain.Screen.print("Autonomous Mode");

    // Drive forward for 1 second
    left_motor_1.spin(forward, 50, percent);
    left_motor_2.spin(forward, 50, percent);
    left_motor_3.spin(forward, 50, percent);
    right_motor_1.spin(forward, 50, percent);
    right_motor_2.spin(forward, 50, percent);
    right_motor_3.spin(forward, 50, percent);
    wait(1, seconds);

    // Stop all motors
    left_motor_1.stop();
    left_motor_2.stop();
    left_motor_3.stop();
    right_motor_1.stop();
    right_motor_2.stop();
    right_motor_3.stop();

    // Clamp onto the Mobile Goal
    clamp.set(true);
    wait(1, seconds);

    Stage1.setVelocity(100, percent);
    belt.setVelocity(100, percent);
    Stage1.spin(forward);
    belt.spin(forward);
    wait(1, seconds);

    // Turn -40 degrees
    left_motor_1.spin(forward, 80, percent);
    left_motor_2.spin(forward, 80, percent);
    left_motor_3.spin(forward, 80, percent);
    right_motor_1.spin(reverse, 50, percent);
    right_motor_2.spin(reverse, 50, percent);
    right_motor_3.spin(reverse, 50, percent);
    wait(0.2, seconds);

    stopDurAuto();

    // Run into the stack to score the bottom ring
    left_motor_1.spin(reverse, 50, percent);
    left_motor_2.spin(reverse, 50, percent);
    left_motor_3.spin(reverse, 50, percent);
    right_motor_1.spin(reverse, 50, percent);
    right_motor_2.spin(reverse, 50, percent);
    right_motor_3.spin(reverse, 50, percent);
    wait(0.75, seconds);

    stopDurAuto();
    turn_90("right");

    // Move backward slightly
    left_motor_1.spin(reverse, 25, percent);
    left_motor_2.spin(reverse, 25, percent);
    left_motor_3.spin(reverse, 25, percent);
    right_motor_1.spin(reverse, 25, percent);
    right_motor_2.spin(reverse, 25, percent);
    right_motor_3.spin(reverse, 25, percent);
    wait(1, seconds);

    // Stop all motors
    left_motor_1.stop();
    left_motor_2.stop();
    left_motor_3.stop();
    right_motor_1.stop();
    right_motor_2.stop();
    right_motor_3.stop();

    // Move forward slightly
    left_motor_1.spin(forward, 50, percent);
    left_motor_2.spin(forward, 50, percent);
    left_motor_3.spin(forward, 50, percent);
    right_motor_1.spin(forward, 50, percent);
    right_motor_2.spin(forward, 50, percent);
    right_motor_3.spin(forward, 50, percent);
    wait(0.4, seconds);

    stopDurAuto();

    // Turn
    left_motor_1.spin(forward, 50, percent);
    left_motor_2.spin(forward, 50, percent);
    left_motor_3.spin(forward, 50, percent);
    right_motor_1.spin(reverse, 80, percent);
    right_motor_2.spin(reverse, 80, percent);
    right_motor_3.spin(reverse, 80, percent);
    wait(0.4, seconds);

    stopDurAuto();

    // Move backward slightly
    left_motor_1.spin(reverse, 50, percent);
    left_motor_2.spin(reverse, 50, percent);
    left_motor_3.spin(reverse, 50, percent);
    right_motor_1.spin(reverse, 50, percent);
    right_motor_2.spin(reverse, 50, percent);
    right_motor_3.spin(reverse, 50, percent);
    wait(0.4, seconds);

    stopDurAuto();
}




void RedLeft() {
    lift.spinFor(forward, 0.5, turns, false);
    clamp.set(false);
    Brain.Screen.clearScreen();
    Brain.Screen.print("Autonomous Mode");

    // Drive forward
    left_motor_1.spin(forward, 50, percent);
    left_motor_2.spin(forward, 50, percent);
    left_motor_3.spin(forward, 50, percent);
    right_motor_1.spin(forward, 50, percent);
    right_motor_2.spin(forward, 50, percent);
    right_motor_3.spin(forward, 50, percent);
    wait(1, seconds);

    // Stop all motors
    left_motor_1.stop();
    left_motor_2.stop();
    left_motor_3.stop();
    right_motor_1.stop();
    right_motor_2.stop();
    right_motor_3.stop();

    // Clamp onto the Mobile Goal
    clamp.set(true);
    wait(1, seconds);

    Stage1.setVelocity(100, percent);
    belt.setVelocity(100, percent);
    Stage1.spin(forward);
    belt.spin(forward);
    wait(1, seconds);

    // Turn -40 degrees
    left_motor_1.spin(reverse, 80, percent);
    left_motor_2.spin(reverse, 80, percent);
    left_motor_3.spin(reverse, 80, percent);
    right_motor_1.spin(forward, 50, percent);
    right_motor_2.spin(forward, 50, percent);
    right_motor_3.spin(forward, 50, percent);
    wait(0.2, seconds);

    stopDurAuto();

    // Run into the stack
    left_motor_1.spin(reverse, 50, percent);
    left_motor_2.spin(reverse, 50, percent);
    left_motor_3.spin(reverse, 50, percent);
    right_motor_1.spin(reverse, 50, percent);
    right_motor_2.spin(reverse, 50, percent);
    right_motor_3.spin(reverse, 50, percent);
    wait(0.75, seconds);
    stopDurAuto();

    turn_90("left");

    // Reverse slightly
    left_motor_1.spin(reverse, 25, percent);
    left_motor_2.spin(reverse, 25, percent);
    left_motor_3.spin(reverse, 25, percent);
    right_motor_1.spin(reverse, 25, percent);
    right_motor_2.spin(reverse, 25, percent);
    right_motor_3.spin(reverse, 25, percent);
    wait(1, seconds);

    // Stop all motors
    left_motor_1.stop();
    left_motor_2.stop();
    left_motor_3.stop();
    right_motor_1.stop();
    right_motor_2.stop();
    right_motor_3.stop();

    // Move forward
    left_motor_1.spin(forward, 50, percent);
    left_motor_2.spin(forward, 50, percent);
    left_motor_3.spin(forward, 50, percent);
    right_motor_1.spin(forward, 50, percent);
    right_motor_2.spin(forward, 50, percent);
    right_motor_3.spin(forward, 50, percent);
    wait(0.4, seconds);
    stopDurAuto();

    // Slight turn
    left_motor_1.spin(reverse, 50, percent);
    left_motor_2.spin(reverse, 50, percent);
    left_motor_3.spin(reverse, 50, percent);
    right_motor_1.spin(forward, 80, percent);
    right_motor_2.spin(forward, 80, percent);
    right_motor_3.spin(forward, 80, percent);
    wait(0.35, seconds);
    stopDurAuto();

    // Reverse
    left_motor_1.spin(reverse, 50, percent);
    left_motor_2.spin(reverse, 50, percent);
    left_motor_3.spin(reverse, 50, percent);
    right_motor_1.spin(reverse, 50, percent);
    right_motor_2.spin(reverse, 50, percent);
    right_motor_3.spin(reverse, 50, percent);
    wait(0.9, seconds);
    stopDurAuto();
}






void RedRight() {
  clamp.set(false);
  Brain.Screen.clearScreen();
  Brain.Screen.print("Autonomous Mode");

  // Drive forward for 2 seconds
  left_motor_1.spin(forward, 50, percent);
  left_motor_2.spin(forward, 50, percent);
  left_motor_3.spin(forward, 50, percent);
  right_motor_1.spin(forward, 50, percent);
  right_motor_2.spin(forward, 50, percent);
  right_motor_3.spin(forward, 50, percent);
  wait(1, seconds);

  // Stop all motors
  left_motor_1.stop();
  left_motor_2.stop();
  left_motor_3.stop();
  right_motor_1.stop();
  right_motor_2.stop();
  right_motor_3.stop();

  // Clamp onto the Mobile Goal
  clamp.set(true);
  wait(1, seconds);

  Stage1.setVelocity(100, percent);
  belt.setVelocity(100, percent);
  Stage1.spin(forward);
  belt.spin(forward);
  wait(1, seconds);

  // Turn negative 40 degrees
  left_motor_1.spin(reverse, 80, percent);
  left_motor_2.spin(reverse, 80, percent);
  left_motor_3.spin(reverse, 80, percent);
  right_motor_1.spin(forward, 50, percent);
  right_motor_2.spin(forward, 50, percent);
  right_motor_3.spin(forward, 50, percent);
  wait(0.2, seconds);

  stopDurAuto();

  // Run into the stack to score the bottom ring
  left_motor_1.spin(reverse, 50, percent);
  left_motor_2.spin(reverse, 50, percent);
  left_motor_3.spin(reverse, 50, percent);
  right_motor_1.spin(reverse, 50, percent);
  right_motor_2.spin(reverse, 50, percent);
  right_motor_3.spin(reverse, 50, percent);
  wait(0.75, seconds);

  stopDurAuto();

  turn_90("left");

  // Operate the clamp (close)
  left_motor_1.spin(reverse, 25, percent);
  left_motor_2.spin(reverse, 25, percent);
  left_motor_3.spin(reverse, 25, percent);
  right_motor_1.spin(reverse, 25, percent);
  right_motor_2.spin(reverse, 25, percent);
  right_motor_3.spin(reverse, 25, percent);
  wait(1, seconds);

  // Stop all motors
  left_motor_1.stop();
  left_motor_2.stop();
  left_motor_3.stop();
  right_motor_1.stop();
  right_motor_2.stop();
  right_motor_3.stop();

  left_motor_1.spin(forward, 50, percent);
  left_motor_2.spin(forward, 50, percent);
  left_motor_3.spin(forward, 50, percent);
  right_motor_1.spin(forward, 50, percent);
  right_motor_2.spin(forward, 50, percent);
  right_motor_3.spin(forward, 50, percent);
  wait(0.4, seconds);

  stopDurAuto();

  left_motor_1.spin(reverse, 50, percent);
  left_motor_2.spin(reverse, 50, percent);
  left_motor_3.spin(reverse, 50, percent);
  right_motor_1.spin(forward, 80, percent);
  right_motor_2.spin(forward, 80, percent);
  right_motor_3.spin(forward, 80, percent);
  wait(0.2, seconds);

  stopDurAuto();

  left_motor_1.spin(reverse, 50, percent);
  left_motor_2.spin(reverse, 50, percent);
  left_motor_3.spin(reverse, 50, percent);
  right_motor_1.spin(reverse, 50, percent);
  right_motor_2.spin(reverse, 50, percent);
  right_motor_3.spin(reverse, 50, percent);
  wait(0.75, seconds);

  stopDurAuto();
}
