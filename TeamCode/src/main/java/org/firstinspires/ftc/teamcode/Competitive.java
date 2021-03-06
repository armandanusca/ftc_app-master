/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Competitive", group="PDRobot")  // @Auto(...) is the other common choice

public class Competitive extends OpMode
{

  HardwarePD robot = new HardwarePD(); // use the class created to define a Pushbot's hardware
  private ElapsedTime runtime = new ElapsedTime();

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {

    robot.init(hardwareMap);
    telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
    // leftMotor  = hardwareMap.dcMotor.get("left_drive");
    // rightMotor = hardwareMap.dcMotor.get("right_drive");

    // eg: Set the drive motor directions:
    // Reverse the motor that runs backwards when connected directly to the battery
    // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
    //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
    // telemetry.addData("Status", "Initialized");
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
      robot.loadServo.setPosition(0.5);
      robot.releaseServo.setPosition(Servo.MAX_POSITION);
      robot.fixServo.setPosition(Servo.MAX_POSITION);
    runtime.reset();
  }
    int beacon=0;
  @Override
  public void loop() {
    int adj=0,adc=0,load_did=0;
    telemetry.addData("Status", "Running: " + runtime.toString());
    robot.fixServo.setPosition(gamepad1.left_trigger);
    telemetry.addData("GP1", gamepad1.toString());
    telemetry.addData("GP2", gamepad2.toString());
    telemetry.update();
      robot.leftMotor.setPower(-gamepad1.left_stick_y);
      robot.rightMotor.setPower(-gamepad1.right_stick_y);
    if(gamepad1.right_bumper)
      load_did = (load_did + 1) % 2;
    if(load_did==1) robot.loadServo.setPosition(0.1);
    else robot.loadServo.setPosition(0.5);
    if(gamepad1.x) adc=(adc+1)%2;
    if(gamepad1.a) adj=(adj+1)%2;
      if(gamepad1.dpad_up) robot.liftMotor.setPower(1);
      if(gamepad1.dpad_down) robot.liftMotor.setPower(-1);
      if(!gamepad1.dpad_up&&!gamepad1.dpad_up) robot.liftMotor.setPower(0);
      if(gamepad1.left_bumper) robot.releaseServo.setPosition(Servo.MIN_POSITION);
    if(adc==1) robot.catchMotor.setPower(1);
    else robot.catchMotor.setPower(0);
    if(adj==1) robot.throwMotor.setPower(1);
    else robot.throwMotor.setPower(0);
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
  }

}
