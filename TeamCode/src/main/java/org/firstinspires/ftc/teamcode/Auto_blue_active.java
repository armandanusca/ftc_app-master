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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal examp
 * le of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto_blue_active", group="PDRobot")  // @Auto(...) is the other common choice

public class Auto_blue_active extends LinearOpMode {


    HardwarePD robot = new HardwarePD();
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.loadServo.setPosition(0.5);
        waitForStart();
        runtime.reset();
        robot.loadServo.setPosition(0.5);
        robot.hitBeacon.setPosition(Servo.MAX_POSITION);
        sleep(500);
        robot.hitBeacon.setPosition(Servo.MIN_POSITION);
        sleep(500);
        robot.hitBeacon.setPosition(Servo.MAX_POSITION);
        sleep(500);
        ///robot.Color.enableLed(false);
        Move_Forward(1,800);
        Throw_Ball();
        Move_Forward(0.4,600);
        Turn_Right();
        Move_Forward(1,750);
        Hit_Beacon(1);
        Move_Forward(0.4,400);
        Turn_Right();
        Hit_Beacon(1);
        Move_Backward(0.7,800);
        Turn_Right();
        Move_Forward(1,750);
        //Move_Forward(0.4,400);
    }

    public void Move_Forward(double pw,long dur){
        robot.leftMotor.setPower(pw);
        robot.rightMotor.setPower(pw);
        sleep(dur);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void Move_Backward(double pw,long dur){
        robot.leftMotor.setPower(-pw);
        robot.rightMotor.setPower(-pw);
        sleep(dur);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void Turn_Right(){
        robot.leftMotor.setPower(1);
        sleep(975);
        robot.leftMotor.setPower(0);
    }

    public void Turn_Left(){
        robot.rightMotor.setPower(1);
        sleep(975);
        robot.rightMotor.setPower(0);
    }

    public void Hit_Beacon(int clr){
        telemetry.addData("Albastru: ",robot.Color.blue());
        telemetry.addData("Rosu: ",robot.Color.red());
        telemetry.update();
        if(clr==1){
            if(robot.Color.blue()>robot.Color.red())
                robot.hitBeacon.setPosition(Servo.MIN_POSITION);
            sleep(100);
            Move_Forward(1,300);
        }
        else{
            if(robot.Color.red()>robot.Color.blue())
                robot.hitBeacon.setPosition(Servo.MIN_POSITION);
            sleep(100);
            Move_Forward(1,300);
        }
    }

    public void Throw_Ball(){
        robot.loadServo.setPosition(0.5);
        robot.throwMotor.setPower(1);
        sleep(1500);
        robot.loadServo.setPosition(0.1);
        sleep(1000);
        robot.loadServo.setPosition(0.5);
        sleep(1000);
        robot.loadServo.setPosition(0.1);
        sleep(1000);
        robot.loadServo.setPosition(0.5);
        sleep(1000);
        robot.throwMotor.setPower(0);
    }
}



