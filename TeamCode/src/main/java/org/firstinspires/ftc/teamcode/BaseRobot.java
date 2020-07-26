/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.Math;


import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class BaseRobot extends OpMode {


    public DcMotor leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor, armMotor1, armMotor2;
    public Servo claw_servo, base_servo_right, base_servo_left;
    public ColorSensor color_sensor;
    public DistanceSensor distance_sensor;
    public ElapsedTime timer = new ElapsedTime();

    public double startLight;

    /*
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;
     */


    public void init() {
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBackDriveMotor");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBackDriveMotor");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");

        armMotor1 = hardwareMap.get(DcMotor.class, "armLiftMotor");
        armMotor2 = hardwareMap.get(DcMotor.class, "armLiftMotor2");

        base_servo_right = hardwareMap.get(Servo.class, "platformMoveRight_Servo");
        base_servo_left = hardwareMap.get(Servo.class, "platformMoveLeft_Servo");

        claw_servo = hardwareMap.get(Servo.class, "armClamp_Servo");

        color_sensor = hardwareMap.get(ColorSensor.class, "sensorColor");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "sensorColor");

        //imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        claw_servo.setDirection(Servo.Direction.REVERSE);

        base_mover(ConstantVariables.K_BASE_SERVO_RIGHT_UP, ConstantVariables.K_BASE_SERVO_LEFT_UP);
        claw(ConstantVariables.K_CLAW_SERVO_CLOSED);

        color_sensor.enableLed(true);
        startLight=color_sensor.alpha();


        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".


        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
         */

    }

    public void start() {
        timer.reset();
        reset_drive_encoders();
        reset_arm_encoders();
    }

    public void loop() {
        /*
        telemetry.addData("D00 Left Front Drive Motor Enc: ", get_left_front_drive_motor_enc());
        telemetry.addData("D01 Right Front Drive Motor Enc: ", get_right_front_drive_motor_enc());
        telemetry.addData("D02 Left Back Drive Motor Enc: ", get_left_back_drive_motor_enc());
        telemetry.addData("D03 Right Back Drive Motor Enc: ", get_right_back_drive_motor_enc());
        telemetry.addData("D04 Arm Motor 1 Enc: ", get_arm_motor_1_enc());
        telemetry.addData("D05 Arm Motor 2 Enc: ", get_arm_motor_2_enc());
        telemetry.addData("D06 Base Servo Right Pos: ", base_servo_right.getPosition());
        telemetry.addData("D07 Base Servo Left Pos: ", base_servo_left.getPosition());
        telemetry.addData("D06 Claw Servo Pos: ", claw_servo.getPosition());

        telemetry.addData("Blue", color_sensor.blue());
        telemetry.addData("Red", color_sensor.red());
        telemetry.addData("Light: ", color_sensor.alpha());
        telemetry.addData("startLight/blue", startLight/color_sensor.blue());
        */
        telemetry.addData("Stone Detected: ", detect_stone());
        telemetry.addData("line detected: ", detect_line());
        //telemetry.addData("blue/red: ", (double) color_sensor.blue()/(double) color_sensor.red());
        telemetry.addData("distance (in): ", distance_sensor.getDistance(DistanceUnit.INCH));
        /*
        telemetry.addData("1 imu Z heading", getAngle());
        telemetry.addData("imu Z heading", lastAngles.firstAngle);
        telemetry.addData("imu Y heading", lastAngles.secondAngle);
        telemetry.addData("imu X heading", lastAngles.thirdAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();
         */

    }

    public boolean auto_drive(double power, double inches) {

        double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * (inches);

        telemetry.addData("Target_enc: ", TARGET_ENC);

        //correction = checkDirection();

        double speed = -power;

        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            return true;
        } else {
            speed = Range.clip(speed, -1, 1);
            leftFrontDriveMotor.setPower(speed);
            leftBackDriveMotor.setPower(speed);
            rightFrontDriveMotor.setPower(speed);
            rightBackDriveMotor.setPower(speed);
            return false;
        }
    }

    /**
     * @param power:   the speed to turn at. Negative for left.
     * @param degrees: the number of degrees to turn.
     * @return Whether the target angle has been reached.
     */

    public boolean auto_turn(double power, double degrees) {
        double TARGET_ENC = Math.abs(ConstantVariables.K_PPDEG_DRIVE * degrees);
        telemetry.addData("D99 TURNING TO ENC: ", TARGET_ENC);

        double speed = Range.clip(power, -1, 1);
        leftFrontDriveMotor.setPower(-speed);
        leftBackDriveMotor.setPower(-speed);
        rightFrontDriveMotor.setPower(speed);
        rightBackDriveMotor.setPower(speed);

        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            return true;
        } else {
            return false;
        }
    }

    public boolean auto_mecanum(double power, double inches) {
        double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * (inches);
        telemetry.addData("Target_enc: ", TARGET_ENC);

        //correction = checkDirection();

        double leftFrontPower = Range.clip(0 - power, -1.0, 1.0);
        double leftBackPower = Range.clip(0 + power, -1.0, 1.0);
        double rightFrontPower = Range.clip(0 + power, -1.0, 1.0);
        double rightBackPower = Range.clip(0 - power, -1.0, 1.0);

        /**
        double error = -get_left_front_drive_motor_enc() - get_right_back_drive_motor_enc();
        error /= ConstantVariables.K_DRIVE_ERROR_P;
        telemetry.addData("error: ", error);
        telemetry.update();

        leftBackPower -= error;
        rightBackPower -= error;
        rightFrontPower += error;
        leftFrontPower += error;
        */

        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            return true;
        } else {
            leftFrontDriveMotor.setPower(leftFrontPower);
            leftBackDriveMotor.setPower(leftBackPower);
            rightFrontDriveMotor.setPower(rightFrontPower);
            rightBackDriveMotor.setPower(rightBackPower);
            return false;
        }
    }

    public void tankanum_drive(double rightPwr, double leftPwr, double lateralpwr) {
        //leftPwr *= -1;

        double leftFrontPower = Range.clip(leftPwr - lateralpwr, -1.0, 1.0);
        double leftBackPower = Range.clip(leftPwr + lateralpwr, -1.0, 1.0);
        double rightFrontPower = Range.clip(rightPwr + lateralpwr, -1.0, 1.0);
        double rightBackPower = Range.clip(rightPwr - lateralpwr, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);

    }
    public void slow_tele_right (){
        double leftFrontPower = Range.clip(-ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double leftBackPower = Range.clip(ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double rightFrontPower = Range.clip(ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double rightBackPower = Range.clip(-ConstantVariables.K_SLOW_POWER, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);
    }

    public void slow_tele_left (){
        double leftFrontPower = Range.clip(ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double leftBackPower = Range.clip(-ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double rightFrontPower = Range.clip(-ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double rightBackPower = Range.clip(ConstantVariables.K_SLOW_POWER, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);
    }

    public void slow_tele_back (){
        double leftFrontPower = Range.clip(ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double leftBackPower = Range.clip(ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double rightFrontPower = Range.clip(ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double rightBackPower = Range.clip(ConstantVariables.K_SLOW_POWER, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);
    }

    public void slow_tele_front (){
        double leftFrontPower = Range.clip(-ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double leftBackPower = Range.clip(-ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double rightFrontPower = Range.clip(-ConstantVariables.K_SLOW_POWER, -1.0, 1.0);
        double rightBackPower = Range.clip(-ConstantVariables.K_SLOW_POWER, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);
    }

    public void arm1(double power) {
        double speed = Range.clip(power, -1, 1);
        armMotor1.setPower(speed);
    }
    public void arm2(double power) {
        double speed = Range.clip(power, -1, 1);
        armMotor2.setPower(speed);
    }

    public boolean auto_arm1(double power, double inches) {
        double TARGET_ENC = ConstantVariables.K_PPIN_ARM * inches /3.8;

        armMotor1.setPower(-power);
        if(Math.abs(get_arm_motor_1_enc())>=TARGET_ENC) {
            armMotor1.setPower(0);
            return true;
        } else {
            return false;
        }
    }

    public boolean auto_arm2(double power, double inches) {
        double TARGET_ENC = ConstantVariables.K_PPIN_ARM * inches / 3.8;

        armMotor2.setPower(-power);
        if(Math.abs(get_arm_motor_2_enc())>=TARGET_ENC) {
            armMotor2.setPower(0);
            return true;
        } else {
            return false;
        }
    }

    public void base_mover(double rightPos, double leftPos) {
        double leftPosition = Range.clip(leftPos, 0, 1.0);
        double rightPosition = Range.clip(rightPos, 0, 1.0);
        base_servo_right.setPosition(rightPosition);
        base_servo_left.setPosition(leftPosition);
    }

    public void claw(double pos) {
        double position = Range.clip(pos, 0, 1.0);
        claw_servo.setPosition(position);
    }

    public boolean detect_stone() {
        if (3.0/4.0 * (double) color_sensor.red()<(double) color_sensor.blue()) {
            return false;
        } else {
            return true;
        }
    }

    public boolean detect_line() {
        if (distance_sensor.getDistance(DistanceUnit.INCH)<ConstantVariables.K_STONE_DETECTION_DISTANCE) {
            return true;
        } else {
            return false;
        }
    }

    public void reset_drive_encoders() {
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reset_arm_encoders() {
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int get_left_front_drive_motor_enc() {
        if (leftFrontDriveMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return leftFrontDriveMotor.getCurrentPosition();
    }

    public int get_right_front_drive_motor_enc() {
        if (rightFrontDriveMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return rightFrontDriveMotor.getCurrentPosition();
    }

    public int get_left_back_drive_motor_enc() {
        if (leftBackDriveMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return leftBackDriveMotor.getCurrentPosition();
    }

    public int get_right_back_drive_motor_enc() {
        if (rightBackDriveMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return rightBackDriveMotor.getCurrentPosition();
    }

    public int get_arm_motor_1_enc() {
        if (armMotor1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return armMotor1.getCurrentPosition();
    }

    public int get_arm_motor_2_enc() {
        if (armMotor2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return armMotor2.getCurrentPosition();
    }



    /*
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    /*
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    */

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */

    /*
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */

    /*
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftFrontDriveMotor.setPower(leftPower);
        leftBackDriveMotor.setPower(leftPower);
        rightBackDriveMotor.setPower(rightPower);
        rightFrontDriveMotor.setPower(leftPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        leftFrontDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);

        // wait for rotation to stop.

        // reset angle tracking on new heading.
        resetAngle();
    }
    */


}