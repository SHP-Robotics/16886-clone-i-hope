package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;



@Disabled
public class GyroFunctions extends BaseRobot{
    public DcMotor leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor, armMotor1, armMotor2;
    public Servo claw_servo, base_servo_right, base_servo_left;
    public ColorSensor color_sensor;
    public DistanceSensor distance_sensor;
    public ElapsedTime timer = new ElapsedTime();

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;

    @Override
    public void init() {
        super.init();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!imu.isGyroCalibrated())
        {}
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public boolean auto_drive(double power, double inches) {

        double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * (inches);

        telemetry.addData("Target_enc: ", TARGET_ENC);

        correction = checkDirection();

        double leftSpeed = power-correction;
        double rightSpeed = power+correction;

        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            return true;
        } else {
            leftSpeed = Range.clip(leftSpeed, -1, 1);
            leftFrontDriveMotor.setPower(leftSpeed);
            leftBackDriveMotor.setPower(leftSpeed);
            rightFrontDriveMotor.setPower(leftSpeed);
            rightBackDriveMotor.setPower(leftSpeed);
            return false;
        }
    }



    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
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

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .005;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }



    public void rotate(int degrees, double power)
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
        /*
        leftFrontDriveMotor.setPower(leftPower);
        leftBackDriveMotor.setPower(leftPower);
        rightBackDriveMotor.setPower(rightPower);
        rightFrontDriveMotor.setPower(rightPower);
         */

        // rotate until turn is completed.
        if (degrees < 0)
        {
            if (getAngle()==0 || getAngle()>degrees) {
                leftFrontDriveMotor.setPower(leftPower);
                leftBackDriveMotor.setPower(leftPower);
                rightBackDriveMotor.setPower(rightPower);
                rightFrontDriveMotor.setPower(rightPower);
            } else {
                leftFrontDriveMotor.setPower(0);
                leftBackDriveMotor.setPower(0);
                rightBackDriveMotor.setPower(0);
                rightFrontDriveMotor.setPower(0);
                //sleep(1000);
                resetAngle();
            }
            // On right turn we have to get off zero first.
            //while (opModeIsActive() && getAngle() == 0) {}

            //while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            if (getAngle()<degrees) {
                leftFrontDriveMotor.setPower(leftPower);
                leftBackDriveMotor.setPower(leftPower);
                rightBackDriveMotor.setPower(rightPower);
                rightFrontDriveMotor.setPower(rightPower);
            } else {
                leftFrontDriveMotor.setPower(0);
                leftBackDriveMotor.setPower(0);
                rightBackDriveMotor.setPower(0);
                rightFrontDriveMotor.setPower(0);
                //sleep(1000);
                resetAngle();
            }
        //while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        /*
        leftFrontDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);

         */

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();
    }
}
