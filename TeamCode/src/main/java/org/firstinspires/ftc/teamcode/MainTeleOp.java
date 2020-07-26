package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp

public class MainTeleOp extends BaseRobot {
    private boolean slow_mode;
    @Override
    public void init() {
        super.init();
        gamepad1.setJoystickDeadzone(0.001f);
        color_sensor.enableLed(false);
        slow_mode = false;
    }

    @Override
    public void start() {
        super.start();
    }
    @Override
    public void loop() {

        super.loop();

        if (gamepad1.dpad_up) {
            slow_mode = false;
        } else if (gamepad1.dpad_down) {
            slow_mode = true;
        }

        //drive train
        if (slow_mode) {
            tankanum_drive(0.3*gamepad1.right_stick_y, 0.3*gamepad1.left_stick_y, 0.3*gamepad1.right_stick_x);
        } else {
            tankanum_drive(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        //arm
        if (gamepad1.a) {
            arm1 (1);
        } else if (gamepad1.x) {
            arm1 (-1);
        } else {
            arm1(0);
        }
        if (gamepad1.y) {
            arm2 (-1);
        } else if (gamepad1.b) {
            arm2 (1);
        } else {
            arm2(0);
        }

        //base mover
        if(gamepad1.left_bumper) {
            base_mover(ConstantVariables.K_BASE_SERVO_RIGHT_DOWN, ConstantVariables.K_BASE_SERVO_LEFT_DOWN);
        } else if (gamepad1.right_bumper){
            base_mover(ConstantVariables.K_BASE_SERVO_RIGHT_UP, ConstantVariables.K_BASE_SERVO_LEFT_UP);
        }

        //claw
        if (gamepad1.left_trigger>0.00) {
            claw(ConstantVariables.K_CLAW_SERVO_OPEN);
        } else if (gamepad1.right_trigger>0.00) {
            claw(ConstantVariables.K_CLAW_SERVO_CLOSED);
        }
    }
}
