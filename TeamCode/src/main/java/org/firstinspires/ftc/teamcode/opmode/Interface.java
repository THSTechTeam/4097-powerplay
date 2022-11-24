package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Interface {
    private static abstract class GamepadControllerBase {
        protected Gamepad gamepad;
        protected Gamepad previous;

        protected GamepadControllerBase() {
            this.gamepad  = new Gamepad();
            this.previous = new Gamepad();
        }

        // Must be called at the beginning of each while opModeIsActive() loop.
        protected void update(Gamepad gamepad) {
            try {
                this.previous.copy(this.gamepad);
                this.gamepad.copy(gamepad);
            } catch (RobotCoreException e) {
                // Swallow exception, gamepad[1/2] should always be valid unless unplugged.
            }
        }
    }

    public enum GamepadButton {
        A,
        B,
        X,
        Y,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON,
        GUIDE,
        START,
        BACK,
        LEFT_TRIGGER,
        RIGHT_TRIGGER,
        LEFT_STICK_X,
        LEFT_STICK_Y,
        RIGHT_STICK_X,
        RIGHT_STICK_Y
    }

    public static class GamepadController extends GamepadControllerBase {
        public double getLeftStickX() {
            return gamepad.left_stick_x;
        }

        public double getLeftStickY() {
            return gamepad.left_stick_y;
        }

        public double getRightStickX() {
            return gamepad.right_stick_x;
        }

        public double getRightStickY() {
            return gamepad.right_stick_y;
        }

        public boolean isPressed(GamepadButton gamepadButton) {
            switch (gamepadButton) {
                case A:
                    return gamepad.a && !previous.a;
                case B:
                    return gamepad.b && !previous.b;
                case X:
                    return gamepad.x && !previous.x;
                case Y:
                    return gamepad.y && !previous.y;
                case DPAD_UP:
                    return gamepad.dpad_up && !previous.dpad_up;
                case DPAD_DOWN:
                    return gamepad.dpad_down && !previous.dpad_down;
                case DPAD_LEFT:
                    return gamepad.dpad_left && !previous.dpad_left;
                case DPAD_RIGHT:
                    return gamepad.dpad_right && !previous.dpad_right;
                case LEFT_BUMPER:
                    return gamepad.left_bumper && !previous.left_bumper;
                case RIGHT_BUMPER:
                    return gamepad.right_bumper && !previous.right_bumper;
                case LEFT_STICK_BUTTON:
                    return gamepad.left_stick_button && !previous.left_stick_button;
                case RIGHT_STICK_BUTTON:
                    return gamepad.right_stick_button && !previous.right_stick_button;
                case GUIDE:
                    return gamepad.guide && !previous.guide;
                case START:
                    return gamepad.start && !previous.start;
                case BACK:
                    return gamepad.back && !previous.back;
                case LEFT_TRIGGER:
                    return gamepad.left_trigger > 0.0 && previous.left_trigger == 0.0;
                case RIGHT_TRIGGER:
                    return gamepad.right_trigger > 0.0 && previous.right_trigger == 0.0;
                case LEFT_STICK_X:
                    return gamepad.left_stick_x != previous.left_stick_x;
                case LEFT_STICK_Y:
                    return gamepad.left_stick_y != previous.left_stick_y;
                case RIGHT_STICK_X:
                    return gamepad.right_stick_x != previous.right_stick_x;
                case RIGHT_STICK_Y:
                    return gamepad.right_stick_y != previous.right_stick_y;
                default:
                    return false;
            }
        }

        public boolean isHeld(GamepadButton gamepadButton) {
            switch (gamepadButton) {
                case A:
                    return gamepad.a;
                case B:
                    return gamepad.b;
                case X:
                    return gamepad.x;
                case Y:
                    return gamepad.y;
                case DPAD_UP:
                    return gamepad.dpad_up;
                case DPAD_DOWN:
                    return gamepad.dpad_down;
                case DPAD_LEFT:
                    return gamepad.dpad_left;
                case DPAD_RIGHT:
                    return gamepad.dpad_right;
                case LEFT_BUMPER:
                    return gamepad.left_bumper;
                case RIGHT_BUMPER:
                    return gamepad.right_bumper;
                case LEFT_STICK_BUTTON:
                    return gamepad.left_stick_button;
                case RIGHT_STICK_BUTTON:
                    return gamepad.right_stick_button;
                case GUIDE:
                    return gamepad.guide;
                case START:
                    return gamepad.start;
                case BACK:
                    return gamepad.back;
                case LEFT_TRIGGER:
                    return gamepad.left_trigger > 0.0;
                case RIGHT_TRIGGER:
                    return gamepad.right_trigger > 0.0;
                case LEFT_STICK_X:
                    return gamepad.left_stick_x != 0.0;
                case LEFT_STICK_Y:
                    return gamepad.left_stick_y != 0.0;
                case RIGHT_STICK_X:
                    return gamepad.right_stick_x != 0.0;
                case RIGHT_STICK_Y:
                    return gamepad.right_stick_y != 0.0;
                default:
                    return false;
            }
        }
    }
}
