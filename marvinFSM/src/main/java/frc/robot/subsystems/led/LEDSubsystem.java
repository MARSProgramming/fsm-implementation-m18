package frc.robot.subsystems.led;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Field;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
    private enum AnimationType {
        STROBE,
        FADE,
        FIRE,
        LARSON,
        TWINKLE,
        RAINBOW
    }

    private static final int NUMBER_OF_LEDS = 58;

    public enum WantedState {
        DISPLAY_OFF,
        DISPLAY_ROBOT_ELEVATOR_NOT_ZEROED,
        DISPLAY_ROBOT_ELEVATOR_ZEROED,
        DISPLAY_POSE_RESET,
        DISPLAY_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH,
        DISPLAY_ALIGN_READY,
        DISPLAY_ROBOT_ZERO_ACTION,
        DISPLAY_READY_FOR_MATCH,
        DISPLAY_INTAKING_ALGAE,
        DISPLAY_HOLDING_ALGAE,
        DISPLAY_ALIGN_TO_PROCESSOR,
        DISPLAY_SCORE_PROCESSOR,
        DISPLAY_HOLDING_CORAL,
        DISPLAY_TAG_NOT_SEEN,
        DISPLAY_ALIGN_TO_TARGET_L_ONE,
        DISPLAY_ALIGN_TO_TARGET_L_TWO,
        DISPLAY_ALIGN_TO_TARGET_L_THREE,
        DISPLAY_ALIGN_TO_TARGET_L_FOUR,
        DISPLAY_CAN_SCORE_L_ONE,
        DISPLAY_CAN_SCORE_L_TWO,
        DISPLAY_CAN_SCORE_L_THREE,
        DISPLAY_CAN_SCORE_L_FOUR,
        DISPLAY_CLIMBING,
        DISPLAY_CONTROLLERS_ACTIVE
    }

    private enum SystemState {
        DISPLAYING_OFF,
        DISPLAYING_ROBOT_ELEVATOR_NOT_ZEROED,
        DISPLAYING_ROBOT_ELEVATOR_ZEROED,
        DISPLAYING_POSE_RESET,
        DISPLAYING_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH,
        DISPLAYING_ALIGN_READY,
        DISPLAYING_ROBOT_ZERO_ACTION,
        DISPLAYING_READY_FOR_MATCH,
        DISPLAYING_INTAKING_ALGAE,
        DISPLAYING_HOLDING_ALGAE,
        DISPLAYING_ALIGN_TO_PROCESSOR,
        DISPLAYING_SCORE_PROCESSOR,
        DISPLAYING_HOLDING_CORAL,
        DISPLAYING_TAG_NOT_SEEN,
        DISPLAYING_ALIGN_TO_TARGET_L_ONE,
        DISPLAYING_ALIGN_TO_TARGET_L_TWO,
        DISPLAYING_ALIGN_TO_TARGET_L_THREE,
        DISPLAYING_ALIGN_TO_TARGET_L_FOUR,
        DISPLAYING_CAN_SCORE_L_ONE,
        DISPLAYING_CAN_SCORE_L_TWO,
        DISPLAYING_CAN_SCORE_L_THREE,
        DISPLAYING_CAN_SCORE_L_FOUR,
        DISPLAYING_CLIMBING,
        DISPLAYING_CONTROLLERS_ACTIVE
    }

    private WantedState wantedAction = WantedState.DISPLAY_OFF;
    private final LEDIO ledIO;

    private SystemState getStateTransition() {
        return switch (wantedAction) {
            case DISPLAY_OFF -> SystemState.DISPLAYING_OFF;
            case DISPLAY_ROBOT_ELEVATOR_NOT_ZEROED -> SystemState.DISPLAYING_ROBOT_ELEVATOR_NOT_ZEROED;
            case DISPLAY_ROBOT_ELEVATOR_ZEROED -> SystemState.DISPLAYING_ROBOT_ELEVATOR_ZEROED;
            case DISPLAY_POSE_RESET -> SystemState.DISPLAYING_POSE_RESET;
            case DISPLAY_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH -> SystemState.DISPLAYING_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH;
            case DISPLAY_ALIGN_READY -> SystemState.DISPLAYING_ALIGN_READY;
            case DISPLAY_ROBOT_ZERO_ACTION -> SystemState.DISPLAYING_ROBOT_ZERO_ACTION;
            case DISPLAY_READY_FOR_MATCH -> SystemState.DISPLAYING_READY_FOR_MATCH;
            case DISPLAY_INTAKING_ALGAE -> SystemState.DISPLAYING_INTAKING_ALGAE;
            case DISPLAY_HOLDING_ALGAE -> SystemState.DISPLAYING_HOLDING_ALGAE;
            case DISPLAY_ALIGN_TO_PROCESSOR -> SystemState.DISPLAYING_ALIGN_TO_PROCESSOR;
            case DISPLAY_SCORE_PROCESSOR -> SystemState.DISPLAYING_SCORE_PROCESSOR;
            case DISPLAY_HOLDING_CORAL -> SystemState.DISPLAYING_HOLDING_CORAL;
            case DISPLAY_TAG_NOT_SEEN -> SystemState.DISPLAYING_TAG_NOT_SEEN;
            case DISPLAY_ALIGN_TO_TARGET_L_ONE -> SystemState.DISPLAYING_ALIGN_TO_TARGET_L_ONE;
            case DISPLAY_ALIGN_TO_TARGET_L_TWO -> SystemState.DISPLAYING_ALIGN_TO_TARGET_L_TWO;
            case DISPLAY_ALIGN_TO_TARGET_L_THREE -> SystemState.DISPLAYING_ALIGN_TO_TARGET_L_THREE;
            case DISPLAY_ALIGN_TO_TARGET_L_FOUR -> SystemState.DISPLAYING_ALIGN_TO_TARGET_L_FOUR;
            case DISPLAY_CAN_SCORE_L_ONE -> SystemState.DISPLAYING_CAN_SCORE_L_ONE;
            case DISPLAY_CAN_SCORE_L_TWO -> SystemState.DISPLAYING_CAN_SCORE_L_TWO;
            case DISPLAY_CAN_SCORE_L_THREE -> SystemState.DISPLAYING_CAN_SCORE_L_THREE;
            case DISPLAY_CAN_SCORE_L_FOUR -> SystemState.DISPLAYING_CAN_SCORE_L_FOUR;
            case DISPLAY_CLIMBING -> SystemState.DISPLAYING_CLIMBING;
            case DISPLAY_CONTROLLERS_ACTIVE -> SystemState.DISPLAYING_CONTROLLERS_ACTIVE;

        };
    }

    public void setWantedAction(WantedState wantedAction) {
        this.wantedAction = wantedAction;
    }

    public LEDSubsystem(LEDIO ledIO) {
        this.ledIO = ledIO;
    }

    @Override
    public void periodic() {

        Logger.recordOutput("Subsystems/LED/WantedState", wantedAction);

        switch (getStateTransition()) {
            case DISPLAYING_READY_FOR_MATCH:
                ledIO.setAnimation(new LarsonAnimation(
                        0, 255, 0, 0, 0.5, NUMBER_OF_LEDS - 30, LarsonAnimation.BounceMode.Center, 7));
                break;
            case DISPLAYING_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH:
                if (Field.isBlueAlliance()) {
                    ledIO.setAnimation(new LarsonAnimation(
                            0, 0, 255, 0, 0.3, NUMBER_OF_LEDS - 30, LarsonAnimation.BounceMode.Back, 7));
                } else {
                    ledIO.setAnimation(new LarsonAnimation(
                            255, 0, 0, 0, 0.3, NUMBER_OF_LEDS - 30, LarsonAnimation.BounceMode.Back, 7));
                }
                break;
            case DISPLAYING_POSE_RESET:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 255, 255, 0.9));
                break;
            case DISPLAYING_ROBOT_ELEVATOR_ZEROED:
                ledIO.clearAnimation();
                ledIO.setLEDs(0, 255, 0);
                break;
            case DISPLAYING_ROBOT_ELEVATOR_NOT_ZEROED:
                ledIO.clearAnimation();
                ledIO.setLEDs(255, 30, 0);
                break;
            case DISPLAYING_ROBOT_ZERO_ACTION:
                ledIO.setAnimation(getAnimation(AnimationType.LARSON, 0, 100, 200, 0.9));
                break;
            case DISPLAYING_OFF:
                ledIO.clearAnimation();
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_INTAKING_ALGAE:
                ledIO.setLEDs(66, 245, 233);
                break;
            case DISPLAYING_HOLDING_ALGAE:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 66, 245, 233, 0.9));
                break;
            case DISPLAYING_ALIGN_TO_PROCESSOR:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 0, 25, 0.9));
                break;
            case DISPLAYING_SCORE_PROCESSOR:
                ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 0, 25, 0.8));
                break;
            case DISPLAYING_HOLDING_CORAL:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 0, 255, 0.95));
                break;
            case DISPLAYING_ALIGN_TO_TARGET_L_ONE, DISPLAYING_CAN_SCORE_L_ONE:
                ledIO.clearAnimation();
                ledIO.setLEDs(113, 0, 242);
                break;
            case DISPLAYING_ALIGN_TO_TARGET_L_TWO:
                // ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 194, 151, 0.9));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_ALIGN_TO_TARGET_L_THREE:
                // ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 131, 246, 0.9));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_ALIGN_TO_TARGET_L_FOUR:
                // ledIO.setAnimation(getAnimation(AnimationType.FADE, 255, 0, 237, 0.9));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_TAG_NOT_SEEN:
                ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 0, 0, 0.75));
                break;
            case DISPLAYING_CAN_SCORE_L_TWO:
                // ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 194, 151, 0.75));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_CAN_SCORE_L_THREE:
                // ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 131, 246, 0.75));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_CAN_SCORE_L_FOUR:
                // ledIO.setAnimation(getAnimation(AnimationType.STROBE, 255, 0, 237, 0.75));
                ledIO.setLEDs(0, 0, 0);
                break;
            case DISPLAYING_CLIMBING:
                ledIO.setAnimation(getAnimation(AnimationType.FADE, 40, 255, 0, 0.9));
                break;
            case DISPLAYING_CONTROLLERS_ACTIVE:
                ledIO.setAnimation(getAnimation(AnimationType.RAINBOW, 2, 9, 10, 0.95));
                break;
            default:
                System.out.println("Fell through on LED commands");
                break;
        }
    }

    private static Animation getAnimation(AnimationType type, int red, int green, int blue, double speed) {
        return switch (type) {
            default -> new StrobeAnimation(red, green, blue, 0, speed, NUMBER_OF_LEDS);
            case FADE -> new SingleFadeAnimation(red, green, blue, 0, speed, NUMBER_OF_LEDS);
            case FIRE -> new FireAnimation(1.0, speed, NUMBER_OF_LEDS, 0.2, 0.2);
            case LARSON -> new LarsonAnimation(
                    red, green, blue, 0, speed, NUMBER_OF_LEDS, LarsonAnimation.BounceMode.Back, 5);
            case TWINKLE -> new TwinkleAnimation(
                    red, green, blue, 0, speed, NUMBER_OF_LEDS, TwinkleAnimation.TwinklePercent.Percent88);
            case RAINBOW -> new RainbowAnimation(1, speed, NUMBER_OF_LEDS, false, 0);
        };
    }
}