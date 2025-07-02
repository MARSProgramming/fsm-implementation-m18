package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.constants.Constants;

public class Dash {
    ShuffleboardTab dashboard;

    private SendableChooser<Constants.SuperstructureConstants.AutomationLevel> automationLevelSendableChooser =
            new SendableChooser<>();

    public Dash() {
        dashboard = Shuffleboard.getTab(Constants.operatorDashboardName);

        automationLevelSendableChooser.setDefaultOption(
                "Auto Release", Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE);
        automationLevelSendableChooser.addOption(
                "Auto Drive", Constants.SuperstructureConstants.AutomationLevel.AUTO_DRIVE_AND_MANUAL_RELEASE);

        dashboard
                .add("Automation Level", automationLevelSendableChooser)
                .withSize(6, 3)
                .withPosition(7, 0)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    public Constants.SuperstructureConstants.AutomationLevel getAutomationLevel() {
        return automationLevelSendableChooser.getSelected();
    }
}