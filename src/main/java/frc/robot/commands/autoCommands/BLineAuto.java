
package frc.robot.commands.autoCommands;

import java.util.List;

import frc.robot.lib.BLine.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class BLineAuto extends AutoCommandInterface {

    private Pose2d m_initPose = null;
    private final CommandSwerveDrivetrain m_driveTrain;
    
    public BLineAuto(List<Object> pathSteps, CommandSwerveDrivetrain driveTrain, boolean isOutpostSide) {
        m_driveTrain = driveTrain;

        for (Object pathName : pathSteps) {

            Path path;
            try {
                path = new Path((String) pathName);
            } catch (Exception e) {
                DriverStation.reportError("Unable to load BLine path: " + (String) pathName, true);
                m_initPose = new Pose2d();
                return;
            }

            if (isOutpostSide) {
                // changes path in place
                path.mirror();
            }
            if (m_initPose == null) {
                m_initPose = path.getStartPose();
                System.out.println("InitPose = (" + m_initPose.getX() + ", " + m_initPose.getY() + ")");
            }

            addCommands(m_driveTrain.followPath(path));
        }
    }
    
    @Override
    public Pose2d getInitialPose() {
        System.out.println("Get InitPose = (" + m_initPose.getX() + ", " + m_initPose.getY() + ")");
        return FieldConstants.flipPose(m_initPose);
    }
}
