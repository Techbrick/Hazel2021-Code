#include "subsystems/Subsystems.h"

TrajectoryTimingSubsystem::TrajectoryTimingSubsystem() : frc::Subsystem("TrajectoryTimingSubsystem") {
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint = frc::DifferentialDriveVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
        kDriveKinematics,
        10_V
    );
    
    frc::TrajectoryConfig forwardConfig = frc::TrajectoryConfig(kMaxSpeed, kMaxAcceleration);
    forwardConfig.SetKinematics(kDriveKinematics);
    forwardConfig.AddConstraint(autoVoltageConstraint);

    frc::TrajectoryConfig reverseConfig = frc::TrajectoryConfig(kMaxSpeed, kMaxAcceleration);
    reverseConfig.SetKinematics(kDriveKinematics);
    reverseConfig.AddConstraint(autoVoltageConstraint);
    reverseConfig.SetReversed(true);

    frc::Trajectory tempTraj;
    std::vector<frc::Trajectory> tempList;

    // Begin Barrel Run
    tempTraj = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        // Compressed in Y Barrel Run
        frc::Pose2d(0_m, 0_m, 0_deg),
        {
            frc::Translation2d(3.61450916527799_m, -0.262744281169519_m),
            frc::Translation2d(3.85594985608241_m, -1.03677473110135_m),
            frc::Translation2d(3.3446636873201_m, -1.5_m),
            frc::Translation2d(2.39310331767914_m, -1.32792379942433_m),
            frc::Translation2d(2.15166262687472_m, -0.781131646720194_m),
            frc::Translation2d(2.24397818512347_m, -0.369262232995001_m),
            frc::Translation2d(2.74106196030904_m, -0.085214361460384_m),
            frc::Translation2d(3.23814573549462_m, 0.0923155582487502_m),
            frc::Translation2d(4.01927738221482_m, 0.156226329344039_m),
            frc::Translation2d(4.83591501287684_m, -0.336672473867596_m),
            frc::Translation2d(5.34720118163915_m, -0.393482048174519_m),
            frc::Translation2d(5.95790410543857_m, -0.0810293894864412_m),
            frc::Translation2d(6.01471367974549_m, 0.444459172852599_m),
            frc::Translation2d(5.84428495682472_m, 0.764013028329041_m),
            frc::Translation2d(5.42531434631116_m, 0.998352522345099_m),
            frc::Translation2d(4.78562339039539_m, 0.941542948038176_m),
            frc::Translation2d(4.25_m, 0.352143614603848_m),
            frc::Translation2d(4.5_m, -0.2_m),
            frc::Translation2d(5.92239812149674_m, -0.916054385699136_m),
            frc::Translation2d(6.34846992879867_m, -1.19300106044539_m),
            frc::Translation2d(6.81004772004242_m, -1.41313816088471_m),
            frc::Translation2d(7.4491554309953_m, -1.46284653840327_m),
            frc::Translation2d(7.79001287683684_m, -1.07228071504318_m),
            frc::Translation2d(7.73320330252992_m, -0.596500530222693_m),
            frc::Translation2d(7.41364944705348_m, 0_m),
            frc::Translation2d(6.84555370398424_m, 0.191732313285866_m),
            frc::Translation2d(6.12123163157097_m, 0_m),
            frc::Translation2d(5.3_m, -0.5_m),
            frc::Translation2d(3.47248522951068_m, 0_m),
            frc::Translation2d(2.20847220118164_m, 0_m),
            frc::Translation2d(1.69718603241933_m, 0_m)
        },
        frc::Pose2d(0.198833510074231_m, -0.0994167550371159_m, 180_deg),
        forwardConfig
    );
    tempList.push_back(tempTraj);
    pathChooser.SetDefaultOption("Barrel Run", tempList);

    tempList.clear();

    // End barrel run, begin slalom

    tempTraj = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        // Compressed in Y Barrel Run
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {
            // Slalom
            //frc::Translation2d(0_m, 0_m),
            frc::Translation2d(0.557317073170732_m, 0_m),
            frc::Translation2d(0.86219512195122_m, 0.152439024390244_m),
            frc::Translation2d(1.16707317073171_m, 0.518292682926829_m),
            frc::Translation2d(1.31951219512195_m, 0.76219512195122_m),
            frc::Translation2d(1.4719512195122_m, 1.00609756097561_m),
            frc::Translation2d(1.77682926829268_m, 1.3719512195122_m),
            frc::Translation2d(1.98170731707317_m, 1.52439024390244_m),
            frc::Translation2d(2.69146341463415_m, 1.67682926829268_m),
            frc::Translation2d(3.50609756097561_m, 1.67682926829268_m),
            frc::Translation2d(4.42073170731707_m, 1.67682926829268_m),
            frc::Translation2d(5.03048780487805_m, 1.52439024390244_m),
            frc::Translation2d(5.33536585365854_m, 1.3719512195122_m),
            frc::Translation2d(5.64024390243902_m, 1.00609756097561_m),
            frc::Translation2d(5.79268292682927_m, 0.76219512195122_m),
            frc::Translation2d(5.94512195121951_m, 0.518292682926829_m),
            frc::Translation2d(6.25_m, 0.152439024390244_m),
            frc::Translation2d(6.61585365853658_m, 0_m),
            frc::Translation2d(7.01219512195122_m, 0.152439024390244_m),
            frc::Translation2d(7.56707317073171_m, 0.457317073170732_m),
            frc::Translation2d(7.76951219512195_m, 0.76219512195122_m),
            frc::Translation2d(7.56707317073171_m, 1.06707317073171_m),
            frc::Translation2d(7.01219512195122_m, 1.3719512195122_m),
            frc::Translation2d(6.61585365853658_m, 1.52439024390244_m),
            frc::Translation2d(6.25_m, 1.3719512195122_m),
            frc::Translation2d(5.94512195121951_m, 1.00609756097561_m),
            frc::Translation2d(5.79268292682927_m, 0.76219512195122_m),
            frc::Translation2d(5.64024390243902_m, 0.518292682926829_m),
            frc::Translation2d(5.33536585365854_m, 0.152439024390244_m),
            frc::Translation2d(5.03048780487805_m, 0_m),
            frc::Translation2d(4.42073170731707_m, -0.0914634146341463_m),
            frc::Translation2d(3.50609756097561_m, -0.0914634146341463_m),
            frc::Translation2d(2.59146341463415_m, -0.0914634146341463_m),
            frc::Translation2d(1.98170731707317_m, 0_m),
            frc::Translation2d(1.67682926829268_m, 0.152439024390244_m),
            frc::Translation2d(1.3719512195122_m, 0.518292682926829_m),
            frc::Translation2d(1.21951219512195_m, 0.76219512195122_m),
            frc::Translation2d(1.06707317073171_m, 1.00609756097561_m),
            frc::Translation2d(0.76219512195122_m, 1.3719512195122_m),
            frc::Translation2d(0.457317073170732_m, 1.52439024390244_m),
            frc::Translation2d(0_m, 1.52439024390244_m),

            //frc::Translation2d(0_m, 1.52439024390244_m),
            //frc::Translation2d(-0.152439024390244_m, 1.52439024390244_m),
        },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(-0.152439024390244_m, 1.52439024390244_m, frc::Rotation2d(180_deg)),
        forwardConfig
    );
    tempList.push_back(tempTraj);
    pathChooser.AddOption("Slalom", tempList);

    tempList.clear();

    // End slalom, begin bounce

    tempTraj = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        // Compressed in Y Barrel Run
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {
            // Slalom
            //frc::Translation2d(0_m, 0_m),
            frc::Translation2d(0.733646511627907_m, 0.006993006993007_m),
            frc::Translation2d(1.3_m, 0.275_m)
            //frc::Translation2d(0_m, 1.52439024390244_m),
            //frc::Translation2d(-0.152439024390244_m, 1.52439024390244_m),
        },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(1.46729302325581_m, 0.951048951048951_m, frc::Rotation2d(90_deg)),
        forwardConfig
    );
    tempList.push_back(tempTraj);

    tempTraj = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        // Compressed in Y Barrel Run
        frc::Pose2d(1.46729302325581_m, 0.951048951048951_m, frc::Rotation2d(90_deg)),
        {
            // Slalom
            //frc::Translation2d(0_m, 0_m),
            frc::Translation2d(1.55235348837209_m, 0.356643356643357_m),
            frc::Translation2d(1.76500465116279_m, -0.0839160839160838_m),
            frc::Translation2d(2.15_m, -0.973426573426574_m),
            frc::Translation2d(2.67940465116279_m, -1.18321678321678_m),
            frc::Translation2d(3.18976744186047_m, -1.18321678321678_m),
            frc::Translation2d(3.59380465116279_m, -0.923076923076923_m),
            frc::Translation2d(3.70013023255814_m, -0.335664335664336_m),
            frc::Translation2d(3.66823255813954_m, 0.426573426573427_m)
            //frc::Translation2d(0_m, 1.52439024390244_m),
            //frc::Translation2d(-0.152439024390244_m, 1.52439024390244_m),
        },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(3.74266046511628_m, 0.993006993006993_m, frc::Rotation2d(-90_deg)),
        reverseConfig
    );
    tempList.push_back(tempTraj);

    tempTraj = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        // Compressed in Y Barrel Run
        frc::Pose2d(3.74266046511628_m, 0.993006993006993_m, frc::Rotation2d(-90_deg)),
        {
            // Slalom
            //frc::Translation2d(0_m, 0_m),
            frc::Translation2d(3.8702511627907_m, 0.272727272727273_m),
            frc::Translation2d(3.9021488372093_m, -0.31048951048951_m),
            frc::Translation2d(4.09353488372093_m, -0.813986013986014_m),
            frc::Translation2d(4.45504186046512_m, -1.07412587412587_m),
            frc::Translation2d(5.19932093023256_m, -1.08251748251748_m),
            frc::Translation2d(5.64588837209302_m, -0.981818181818182_m),
            frc::Translation2d(5.82664186046512_m, -0.662937062937063_m),
            frc::Translation2d(5.92233488372093_m, -0.0083916083916084_m)
            //frc::Translation2d(0_m, 1.52439024390244_m),
            //frc::Translation2d(-0.152439024390244_m, 1.52439024390244_m),
        },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(6.03929302325581_m, 0.986013986013986_m, frc::Rotation2d(90_deg)),
        forwardConfig
    );
    tempList.push_back(tempTraj);

    tempTraj = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        // Compressed in Y Barrel Run
        frc::Pose2d(6.03929302325581_m, 0.986013986013986_m, frc::Rotation2d(90_deg)),
        {
            // Slalom
            //frc::Translation2d(0_m, 0_m),
            frc::Translation2d(6.16688372093023_m, 0.503496503496504_m),
            frc::Translation2d(6.59218604651163_m, 0.216783216783217_m)
            //frc::Translation2d(0_m, 1.52439024390244_m),
            //frc::Translation2d(-0.152439024390244_m, 1.52439024390244_m),
        },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(7.57038139534884_m, -0.067132867132867_m, frc::Rotation2d(180_deg)),
        reverseConfig
    );
    tempList.push_back(tempTraj);
    pathChooser.AddOption("Bounce", tempList);

    tempList.clear();

    frc::SmartDashboard::PutData("Auto Paths", &pathChooser);
}