void run_KineFitting()
{

   std::vector<Double_t> parameters{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   AtTools::AtKinematics kinematics;
   kinematics.KinematicalFit(parameters);
}
